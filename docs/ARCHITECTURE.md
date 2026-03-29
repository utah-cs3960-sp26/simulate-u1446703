# Architecture

## Project Structure

```
simulate-u1446703/
├── CMakeLists.txt          # Build configuration (6 targets: physics_lib, csv_io_lib, simulator, color_assign, scene_gen, tests)
├── .github/
│   └── workflows/
│       └── ci.yml          # GitHub Actions CI (build, test, headless sim, pipeline)
├── examples/
│   └── two_groups_center_funnel.csv # Example CSV scene with mirrored groups feeding a center funnel
├── include/
│   ├── physics.h           # Core physics types: Vec2, Ball, BallColor, Wall, PhysicsWorld, SpatialGrid
│   ├── sim_config.h        # Shared constants (WINDOW_WIDTH/HEIGHT, DefaultPhysicsConfig) — no SDL dependency
│   ├── renderer.h          # SDL3 renderer wrapper with interactive controls
│   ├── csv_io.h            # CSV scene file I/O (load/save balls + walls)
│   └── stb_image.h         # Single-header image library (PNG/JPG/BMP/TGA support)
├── src/
│   ├── physics.cpp         # Physics engine implementation (CCD, spatial grid, solvers)
│   ├── renderer.cpp        # SDL3 rendering (circles, lines, HUD, screenshots, controls)
│   ├── csv_io.cpp          # CSV scene loading/saving implementation
│   ├── color_assign.cpp    # Color assignment tool (maps final positions to image colors, multi-format)
│   ├── scene_gen.cpp       # Procedural scene generator (grid/rain/funnel/pile layouts)
│   └── main.cpp            # Entry point, scene setup, main loop, headless mode, CSV CLI
├── tests/
│   └── test_physics.cpp    # 64 unit tests (physics + CSV I/O + sleep system + edge cases + pipeline + config)
├── screenshots/            # BMP screenshots from headless runs (gitignored)
├── docs/
│   ├── ARCHITECTURE.md     # This file
│   └── BUILD.md            # Build instructions
└── build/                  # CMake build directory (gitignored)
```

## Design Decisions

### Physics Engine (physics.h / physics.cpp)

- **Substep integration**: Each frame is divided into N substeps (default 8) to prevent tunneling.
- **Iterative constraint solving**: Each substep runs 4 iterations of collision resolution.
- **Position-based correction + impulse**: Overlapping objects are first separated positionally, then velocity impulses are applied. This prevents the "jitter→explode" failure mode.
- **CCD (continuous collision detection)**: After each position integration, a swept-circle-vs-line test checks whether any ball crossed a wall during the substep. If so, the ball is clipped back to the wall surface and its velocity is reflected. This catches extreme-speed tunneling that substeps alone might miss.
- **Correct ball-ball restitution response**: The pair solver computes relative velocity using the standard normal direction (A→B) so approaching balls receive the intended restitution impulse.
- **Endpoint-aware wall contacts**: Exact wall-endpoint overlaps distinguish point contacts from segment interiors, allowing correct reflection at corners.
- **Spatial hash grid with generation counter**: Ball-ball collision uses `SpatialGrid` that buckets balls into uniform cells. Only pairs sharing a cell are tested, reducing average cost from O(n²) to ~O(n). Cell size is auto-tuned to 2× the max ball radius. The grid uses a generation counter for O(1) `clear()` — cells with stale generation are treated as empty on access. Duplicate pairs from overlapping cells are handled by idempotency.
- **Two-phase sleep system**: The sleep threshold uses a two-phase approach:
  - **Phase 1 (never-active balls)**: Balls that have never exceeded the speed threshold get a counter-based delay (`sleepDelay` substeps) before sleep triggers. This gives gravity time to build velocity above the threshold so zero-velocity balls can actually fall.
  - **Phase 2 (previously-active balls)**: Once a ball has exceeded the threshold, `hasBeenActive` is set permanently. Future dips below the threshold trigger instant sleep. This aggressively kills constraint-solver micro-vibrations and prevents perpetual bouncing oscillations in dense stacks.
  - The `hasBeenActive` flag is stored on each `Ball` struct and persists across frames.
- **Contact-aware settling** (iteration 12): Two mechanisms ensure balls fully settle (KE reaches exactly 0):
  - **Contact sleep** (`contactSleepSpeed`): Balls in contact and moving below 40 px/s are zeroed. This simulates static friction and catches shelf-sliding equilibria where gravity's slope component balances damping/friction at a steady-state speed above the normal sleep threshold.
  - **Stuck detection** (`stuckThreshold`): Per-frame position comparison detects balls with high velocity but zero net displacement. These are trapped at terminal velocity (~250 px/s) against a surface — gravity pushes them in, collision correction pushes them back. Zeroed when displacement < 0.1 px and speed > 100 px/s. Previously, such balls would vibrate at 250 px/s indefinitely.
  - Both mechanisms are disabled when `sleepSpeed=0` (for unit tests that need precise velocity tracking).
- **Settling invariant coverage**: Tests verify that restitution affects decay time but not the final packed footprint — at 50, 120, 500, and 1000 ball scales. KE reaches exactly 0 at all restitution values (0.0, 0.3, 0.9).
- **Full-scale 1000-ball tests**: No-overlap, settling-invariance, and KE=0 tests at the actual production ball count.

### Renderer (renderer.h / renderer.cpp)

- Circles drawn as triangle fans via `SDL_RenderGeometry` (16 segments each).
- Precomputed trig tables and static vertex buffer eliminate per-ball heap allocation.
- **Ball outlines**: Each ball is drawn with a dark outline (0.8px border) for visual separation in dense packs.
- Balls colored by speed: blue (slow) → green (medium) → red (fast).
- Walls drawn as thick gray quads (4px).
- **HUD overlay**: FPS, ball count, kinetic energy, speed multiplier, pause state, and controls help text via `SDL_RenderDebugText` (scaled 2×/1.5×).
- `saveScreenshot()` captures the framebuffer to BMP via `SDL_RenderReadPixels` + `SDL_SaveBMP`.
- **Interactive controls** (iteration 13):
  - SPACE: pause/resume simulation
  - RIGHT/N: single-step when paused
  - UP/DOWN: speed multiplier (0.25x–4x)
  - 1: reset speed to 1x
  - R: restart simulation from initial state
  - ESC/Q: quit

### Shared Configuration (sim_config.h)

- **WINDOW_WIDTH / WINDOW_HEIGHT**: Simulation coordinate space constants (1200×800), shared by all tools without SDL dependency.
- **DefaultPhysicsConfig**: Centralized physics defaults (gravity, damping, substeps, etc.) used by both `main.cpp` and `color_assign.cpp` to ensure consistent simulation behavior across tools.
- Extracted in iteration 14 to fix config drift between `color_assign` (which had different damping=0.999, sleepSpeed=2.0, missing solverIterations/bounceThreshold) and the main simulator.

### CSV I/O (csv_io.h / csv_io.cpp)

- **File format**: Single CSV file holds both balls and walls, distinguished by a `type` column.
- Ball rows: `ball,x,y,radius,r,g,b` (color columns optional on load).
- Wall rows: `wall,x1,y1,x2,y2`.
- **Window metadata**: Saved CSV files include a `# Window: WxH` comment with the simulation coordinate space dimensions. This allows `color_assign` and other tools to determine the correct coordinate mapping.
- Comments (lines starting with `#`) and a header row are supported.
- `loadSceneFromCSV()` clears existing world data before loading.
- `saveSceneToCSV()` writes current ball positions and colors.
- Roundtrip tested: save → reload preserves all data within float precision.
- **Bundled example scene**: `examples/two_groups_center_funnel.csv` starts two colored ball packs on the left/right sides of the default container and uses sloped guide walls plus a short center chute to funnel them together.

### Color Assignment Tool (color_assign.cpp)

- Standalone executable: `./color_assign <input.csv> <image> <output.csv> [restitution] [frames]`
- **Multi-format image support** (iteration 13): Uses `stb_image.h` to load BMP, PNG, JPG, TGA, PSD, GIF, HDR, PIC, PNM. No SDL dependency required for this tool.
- Phase 1: Load initial scene from CSV, save original ball positions.
- Phase 2: Run physics simulation for specified frames until balls settle.
- Phase 3: Load image, sample pixel color at each ball's final world position (scaled proportionally if image size differs from window).
- Phase 4: Write output CSV with original starting positions but colors sampled from the image at final positions.

### Scene Setup (main.cpp)

- Rectangular container with two angled shelves for visual interest.
- 1000 balls placed in a grid with slight random offsets and velocities.
- Restitution configurable via command-line argument: `./simulator [restitution]`
- **CSV loading**: `--load-csv <file>` loads scene from CSV instead of generating.
- **CSV saving**: `--save-csv <file>` saves final ball positions after simulation.
- **Headless mode**: `./simulator --headless [restitution] [frames] [prefix]` runs for a fixed number of frames using the offscreen video driver, saving BMP screenshots at key moments (initial, bouncing, settling, settled).

## Key Classes

| Class | File | Purpose |
|-------|------|---------|
| `Vec2` | physics.h | 2D vector math (add, sub, dot, normalize, etc.) |
| `BallColor` | physics.h | RGB color + hasColor flag for persistent ball coloring |
| `Ball` | physics.h | Circular body with position, velocity, radius, mass, color, sleep state |
| `Wall` | physics.h | Immovable line segment with outward normal |
| `PhysicsConfig` | physics.h | Simulation parameters (gravity, restitution, substeps, etc.) |
| `SpatialGrid` | physics.h/cpp | Spatial hash grid with generation counter for broadphase |
| `CellKey` / `CellKeyHash` | physics.h | Grid cell coordinate + hash for unordered_map |
| `CellData` | physics.h | Per-cell ball indices + generation stamp |
| `PhysicsWorld` | physics.h/cpp | Owns balls+walls, runs simulation step with CCD |
| `DefaultPhysicsConfig` | sim_config.h | Centralized physics defaults shared across simulator and tools |
| `Renderer` | renderer.h/cpp | SDL3 window, drawing, event handling, FPS HUD, screenshots |
| `loadSceneFromCSV` | csv_io.h/cpp | Load balls + walls from CSV file |
| `saveSceneToCSV` | csv_io.h/cpp | Save current scene to CSV file |
| `splitCSVLine` | csv_io.h/cpp | Parse CSV line into trimmed tokens |

## Collision Resolution Algorithm

### Ball-Wall (with CCD)
1. **CCD pass** (in `integratePositions`): For each ball, swept-circle-vs-line test against all walls. If the ball's trajectory this substep crosses a wall, clip position back to the contact point and reflect velocity.
2. **Overlap resolution** (in `solveBallWallCollisions`): Find closest point on wall segment to ball center. Distinguish segment interiors from clamped endpoints. If distance < radius: push ball out along normal, reflect velocity with restitution.

### Ball-Ball
1. Populate spatial hash grid with all ball bounding boxes
2. For each unique pair sharing a grid cell, check center-to-center distance vs sum of radii
3. If overlapping: push apart proportional to inverse mass
4. Compute B-relative-to-A velocity along the A→B collision normal and apply the restitution impulse only while the pair is closing
5. Apply tangential friction (Coulomb model)

## Performance

| Metric | Value |
|--------|-------|
| 1000-ball physics step | ~1.8–1.9 ms/frame |
| Target frametime (30 FPS) | 33 ms |
| Headroom | ~17× |
| Spatial grid clear | O(1) via generation counter |
| Ball-ball broadphase | O(n) average via spatial hash |
| KE convergence (r=0.0) | 0 by frame ~300 |
| KE convergence (r=0.9) | 0 by frame ~360 |
