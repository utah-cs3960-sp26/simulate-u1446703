# Architecture

## Project Structure

```
simulate-u1446703/
├── CMakeLists.txt          # Build configuration
├── include/
│   ├── physics.h           # Core physics types: Vec2, Ball, Wall, PhysicsWorld
│   └── renderer.h          # SDL3 renderer wrapper
├── src/
│   ├── physics.cpp         # Physics engine implementation
│   ├── renderer.cpp        # SDL3 rendering (circles, lines)
│   └── main.cpp            # Entry point, scene setup, main loop
├── tests/
│   └── test_physics.cpp    # 29 unit tests for physics engine
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
- **Correct ball-ball restitution response**: The pair solver now computes relative velocity using the standard normal direction (A→B) so approaching balls receive the intended restitution impulse instead of only positional separation.
- **Endpoint-aware wall contacts**: Exact wall-endpoint overlaps now distinguish point contacts from segment interiors so a ball resting exactly on a corner can reflect away from the endpoint instead of only mirroring the segment axis.
- **Spatial hash grid**: Ball-ball collision uses a spatial hash grid (`SpatialGrid` in physics.h) that buckets balls into uniform cells. Only pairs sharing a cell are tested, reducing average cost from O(n²) to ~O(n). Cell size is auto-tuned to 2× the max ball radius. A pair-visited set prevents duplicate resolution. With 1000 balls the physics step averages ~2.6 ms/frame.
- **Sleep threshold**: Balls below a velocity threshold are zeroed out to help convergence.
- **Settling invariant coverage**: The regression suite checks that changing restitution affects decay time, but not the final packed footprint of a settled pile.
- **Shelf-scene settling coverage**: The regression suite now also checks that the same restitution invariant holds in a more simulator-like scene with internal shelves and mixed-radius balls.
- **Momentum-transfer regression coverage**: The tests now assert post-collision velocities for equal-mass head-on impacts so future refactors cannot silently break restitution handling.
- **Wall-joint regression coverage**: The test suite now covers exact endpoint overlaps and sealed corner joints so wall-contact edge cases are exercised as directly as the ball-ball impulse path.

### Renderer (renderer.h / renderer.cpp)

- Circles drawn as triangle fans via `SDL_RenderGeometry` (16 segments each).
- Balls colored by speed: blue (slow) → green (medium) → red (fast).
- Walls drawn as white lines.
- FPS + ball count HUD overlay via `SDL_RenderDebugText` (scaled 2×).

### Scene Setup (main.cpp)

- Rectangular container with two angled shelves for visual interest.
- 1000 balls placed in a grid with slight random offsets and velocities.
- Restitution configurable via command-line argument: `./simulator [restitution]`
- The new shelf-scene regression mirrors this layout style in deterministic form so the automated suite covers geometry closer to the real app scene, not just a plain rectangular box.

## Key Classes

| Class | File | Purpose |
|-------|------|---------|
| `Vec2` | physics.h | 2D vector math (add, sub, dot, normalize, etc.) |
| `Ball` | physics.h | Circular body with position, velocity, radius, mass |
| `Wall` | physics.h | Immovable line segment with outward normal |
| `PhysicsConfig` | physics.h | Simulation parameters (gravity, restitution, substeps, etc.) |
| `SpatialGrid` | physics.h/cpp | Spatial hash grid for broadphase ball-ball collision |
| `CellKey` / `CellKeyHash` | physics.h | Grid cell coordinate + hash for unordered_map |
| `PhysicsWorld` | physics.h/cpp | Owns balls+walls, runs simulation step |
| `Renderer` | renderer.h/cpp | SDL3 window, drawing, event handling, FPS HUD |

## Collision Resolution Algorithm

### Ball-Wall
1. Find closest point on wall segment to ball center
2. Distinguish segment interiors from clamped endpoints so exact corner-point contacts can choose a valid point-contact normal
3. If distance < radius: push ball out along normal, reflect velocity with restitution

### Ball-Ball
1. Populate spatial hash grid with all ball bounding boxes
2. For each unique pair sharing a grid cell, check center-to-center distance vs sum of radii
3. If overlapping: push apart proportional to inverse mass
4. Compute B-relative-to-A velocity along the A→B collision normal and apply the restitution impulse only while the pair is closing
5. Apply tangential friction (Coulomb model)
