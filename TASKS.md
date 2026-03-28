# Tasks

## Completed — Iteration 13 (2026-03-28)

- [x] Fix broken git repository (alternates pointing to wrong path after iteration 12 workaround)
- [x] Add GitHub Actions CI workflow (`.github/workflows/ci.yml`):
  - Builds SDL3 from source, configures project, runs all tests
  - Runs headless simulation and verifies KE reaches 0
  - Tests scene_gen pipeline end-to-end
  - Uploads screenshots as artifacts
- [x] Add multi-format image support to `color_assign` via `stb_image.h`:
  - Supports PNG, JPG, BMP, TGA, PSD, GIF, HDR, PIC, PNM
  - Removed SDL3 dependency from color_assign (pure C++ + stb_image)
- [x] Add interactive keyboard controls to simulator:
  - SPACE: pause/resume, RIGHT/N: single-step, UP/DOWN: speed (0.25x–4x)
  - 1: reset speed, R: restart simulation, ESC/Q: quit
  - HUD shows KE, speed multiplier, pause state, controls help
- [x] Add 7 new tests (51→58):
  - `ball_in_corner_gets_pushed_out`: ball jammed in L-corner resolves cleanly
  - `many_balls_in_narrow_channel`: 50 balls in 30px-wide channel, no overlaps
  - `zero_radius_ball_does_not_crash`: very small ball doesn't produce NaN/Inf
  - `csv_save_preserves_ball_color_flag`: color roundtrip through CSV
  - `scene_gen_all_layouts_produce_valid_output`: all 4 layouts produce loadable CSV
  - `spatial_grid_handles_large_radius_difference`: mixed 2px/20px radius collision
  - `headless_csv_pipeline_end_to_end`: full scene_gen → headless → CSV → reload
- [x] Verify all 58/58 tests pass
- [x] Run headless simulation — KE=0 by frame ~240 at r=0.3
- [x] Update documentation (ARCHITECTURE.md, BUILD.md, TASKS.md, AGENT-PROGRESS.md)

## Completed — Iteration 12 (2026-03-28)

- [x] Diagnose residual KE plateauing at non-zero values (860K at r=0.0, 995K at r=0.3)
  - Root cause 1: shelf-sliding equilibrium — gravity's slope component balances damping/friction at ~30 px/s
  - Root cause 2: terminal velocity trapping — balls at ~250 px/s against pile, zero net displacement
- [x] Implement contact-aware settling system with two mechanisms:
  - Contact sleep (`contactSleepSpeed=40`): balls in resting contact below threshold zeroed (static friction)
  - Stuck detection (`stuckThreshold=0.1`): per-frame position comparison catches zero-displacement balls
  - Added `inRestingContact`, `prevPos` fields to Ball; `contactSleepSpeed`, `stuckThreshold` to PhysicsConfig
  - Both disabled when `sleepSpeed=0` (for unit tests with precise velocity tracking)
- [x] KE now reaches exactly 0 at all restitution values: r=0.0 by frame 300, r=0.3 by frame 300, r=0.9 by frame 360
- [x] Add ball outlines (0.8px dark border) for visual separation in dense packs
- [x] Add 5 new tests (46→51):
  - `contact_sleep_stops_shelf_sliding`: shelf-sliding balls settle to KE=0
  - `stuck_detection_catches_terminal_velocity_ball`: fast ball against pile zeroed
  - `full_scale_settles_to_zero_ke`: 500 balls reach KE=0 at r=0.0/0.3/0.9
  - `scene_gen_grid_produces_valid_csv`: scene_gen → CSV → loadScene roundtrip
  - `scene_gen_funnel_layout`: funnel layout with extra walls validated
- [x] Adjust settling-invariance test tolerances (15→25-30px) and frames for contact-aware dynamics
- [x] Verify all 51/51 tests pass
- [x] Run headless simulations at r=0.0, 0.3, 0.9 — all reach KE=0
- [x] End-to-end pipeline verified: scene_gen → simulator → CSV save
- [x] Update documentation (ARCHITECTURE.md, BUILD.md, TASKS.md, AGENT-PROGRESS.md)

## Completed — Iteration 11 (2026-03-28)

- [x] Implement two-phase sleep system to fix gravity-vs-sleep issue
  - Phase 1 (never-active): counter-based delay gives gravity `sleepDelay` substeps to build velocity
  - Phase 2 (previously-active): instant sleep kills solver micro-vibrations
  - Added `sleepCounter` and `hasBeenActive` fields to `Ball` struct
  - Added `sleepDelay` config parameter (default 8, matching substeps)
- [x] Add 3 new tests (43→46): gravity_wakes_zero_velocity_balls, sleep_counter_resets_on_fast_motion, settling_with_zero_initial_velocity
- [x] Increase shelf scene settling test frames (4000→5000) for counter-based sleep convergence
- [x] Verify all 46/46 tests pass
- [x] Run headless simulations at restitution 0.0, 0.3, 0.9 — all produce valid results
- [x] Update documentation (ARCHITECTURE.md, BUILD.md, TASKS.md, AGENT-PROGRESS.md)

## Completed — Iteration 10 (2026-03-25)

- [x] Diagnose 5 failing physics settling tests (38/43 passing at start of iteration)
- [x] Root cause: iteration 9 moved `applySleepThreshold()` from per-substep to per-frame, allowing energy accumulation in dense packings via constraint solver residuals
- [x] Fix: restore `applySleepThreshold()` to run inside the substep loop (per-substep), while also keeping it at frame-end
- [x] Verify all 43/43 tests pass after fix
- [x] Verify headless simulation works (KE decreases from ~850M peak to ~37M after 200 frames)
- [x] Update documentation (ARCHITECTURE.md, TASKS.md, AGENT-PROGRESS.md)

## Completed — Iteration 9 (2026-03-23)

- [x] Add `BallColor` struct and `color` field to `Ball` for persistent ball coloring
- [x] Update renderer to use ball's assigned color when `hasColor` is true
- [x] Implement `csv_io.h`/`csv_io.cpp`: `loadSceneFromCSV()`, `saveSceneToCSV()`, `splitCSVLine()`
- [x] CSV format supports balls (type,x,y,radius,r,g,b) and walls (type,x1,y1,x2,y2), comments, headers
- [x] Add `--load-csv` and `--save-csv` CLI options to simulator (both headless and interactive)
- [x] Create `color_assign` tool: loads CSV + BMP, simulates, samples image at final positions, writes colored CSV
- [x] Add `csv_io_lib` static library and `color_assign` executable to CMakeLists.txt
- [x] Add 8 new tests (35→43): CSV split, load/save roundtrip, comments, missing file, ball color default
- [x] Build, verify 43/43 tests pass, headless CSV workflow verified end-to-end
- [x] Update documentation (ARCHITECTURE.md, BUILD.md, AGENTS.md, AGENT-PROGRESS.md, TASKS.md)

## Completed — Iteration 8 (2026-03-23)

- [x] Rebuild SDL3 with video support enabled (`SDL_VIDEO=ON`)
- [x] Add `--headless` mode with BMP screenshot capture at key simulation moments
- [x] Add `saveScreenshot()` to Renderer using `SDL_RenderReadPixels` + `SDL_SaveBMP`
- [x] Implement CCD (continuous collision detection) — swept-circle-vs-line in `integratePositions()`
- [x] Add 2 CCD tests: `ccd_prevents_fast_ball_tunneling`, `ccd_works_with_angled_walls`
- [x] Add 1000-ball full-scale tests: `full_scale_1000_balls_no_overlap_after_settling`, `full_scale_1000_balls_restitution_invariance`
- [x] Optimize spatial grid: O(1) `clear()` via generation counter (replaces per-cell iteration)
- [x] Capture screenshots at restitution 0.0, 0.3, 0.9 — verified rendering and settling behavior
- [x] Build, verify 35/35 tests pass, performance still 0.8–0.9 ms/frame
- [x] Update documentation (ARCHITECTURE.md, BUILD.md, AGENT-PROGRESS.md, TASKS.md)

## Completed — Iteration 7 (2026-03-23)

- [x] Build and verify 29/29 baseline tests pass
- [x] Remove pair-dedup `unordered_set` — rely on idempotent resolution instead (0.8 ms/frame, down from 2.4 ms)
- [x] Optimize renderer: precomputed trig tables + static vertex buffer (no per-ball heap allocation)
- [x] Add `large_scale_no_overlap_after_settling` test (500 balls, no overlaps, all contained)
- [x] Add `large_scale_restitution_preserves_packed_size` test (500 balls, settling invariance)
- [x] Build, verify 31/31 tests pass, confirm simulator startup (headless)
- [x] Update documentation (ARCHITECTURE.md, BUILD.md, AGENT-PROGRESS.md, TASKS.md)

## Completed — Iteration 6 (2026-03-23)

- [x] Build and verify 25/25 baseline tests pass
- [x] Implement spatial hash grid for O(n) ball-ball collision detection
- [x] Add FPS counter + ball count HUD overlay to renderer
- [x] Add 4 new tests: glancing endpoint impact, dense column stack, spatial grid correctness, 1000-ball performance benchmark
- [x] Build, verify 29/29 tests pass, confirm simulator startup
- [x] Update documentation (ARCHITECTURE.md, BUILD.md, AGENT-PROGRESS.md, TASKS.md)

## Completed — Iteration 5 (2026-03-22)

- [x] Rebuild and rerun the simulator test suite to re-establish a clean baseline before extending settling coverage
- [x] Convert the shelf-geometry settling probe into an automated mixed-radius regression
- [x] Rebuild, rerun tests, and relaunch the simulator in this environment for verification
- [x] Update documentation and append the handoff details to `AGENT-PROGRESS.md`

## Completed — Iteration 4 (2026-03-22)

- [x] Rebuild and rerun the simulator test suite to establish a clean baseline before touching wall contacts
- [x] Add explicit regression coverage for wall-endpoint overlap resolution and corner-joint containment
- [x] Fix the wall solver so exact endpoint contacts use a point-contact normal instead of only the segment normal
- [x] Rebuild, rerun tests, and relaunch the simulator in this environment for verification
- [x] Update documentation and append the handoff details to `AGENT-PROGRESS.md`

## Completed — Iteration 3 (2026-03-22)

- [x] Rebuild and rerun the test suite to re-establish a clean baseline before physics changes
- [x] Investigate ball-ball collision response to confirm whether restitution impulses are being applied correctly
- [x] Fix the ball-ball impulse math so approaching balls exchange momentum according to restitution
- [x] Strengthen collision regression coverage to assert post-impact velocities, not just separation
- [x] Rebuild, rerun tests, and relaunch the simulator in this environment for verification
- [x] Update documentation and append the handoff details to `AGENT-PROGRESS.md`

## Completed — Iteration 2 (2026-03-22)

- [x] Rebuild the project and rerun the existing test suite to establish a clean baseline
- [x] Probe settling behavior across multiple restitution values to verify whether packed resting height already matches
- [x] Add an automated regression test that proves different restitution values converge to the same settled packing footprint
- [x] Update project documentation to describe the new settling-invariance verification
- [x] Launch the simulator again for manual verification after the test/documentation changes
- [x] Append this iteration's handoff details to `AGENT-PROGRESS.md`

## Completed — Iteration 1 (2026-03-18)

- [x] Install SDL3 and cmake via homebrew
- [x] Set up CMake project structure (src/, include/, tests/)
- [x] Implement core physics engine (Ball, Wall, PhysicsWorld)
- [x] Implement SDL3 renderer and scene setup (~1000 balls)
- [x] Write 20 unit tests for physics engine — all passing
- [x] Build and verify: simulator runs, tests pass
- [x] Visual verification: simulator launched with restitution 0.0, 0.3, 0.9
- [x] Documentation: ARCHITECTURE.md, BUILD.md

## Known Issues / Future Work

- [x] ~~**Spatial partitioning**: O(n²) ball-ball collision~~ — Implemented spatial hash grid in iteration 6
- [x] ~~**Performance profiling**~~ — 1000-ball step measured at ~0.8 ms/frame avg; idempotent spatial grid, no hash-set overhead
- [x] ~~**FPS counter**~~ — Added FPS + ball count HUD overlay
- [x] ~~**Collision edge cases (glancing, dense stacks)**~~ — Added glancing endpoint, dense column, and spatial grid correctness tests
- [x] ~~**Wall thickness / CCD**~~ — Implemented swept-circle CCD in iteration 8; fast balls no longer tunnel through walls
- [x] ~~**Settling verification**~~ — Full 1000-ball no-overlap and settling-invariance tests added in iteration 8
- [x] ~~**Pair dedup optimization**~~ — Removed hash-set; idempotent resolution handles duplicates (iteration 7)
- [x] ~~**SDL3 video support**~~ — Rebuilt SDL3 with `SDL_VIDEO=ON` in iteration 8; offscreen rendering + BMP screenshots now work
- [x] ~~**Screen recording**~~ — Headless screenshot capture implemented in iteration 8; screenshots at 3 restitution values saved
- [x] ~~**Spatial grid clear optimization**~~ — Generation counter replaces per-cell iteration (iteration 8)
- [x] ~~**CSV scene I/O**~~ — Load/save balls+walls from CSV; supports colors, comments, headers (iteration 9)
- [x] ~~**Color assignment tool**~~ — `color_assign` maps final ball positions to BMP image colors (iteration 9)
- [x] ~~**Gravity-vs-sleep fix**~~ — Two-phase sleep system: counter-based delay for rest-start balls, instant sleep for active balls (iteration 11)
- [x] ~~**High-restitution residual bouncing**~~ — Contact-aware settling: contact sleep + stuck detection ensures KE=0 at all restitution values (iteration 12)
- [x] ~~**Visual polish: Ball outlines**~~ — Dark 0.8px outline around each ball for visual separation (iteration 12)
- [x] ~~**CSV scene generator**~~ — `scene_gen` tool with grid/rain/funnel/pile layouts (iteration 10/12; tests added in 12)
- [x] ~~**PNG image support**~~ — `color_assign` now uses stb_image: supports PNG, JPG, BMP, TGA, and more (iteration 13)
- [x] ~~**CI workflow**~~ — GitHub Actions CI: build, test, headless sim, pipeline validation (iteration 13)
- [x] ~~**Interactive controls**~~ — Pause/resume, single-step, speed adjustment (0.25x–4x), restart (iteration 13)
- [ ] **Visual polish**: Restitution slider UI, color scheme options
- [ ] **SIMD vectorization**: Consider SIMD for the physics step inner loops
- [ ] **Interactive display**: Need an environment with a real display server (X11/Wayland) for interactive mode
- [ ] **Git push**: Need GitHub credentials configured to push (SSH keys, gh CLI, or .netrc)
