# Tasks

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
- [x] ~~**Performance profiling**~~ — 1000-ball step measured at ~2.6 ms/frame avg; automated benchmark test added
- [x] ~~**FPS counter**~~ — Added FPS + ball count HUD overlay
- [x] ~~**Collision edge cases (glancing, dense stacks)**~~ — Added glancing endpoint, dense column, and spatial grid correctness tests
- [ ] **Wall thickness**: Very fast balls could still tunnel if substeps are too low. Consider CCD (continuous collision detection) for extreme cases
- [ ] **Settling verification**: Extend the restitution-invariance coverage to higher ball counts (500+) that more closely match the full 1000-ball scene
- [ ] **Visual polish**: Ball outlines, restitution slider UI, color scheme options
- [ ] **Screen recording**: Take a screenshot/recording to document visual behavior (requires display-capable environment)
- [ ] **Pair dedup optimization**: The current `unordered_set<pair>` for spatial grid dedup works but could be replaced with a per-frame generation counter on each pair to avoid hash-set overhead
- [ ] **SDL3 video support**: The current environment's SDL3 is built without video support; visual verification requires a display-capable environment
