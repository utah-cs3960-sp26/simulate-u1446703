# Agent Progress Log

## Iteration 1 — 2026-03-18 (Claude Opus 4.6)

### What was done
Built the entire 2D physics simulator from scratch:

1. **Dependencies**: Installed SDL3 3.4.2 and cmake 4.2.3 via homebrew
2. **Project structure**: CMakeLists.txt with three targets (physics_lib, simulator, tests)
3. **Physics engine** (`include/physics.h`, `src/physics.cpp`):
   - `Vec2` math helper
   - `Ball` (circular body with pos, vel, radius, mass)
   - `Wall` (immovable line segment)
   - `PhysicsWorld` with substep integration + iterative constraint solving
   - Ball-wall collision: closest-point-on-segment + push-out + restitution impulse
   - Ball-ball collision: position correction (inverse mass weighted) + impulse + friction
   - Sleep threshold to zero out near-stopped balls
4. **Renderer** (`include/renderer.h`, `src/renderer.cpp`):
   - SDL3 window + renderer
   - Balls as triangle-fan circles, colored by speed
   - Walls as white lines
5. **Scene** (`src/main.cpp`):
   - 1000 balls in a rectangular container with two angled shelves
   - Restitution configurable via CLI argument
6. **Tests** (`tests/test_physics.cpp`): 20 tests, all passing:
   - Vec2 math (6 tests)
   - Gravity (2 tests)
   - Ball-wall collision (3 tests)
   - Ball-ball collision (3 tests)
   - Restitution behavior (2 tests)
   - Energy dissipation (1 test)
   - Settling/stacking (1 test)
   - Wall normals (2 tests)
7. **Visual verification**: Launched simulator with restitution 0.0, 0.3, 0.9 — all ran without crashes

### Current state
- Code builds cleanly, 20/20 tests pass
- Simulator runs with ~1000 balls, configurable restitution
- All source files are well-commented for agent handoff

### What the next iteration should focus on
- Run the simulator and visually confirm balls don't overlap/phase through walls
- Performance testing — is 1000 balls smooth at 60fps?
- Consider spatial hash grid if performance is an issue
- Add more edge-case tests (e.g., balls at exact wall corners)
- Add FPS counter or performance overlay
- See TASKS.md for full list of future work

## Iteration 2 — 2026-03-22 (GPT-5 Codex)

### What was done
Focused this iteration on converting a previously manual physics requirement into an automated regression check:

1. **Baseline verification**:
   - Reconfigured and rebuilt the project with CMake
   - Reran the existing unit suite and confirmed the baseline passed before changes
   - Probed the settling fixture at restitution `0.0`, `0.3`, and `0.9` to measure the final packed bounds before modifying code
2. **Regression coverage** (`tests/test_physics.cpp`):
   - Added shared settling helpers that build a deterministic container-and-ball fixture
   - Added `restitution_changes_decay_not_final_packed_size`
   - The new test verifies three things:
     - low restitution still settles faster
     - low, medium, and high restitution all reach a fully stopped state
     - the final settled width, height, and top-of-pile position stay effectively the same across restitution values
3. **Documentation updates**:
   - Updated `docs/BUILD.md` to reflect the expanded test count and the new settling-invariance guarantee
   - Updated `docs/ARCHITECTURE.md` to record that the regression suite now checks the packed-footprint invariant
4. **Planning / handoff updates**:
   - Updated `TASKS.md` with an iteration-specific checklist and marked the restitution-invariance work complete

### Verification performed
- `cmake -S . -B build`
- `cmake --build build`
- `./tests` → `21/21` passed
- Attempted normal simulator launch with `./simulator 0.3`
  - Result: SDL failed to initialize a real display in this environment (`The video driver did not add any displays`)
- Performed headless startup verification with `SDL_VIDEODRIVER=dummy ./simulator 0.3`
  - Result: simulator started successfully and entered the main loop for the timed run window without crashing

### Current state
- The project now has automated coverage for the key requirement that restitution changes settling speed but not the final occupied resting footprint
- Docs and task tracking reflect the new regression coverage
- The simulator still cannot be visually inspected in this headless environment without an attached display, so true on-screen validation remains an environment-dependent follow-up

### What the next iteration should focus on
- Extend the new settling-invariance coverage to more complex scenes, especially shelf geometry and mixed-radius packs
- Add corner-case collision tests where balls contact wall endpoints or multiple constraints simultaneously
- Investigate performance instrumentation so future iterations can measure whether the current O(n^2) solver remains pleasant with ~1000 balls

## Iteration 3 — 2026-03-22 (GPT-5 Codex)

### What was done
Focused this iteration on a real collision-response defect in the ball-ball solver:

1. **Baseline verification**:
   - Reconfigured and rebuilt the project with CMake
   - Reran the unit suite before making changes to confirm the current baseline still passed
   - Re-ran the simulator launch path in this environment to confirm the same SDL display limitation still applied
2. **Physics fix** (`src/physics.cpp`):
   - Traced the ball-ball impulse code and found the relative-velocity sign was inverted
   - Corrected the solver to use the standard A→B collision normal with B-relative-to-A velocity
   - Updated both the normal impulse and tangential friction applications so equal-mass collisions now exchange momentum according to restitution instead of relying mostly on positional correction
3. **Regression coverage** (`tests/test_physics.cpp`):
   - Strengthened `head_on_collision_conserves_direction` so it now asserts the actual post-collision velocities
   - Added `inelastic_head_on_collision_reduces_relative_speed` to verify that restitution controls the separating speed after impact
4. **Documentation / handoff**:
   - Updated `TASKS.md`, `docs/ARCHITECTURE.md`, and `docs/BUILD.md` to reflect the solver correction and expanded regression coverage
   - Appended this handoff entry for the next agent iteration

### Verification performed
- `cmake -S . -B build`
- `cmake --build build`
- `./build/tests` → `22/22` passed
- `./build/tests | rg 'inelastic_head_on_collision_reduces_relative_speed|Results'`
  - Result: confirmed the new regression is executing and the suite reports `22/22 passed`
- `./build/simulator 0.3`
  - Result: SDL still fails in this headless environment with `SDL_Init failed: The video driver did not add any displays`
- `SDL_VIDEODRIVER=dummy perl -e 'alarm 2; exec @ARGV' ./build/simulator 0.3`
  - Result: process stayed alive until the timed alarm terminated it (`exit_code:142`), which is consistent with a successful headless startup and main-loop run in this environment

### Current state
- Ball-ball collisions now apply restitution impulses in the correct direction, so collisions transfer momentum instead of mostly relying on positional separation
- The regression suite now covers the actual post-impact velocities for head-on ball-ball collisions
- The project still cannot be visually inspected on-screen in this environment without a usable display, but headless startup remains viable

### What the next iteration should focus on
- Add wall-endpoint and corner-contact regressions so the remaining collision edge cases are covered as directly as the ball-ball impulse path is now
- Measure frame time or add lightweight instrumentation to understand how close the current O(n^2) solver is to the “pleasant to look at” target with ~1000 balls
- If a display-capable environment becomes available, perform a true visual verification pass and capture screenshots or recordings for documentation

## Iteration 4 — 2026-03-22 (GPT-5 Codex)

### What was done
Focused this iteration on the remaining wall-contact edge cases:

1. **Baseline verification**:
   - Reconfigured and rebuilt the project with CMake
   - Reran the full unit suite before editing the solver to confirm the pre-change baseline
2. **Regression coverage** (`tests/test_physics.cpp`):
   - Added `ball_bounces_off_wall_endpoint` to exercise the exact wall-endpoint overlap path directly
   - Added `ball_remains_outside_corner_joint` to verify a ball cannot leak through a two-wall corner joint while being driven into it
3. **Physics fix** (`src/physics.cpp`):
   - Updated `solveBallWallCollisions()` to track the clamped segment parameter for each closest-point query
   - Distinguished exact endpoint contacts from exact segment-interior contacts
   - When a ball center lands exactly on an endpoint, the solver now derives a point-contact normal from the incoming velocity instead of always falling back to the segment normal
   - This preserves the existing interior-wall behavior while fixing the corner-point reflection case
4. **Documentation / handoff**:
   - Updated `TASKS.md`, `docs/ARCHITECTURE.md`, and `docs/BUILD.md` to record the new endpoint-aware wall handling and the expanded regression suite
   - Appended this handoff entry for the next iteration

### Verification performed
- `cmake -S . -B build`
- `cmake --build build`
- `./build/tests` → `24/24` passed
- `./build/simulator 0.3`
  - Result: SDL still fails in this headless environment with `SDL_Init failed: The video driver did not add any displays`
- `SDL_VIDEODRIVER=dummy perl -e 'alarm 2; exec @ARGV' ./build/simulator 0.3`
  - Result: process stayed alive until the timed alarm terminated it (`exit_code:142`), which is consistent with a successful headless startup and main-loop run in this environment

### Current state
- The wall solver now handles exact endpoint overlaps as point contacts instead of treating them like ordinary segment hits
- The regression suite now covers both exact endpoint resolution and sealed corner-joint containment
- The project still cannot be visually inspected on-screen in this environment without a usable display, but headless startup remains viable

### What the next iteration should focus on
- Extend collision-edge coverage from exact endpoints/corners to glancing endpoint impacts, shelf corners, and dense multi-contact stacks
- Add lightweight timing instrumentation or profiling so future iterations can quantify whether the current O(n^2) solver is pleasant with ~1000 balls
- If a display-capable environment becomes available, perform a true visual verification pass and capture screenshots or recordings for documentation

## Iteration 5 — 2026-03-22 (GPT-5 Codex)

### What was done
Focused this iteration on turning a successful manual shelf-scene probe into permanent regression coverage:

1. **Baseline verification**:
   - Reused the current build directory and reran the full unit suite before extending the tests
   - Confirmed the pre-change baseline was still green before adding more expensive settling coverage
2. **Regression coverage** (`tests/test_physics.cpp`):
   - Added deterministic shelf-scene helpers that mirror the simulator's container-with-shelves layout without relying on SDL or randomness
   - Added `restitution_preserves_final_packed_size_in_shelf_scene`
   - The new regression uses mixed ball radii and internal shelves to verify that restitution still changes transient behavior without changing the final occupied footprint in a more realistic scene
3. **Documentation / tracking**:
   - Updated `TASKS.md` with the completed iteration checklist and refreshed the remaining future-work items
   - Updated `docs/ARCHITECTURE.md` and `docs/BUILD.md` to reflect the new shelf-scene settling coverage and the increased test count
4. **Handoff hygiene**:
   - Appended this log entry so the next iteration can immediately see what changed and what remains open

### Verification performed
- `cmake --build build`
- `./build/tests` → `25/25` passed
- `./build/simulator 0.3`
  - Result: SDL still fails in this headless environment with `SDL_Init failed: The video driver did not add any displays`
- `SDL_VIDEODRIVER=dummy perl -e 'alarm 2; exec @ARGV' ./build/simulator 0.3`
  - Result: process stayed alive until the timed alarm terminated it (`exit_code:142`), which is consistent with a successful headless startup and main-loop run in this environment

### Current state
- The regression suite now checks the restitution-invariant final packing footprint in both a simple box and a more simulator-like shelf scene with mixed-radius balls
- The project builds cleanly and the physics suite now reports `25/25` passing tests
- On-screen validation is still blocked by the lack of a usable display in this environment, but headless startup remains viable

### What the next iteration should focus on
- Extend collision-edge coverage to glancing endpoint impacts, shelf corners, and dense multi-contact stacks where several constraints resolve in the same substep
- Add lightweight timing instrumentation or profiling so future iterations can quantify whether the current O(n^2) solver remains pleasant with ~1000 balls
- If a display-capable environment becomes available, perform a true visual verification pass and capture screenshots or recordings for documentation
