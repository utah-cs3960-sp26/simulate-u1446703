# Build Instructions

## Prerequisites

- macOS (tested on macOS 15 / Apple Silicon)
- Homebrew
- C++17 compiler (Apple Clang 17+)

## Install Dependencies

```bash
brew install cmake sdl3
```

## Build

```bash
mkdir -p build && cd build
cmake ..
cmake --build .
```

## Run Tests

```bash
cd build
./tests
```

Expected: 29/29 tests pass.

## Run Simulator

```bash
cd build
./simulator [restitution]
```

- `restitution`: float 0.0–1.0 (default 0.3)
- `0.0` = perfectly inelastic, balls stop quickly
- `0.3` = default, nice settling behavior
- `0.9` = very bouncy, takes longer to settle
- The automated test suite verifies both behaviors: lower restitution dissipates energy faster, and different restitution values still converge to the same settled packing footprint in the regression fixture
- The settling regressions now cover both a plain container and a more simulator-like shelf scene with mixed ball radii, so the restitution invariant is checked against more realistic contact geometry
- The collision suite also verifies that equal-mass head-on impacts produce the expected post-collision velocities, which guards the ball-ball restitution math directly
- The wall-contact regressions now also verify exact endpoint overlaps and sealed corner joints, which guards the remaining wall-edge cases directly
- The performance test verifies that the 1000-ball physics step completes in well under 33ms (measured ~2.6 ms avg with spatial hash grid)
- The FPS counter and ball count are displayed in the top-left corner of the window
- Press **ESC** or **Q** to quit

## Examples

```bash
./simulator          # Default restitution 0.3
./simulator 0.0      # Balls stop almost immediately
./simulator 0.9      # Very bouncy
```
