# Build Instructions

## Prerequisites

- C++17 compiler (GCC, Clang, or Apple Clang)
- CMake 3.16+
- SDL3 (built with `SDL_VIDEO=ON` for rendering support)

## Install Dependencies

**macOS (Homebrew)**:
```bash
brew install cmake sdl3
```

**Linux (from source)**:
```bash
# Build SDL3 with video support
git clone https://github.com/libsdl-org/SDL.git --branch release-3.2.4
cd SDL && mkdir build && cd build
cmake .. -DSDL_VIDEO=ON -DSDL_OFFSCREEN=ON
cmake --build . -j$(nproc)
sudo cmake --install .
```

## Build

```bash
cmake -S . -B build
cmake --build build
```

## Run Tests

```bash
./build/tests
```

Expected: **70/70 tests pass**. Includes:
- Vec2 math (6), gravity (2), ball-wall (5), ball-ball (4)
- Restitution behavior (4), energy/settling (3), wall normals (2)
- Collision edge cases (3), CCD (2), performance benchmark (1)
- Large-scale 500-ball (2), full-scale 1000-ball (2)
- CSV I/O (7), ball color (2)
- Sleep system (3): gravity-from-rest wakeup, counter reset, zero-velocity settling
- Contact-aware settling (3): shelf-sliding sleep, stuck detection, full-scale KE=0
- Scene generator (3): grid CSV validation, funnel layout, all-layouts validation
- Edge cases (4): corner collision, narrow channel, small radius, large radius difference
- Pipeline (1): end-to-end headless CSV workflow
- Shared config (1): DefaultPhysicsConfig matches PhysicsConfig defaults
- CSV metadata (1): window dimensions written in save
- Degenerate cases (2): coincident balls, high-speed ball-to-ball
- Color assign pipeline (1): end-to-end color_assign tool test
- CSV roundtrip (1): wall coordinate preservation
- CSV color preservation (3): uncolored roundtrip, 4-column load, mixed colored/uncolored
- Config helper (2): applyDefaultConfig sets all fields, allows override
- Multi-restitution settling (1): KE=0 at r=0.0/0.3/0.9 for 100 balls

## Run Simulator

### Interactive mode
```bash
./build/simulator [restitution]
```

### Headless mode (offscreen rendering + screenshots)
```bash
./build/simulator --headless [restitution] [frames] [screenshot_prefix]
```

- `restitution`: float 0.0–1.0 (default 0.3)
- `frames`: number of frames to simulate (default 600)
- `screenshot_prefix`: path/prefix for BMP files (default "screenshot")

### CSV scene loading and saving
```bash
# Load scene from CSV
./build/simulator --load-csv scene.csv [restitution]

# Save final positions to CSV
./build/simulator --headless --save-csv output.csv [restitution] [frames]

# Load from CSV, simulate, save result
./build/simulator --headless --load-csv input.csv --save-csv output.csv 0.3 600
```

Bundled example:
```bash
./build/simulator --headless \
  --load-csv examples/two_groups_center_funnel.csv \
  --save-csv /tmp/two_groups_center_funnel_settled.csv \
  0.3 600 screenshots/two_groups_center_funnel
```

This example starts two colored ball groups on opposite sides of the default container and uses sloped walls to funnel both groups into a narrow center chute.

### Scene generator tool
```bash
./build/scene_gen <output.csv> [options]

Options:
  --balls N         Number of balls (default: 1000)
  --radius-min R    Minimum ball radius (default: 3.0)
  --radius-max R    Maximum ball radius (default: 6.0)
  --layout TYPE     Layout: grid, rain, funnel, pile (default: grid)
  --width W         Container width (default: 1100)
  --height H        Container height (default: 700)
  --margin M        Wall margin (default: 50)
  --shelves N       Number of angled shelves (default: 2)
  --seed S          Random seed (default: time-based)
```

### Color assignment tool
```bash
./build/color_assign <input.csv> <image> <output.csv> [restitution] [frames]
```

Runs the simulation on the input scene, then assigns each ball a color based on the pixel in the image at the ball's final position. Writes the original scene with new colors to the output CSV.

Supported image formats: BMP, PNG, JPG, TGA, PSD, GIF, HDR, PIC, PNM (via stb_image).

### Controls (interactive mode)
- **SPACE** — Pause / Resume simulation
- **RIGHT** or **N** — Single-step (when paused)
- **UP** / **DOWN** — Speed up / slow down (0.25x–4x)
- **1** — Reset speed to 1x
- **R** — Restart simulation from initial state
- **ESC** or **Q** — Quit
- HUD displays: FPS, ball count, kinetic energy, speed multiplier, pause state, controls help

## Examples

```bash
./build/simulator              # Interactive, restitution 0.3
./build/simulator 0.0          # Interactive, perfectly inelastic
./build/simulator 0.9          # Interactive, very bouncy

# Headless: 800 frames, save screenshots to ./screenshots/
mkdir -p screenshots
./build/simulator --headless 0.3 800 screenshots/sim_r03
# Produces: sim_r03_initial.bmp, sim_r03_bouncing.bmp,
#           sim_r03_settling.bmp, sim_r03_settled.bmp

# Scene generator → simulate → color from image pipeline
./build/scene_gen scene.csv --balls 500 --layout rain --seed 42
./build/simulator --headless --load-csv scene.csv --save-csv settled.csv 0.3 600 screenshots/sim
./build/color_assign scene.csv screenshots/sim_settled.bmp colored.csv 0.3 600
./build/simulator --headless --load-csv colored.csv 0.3 10 screenshots/colored

# Bundled two-group funnel example
./build/simulator --headless --load-csv examples/two_groups_center_funnel.csv 0.3 600 screenshots/two_groups_center_funnel
```

## Performance

- 1000-ball physics step: ~1.8–1.9 ms/frame (includes contact-aware settling)
- 30 FPS budget: 33 ms → ~17× headroom for physics alone
- Spatial hash grid: O(n) broadphase with O(1) clear via generation counter
- CCD: negligible overhead (one dot product per ball-wall pair per substep)
