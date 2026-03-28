// main.cpp — Entry point for the 2D physics simulator.
//
// Sets up a scene with ~1000 balls inside a walled container,
// then runs the simulation loop. Restitution can be configured
// via command-line argument (default 0.3).
//
// Usage: ./simulator [restitution]
//        ./simulator --headless [restitution] [frames] [screenshot_prefix]
//        ./simulator --load-csv input.csv [--save-csv output.csv] [restitution]
//        ./simulator --headless --load-csv input.csv --save-csv output.csv [restitution] [frames]
//
//   restitution: float in [0, 1], default 0.3
//   --headless:  Run for a fixed number of frames using the offscreen driver,
//                saving BMP screenshots at key moments (start, mid, settled).
//   --load-csv:  Load initial scene from a CSV file instead of generating one.
//   --save-csv:  Save final ball positions to a CSV file after simulation.
//   frames:      Number of frames to simulate in headless mode (default 600).
//   screenshot_prefix: Prefix for screenshot filenames (default "screenshot").

#include "physics.h"
#include "renderer.h"
#include "csv_io.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <cstring>
#include <vector>

// ── Scene setup constants ───────────────────────────────────────────
constexpr int   NUM_BALLS       = 1000;
constexpr float BALL_RADIUS_MIN = 3.0f;
constexpr float BALL_RADIUS_MAX = 6.0f;
constexpr float WALL_MARGIN     = 50.0f;  // Distance from window edge to walls

// ── Random float in [lo, hi] ───────────────────────────────────────
static float randFloat(float lo, float hi) {
    return lo + static_cast<float>(rand()) / RAND_MAX * (hi - lo);
}

// ── Build the container walls ───────────────────────────────────────
// Creates a rectangular box with a funnel at the top so balls spread out.
static void setupWalls(PhysicsWorld& world) {
    float left   = WALL_MARGIN;
    float right  = WINDOW_WIDTH - WALL_MARGIN;
    float top    = WALL_MARGIN;
    float bottom = WINDOW_HEIGHT - WALL_MARGIN;

    // Main rectangular container
    world.walls.push_back(Wall({left,  top},    {right, top}));     // Top
    world.walls.push_back(Wall({right, top},    {right, bottom})); // Right
    world.walls.push_back(Wall({right, bottom}, {left,  bottom})); // Bottom
    world.walls.push_back(Wall({left,  bottom}, {left,  top}));    // Left

    // Angled shelves to make the simulation visually interesting
    // These create ramps for balls to bounce off of
    float midX = (left + right) / 2.0f;
    float shelfY1 = top + (bottom - top) * 0.35f;
    float shelfY2 = top + (bottom - top) * 0.6f;

    // Left shelf (angled down-right)
    world.walls.push_back(Wall({left, shelfY1}, {midX - 40, shelfY1 + 50}));
    // Right shelf (angled down-left)
    world.walls.push_back(Wall({midX + 40, shelfY2 + 50}, {right, shelfY2}));
}

// ── Place balls in a grid above the container ───────────────────────
// Balls start tightly packed above the container so they rain down.
static void setupBalls(PhysicsWorld& world) {
    float left  = WALL_MARGIN + 10.0f;
    float right = WINDOW_WIDTH - WALL_MARGIN - 10.0f;
    float top   = WALL_MARGIN + 10.0f;

    // Calculate grid dimensions to fit NUM_BALLS
    float spacing = (BALL_RADIUS_MAX * 2.0f) + 1.0f;
    int cols = static_cast<int>((right - left) / spacing);
    if (cols < 1) cols = 1;

    for (int i = 0; i < NUM_BALLS; ++i) {
        int col = i % cols;
        int row = i / cols;

        float x = left + col * spacing + randFloat(-0.5f, 0.5f);
        float y = top + row * spacing + randFloat(-0.5f, 0.5f);
        float r = randFloat(BALL_RADIUS_MIN, BALL_RADIUS_MAX);

        Ball ball(Vec2(x, y), r);
        // Give a small random initial velocity for visual variety
        ball.vel = Vec2(randFloat(-30.0f, 30.0f), randFloat(-10.0f, 10.0f));

        world.balls.push_back(ball);
    }
}

// ═══════════════════════════════════════════════════════════════════════
// Headless mode — run for a fixed number of frames with the offscreen
// video driver, saving BMP screenshots at key moments.
// ═══════════════════════════════════════════════════════════════════════
static int runHeadless(float restitution, int totalFrames, const char* prefix,
                       const char* loadCSV, const char* saveCSV) {
    printf("Headless mode: %d frames, restitution=%.2f, prefix='%s'\n",
           totalFrames, restitution, prefix);
    if (loadCSV) printf("  Loading scene from: %s\n", loadCSV);
    if (saveCSV) printf("  Will save final scene to: %s\n", saveCSV);

    // Force offscreen driver so we get a real rendering surface
    // without needing a display server.
    SDL_SetHint(SDL_HINT_VIDEO_DRIVER, "offscreen");

    PhysicsWorld world;
    world.config.restitution = restitution;
    world.config.gravity     = 500.0f;
    world.config.substeps    = 8;
    world.config.solverIterations = 8;
    world.config.damping     = 0.998f;
    world.config.friction    = 0.1f;
    world.config.sleepSpeed  = 5.0f;
    world.config.bounceThreshold = 30.0f;

    if (loadCSV) {
        if (!loadSceneFromCSV(loadCSV, world)) {
            fprintf(stderr, "Failed to load CSV scene from '%s'\n", loadCSV);
            return 1;
        }
    } else {
        setupWalls(world);
        setupBalls(world);
    }

    Renderer renderer;
    if (!renderer.init()) {
        fprintf(stderr, "Headless: failed to initialize renderer\n");
        return 1;
    }

    // Fixed timestep for deterministic headless simulation.
    constexpr float dt = 1.0f / 60.0f;

    // Screenshot schedule: frame 0 (initial), 25% (bouncing), 50% (settling),
    // and the final frame (fully settled).
    int screenshotFrames[] = {
        0,
        totalFrames / 4,
        totalFrames / 2,
        totalFrames - 1
    };
    const char* labels[] = {"initial", "bouncing", "settling", "settled"};

    int nextShot = 0;

    for (int frame = 0; frame < totalFrames; ++frame) {
        world.step(dt);

        // Render every frame so the final screenshot reflects the true state.
        float ke = world.totalKineticEnergy();
        renderer.clear();
        renderer.draw(world);
        renderer.drawHUD(60.0f, static_cast<int>(world.balls.size()), ke);
        renderer.present();

        // Save screenshot at scheduled moments.
        if (nextShot < 4 && frame == screenshotFrames[nextShot]) {
            char filename[256];
            snprintf(filename, sizeof(filename), "%s_%s.bmp", prefix, labels[nextShot]);
            if (renderer.saveScreenshot(filename)) {
                printf("  Saved %s (frame %d)\n", filename, frame);
            }
            nextShot++;
        }

        // Progress reporting every 10% of frames
        if (frame > 0 && frame % (totalFrames / 10) == 0) {
            float ke = world.totalKineticEnergy();
            printf("  Frame %d/%d — KE=%.1f\n", frame, totalFrames, ke);
        }
    }

    // Save final positions to CSV if requested
    if (saveCSV) {
        if (!saveSceneToCSV(saveCSV, world)) {
            fprintf(stderr, "Failed to save CSV scene to '%s'\n", saveCSV);
            return 1;
        }
    }

    printf("Headless run complete.\n");
    return 0;
}

// ═══════════════════════════════════════════════════════════════════════
// main
// ═══════════════════════════════════════════════════════════════════════
int main(int argc, char* argv[]) {
    srand(static_cast<unsigned>(time(nullptr)));

    // ── Parse CLI arguments ─────────────────────────────────────────
    // Flags: --headless, --load-csv <file>, --save-csv <file>
    // Positional (after flags): [restitution] [frames] [screenshot_prefix]
    bool headless = false;
    const char* loadCSV = nullptr;
    const char* saveCSV = nullptr;

    // Collect positional arguments (non-flag arguments)
    std::vector<const char*> positional;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--headless") == 0) {
            headless = true;
        } else if (strcmp(argv[i], "--load-csv") == 0 && i + 1 < argc) {
            loadCSV = argv[++i];
        } else if (strcmp(argv[i], "--save-csv") == 0 && i + 1 < argc) {
            saveCSV = argv[++i];
        } else {
            positional.push_back(argv[i]);
        }
    }

    // Parse positional args: [restitution] [frames] [screenshot_prefix]
    float restitution = 0.3f;
    if (positional.size() > 0) {
        restitution = static_cast<float>(atof(positional[0]));
        if (restitution < 0.0f) restitution = 0.0f;
        if (restitution > 1.0f) restitution = 1.0f;
    }

    if (headless) {
        int frames = 600;
        const char* prefix = "screenshot";
        if (positional.size() > 1) frames = atoi(positional[1]);
        if (positional.size() > 2) prefix = positional[2];
        if (frames < 1) frames = 600;
        return runHeadless(restitution, frames, prefix, loadCSV, saveCSV);
    }

    printf("Physics Simulator — restitution = %.2f\n", restitution);
    printf("Controls:\n");
    printf("  SPACE       — Pause / Resume\n");
    printf("  RIGHT / N   — Single step (when paused)\n");
    printf("  UP / DOWN   — Speed up / slow down (0.25x–4x)\n");
    printf("  1           — Reset speed to 1x\n");
    printf("  R           — Restart simulation\n");
    printf("  ESC / Q     — Quit\n");

    // Set up physics world
    PhysicsWorld world;
    world.config.restitution = restitution;
    world.config.gravity     = 500.0f;
    world.config.substeps    = 8;
    world.config.solverIterations = 8;
    world.config.damping     = 0.998f;
    world.config.friction    = 0.1f;
    world.config.sleepSpeed  = 5.0f;
    world.config.bounceThreshold = 30.0f;

    if (loadCSV) {
        if (!loadSceneFromCSV(loadCSV, world)) {
            fprintf(stderr, "Failed to load CSV scene from '%s'\n", loadCSV);
            return 1;
        }
    } else {
        setupWalls(world);
        setupBalls(world);
    }

    // Initialize renderer
    Renderer renderer;
    if (!renderer.init()) {
        fprintf(stderr, "Failed to initialize renderer\n");
        return 1;
    }

    // ── Helper lambda to reset the simulation ───────────────────────
    // Clears and rebuilds the scene so R-key restart works.
    auto resetSimulation = [&]() {
        world.balls.clear();
        world.walls.clear();
        if (loadCSV) {
            loadSceneFromCSV(loadCSV, world);
        } else {
            setupWalls(world);
            setupBalls(world);
        }
    };

    // ── Main loop ───────────────────────────────────────────────────
    Uint64 lastTime = SDL_GetPerformanceCounter();
    Uint64 freq = SDL_GetPerformanceFrequency();

    // FPS tracking: exponential moving average for smooth display.
    float fpsSmoothed = 60.0f;

    bool running = true;
    while (running) {
        // Calculate delta time
        Uint64 now = SDL_GetPerformanceCounter();
        float dt = static_cast<float>(now - lastTime) / static_cast<float>(freq);
        lastTime = now;

        // Update smoothed FPS (blend factor 0.05 → responds in ~20 frames)
        if (dt > 0.0f) {
            float instantFps = 1.0f / dt;
            fpsSmoothed = fpsSmoothed * 0.95f + instantFps * 0.05f;
        }

        // Handle input (updates pause, speed, step state)
        running = renderer.pollEvents();

        // Handle restart request
        if (renderer.consumeRestart()) {
            resetSimulation();
        }

        // Step physics: respect pause and speed controls
        if (!renderer.isPaused() || renderer.consumeStep()) {
            float adjustedDt = dt * renderer.speedMultiplier();
            world.step(adjustedDt);
        }

        // Render
        float ke = world.totalKineticEnergy();
        renderer.clear();
        renderer.draw(world);
        renderer.drawHUD(fpsSmoothed, static_cast<int>(world.balls.size()), ke);
        renderer.present();
    }

    // Save final positions to CSV if requested
    if (saveCSV) {
        if (!saveSceneToCSV(saveCSV, world)) {
            fprintf(stderr, "Failed to save CSV scene to '%s'\n", saveCSV);
            return 1;
        }
    }

    return 0;
}
