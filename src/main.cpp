// main.cpp — Entry point for the 2D physics simulator.
//
// Sets up a scene with ~1000 balls inside a walled container,
// then runs the simulation loop. Restitution can be configured
// via command-line argument (default 0.3).
//
// Usage: ./simulator [restitution]
//   restitution: float in [0, 1], default 0.3
//                0.0 = perfectly inelastic (balls stop fast)
//                1.0 = perfectly elastic (balls bounce forever)

#include "physics.h"
#include "renderer.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <ctime>

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
// main
// ═══════════════════════════════════════════════════════════════════════
int main(int argc, char* argv[]) {
    srand(static_cast<unsigned>(time(nullptr)));

    // Parse optional restitution from command line
    float restitution = 0.3f;
    if (argc >= 2) {
        restitution = static_cast<float>(atof(argv[1]));
        if (restitution < 0.0f) restitution = 0.0f;
        if (restitution > 1.0f) restitution = 1.0f;
    }

    printf("Physics Simulator — restitution = %.2f\n", restitution);
    printf("Controls: ESC or Q to quit\n");

    // Set up physics world
    PhysicsWorld world;
    world.config.restitution = restitution;
    world.config.gravity     = 500.0f;
    world.config.substeps    = 8;
    world.config.damping     = 0.999f;
    world.config.friction    = 0.1f;
    world.config.sleepSpeed  = 2.0f;

    setupWalls(world);
    setupBalls(world);

    // Initialize renderer
    Renderer renderer;
    if (!renderer.init()) {
        fprintf(stderr, "Failed to initialize renderer\n");
        return 1;
    }

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

        // Handle input
        running = renderer.pollEvents();

        // Step physics
        world.step(dt);

        // Render
        renderer.clear();
        renderer.draw(world);
        renderer.drawHUD(fpsSmoothed, static_cast<int>(world.balls.size()));
        renderer.present();
    }

    return 0;
}
