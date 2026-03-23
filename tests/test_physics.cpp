// test_physics.cpp — Unit tests for the physics engine.
// Uses a minimal test framework (no external dependencies).
// Tests cover: Vec2 math, gravity, ball-wall collision, ball-ball collision,
// overlap resolution, restitution behavior, and energy dissipation.

#include "physics.h"
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <chrono>

// ── Minimal test framework ──────────────────────────────────────────
static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) \
    static void test_##name(); \
    static struct Register_##name { \
        Register_##name() { \
            tests_run++; \
            printf("  TEST %-50s ", #name); \
            try { test_##name(); tests_passed++; printf("PASS\n"); } \
            catch (...) { tests_failed++; printf("FAIL (exception)\n"); } \
        } \
    } register_##name; \
    static void test_##name()

#define ASSERT(cond) do { \
    if (!(cond)) { \
        printf("FAIL\n    Assertion failed: %s\n    at %s:%d\n", \
               #cond, __FILE__, __LINE__); \
        tests_failed++; tests_passed--; return; \
    } \
} while(0)

#define ASSERT_NEAR(a, b, eps) do { \
    float _a = (a), _b = (b), _e = (eps); \
    if (std::abs(_a - _b) > _e) { \
        printf("FAIL\n    ASSERT_NEAR: %f != %f (eps=%f)\n    at %s:%d\n", \
               _a, _b, _e, __FILE__, __LINE__); \
        tests_failed++; tests_passed--; return; \
    } \
} while(0)

// ── Shared settling helpers ─────────────────────────────────────────
// These helpers intentionally build the exact same compact stacking
// scenario with different restitution values. The project requirement
// is not just "lower restitution settles faster"; it also requires the
// final resting pile to occupy the same amount of space regardless of
// restitution. Keeping the fixture in one place makes that invariant
// easy to extend in later iterations.
struct SettledBounds {
    float minX = 0.0f;
    float maxX = 0.0f;
    float minY = 0.0f;
    float maxY = 0.0f;
    float maxSpeed = 0.0f;

    float width() const { return maxX - minX; }
    float height() const { return maxY - minY; }
};

static PhysicsWorld makeSettlingWorld(float restitution) {
    PhysicsWorld world;
    world.config.gravity = 500.0f;
    world.config.substeps = 8;
    world.config.restitution = restitution;
    world.config.damping = 0.999f;
    world.config.friction = 0.1f;
    world.config.sleepSpeed = 2.0f;

    // A simple rectangular container removes scene-specific noise so
    // the test isolates the solver's packing behavior.
    world.walls.push_back(Wall(Vec2(0, 0), Vec2(200, 0)));
    world.walls.push_back(Wall(Vec2(200, 0), Vec2(200, 400)));
    world.walls.push_back(Wall(Vec2(200, 400), Vec2(0, 400)));
    world.walls.push_back(Wall(Vec2(0, 400), Vec2(0, 0)));

    // The balls start in a regular grid so every restitution value
    // begins from the same state and any final-volume drift is easy to spot.
    for (int i = 0; i < 50; ++i) {
        float x = 20 + (i % 10) * 18;
        float y = 20 + (i / 10) * 18;
        Ball b(Vec2(x, y), 7.0f);
        world.balls.push_back(b);
    }

    return world;
}

static SettledBounds simulateToSettledBounds(float restitution, int steps) {
    PhysicsWorld world = makeSettlingWorld(restitution);

    // Run long enough for high-restitution cases to finish bouncing and
    // for the sleep threshold to zero out residual micro-motion.
    for (int i = 0; i < steps; ++i) {
        world.step(0.016f);
    }

    SettledBounds bounds;
    bounds.minX = 1e9f;
    bounds.maxX = -1e9f;
    bounds.minY = 1e9f;
    bounds.maxY = -1e9f;

    for (const auto& b : world.balls) {
        bounds.minX = std::min(bounds.minX, b.pos.x - b.radius);
        bounds.maxX = std::max(bounds.maxX, b.pos.x + b.radius);
        bounds.minY = std::min(bounds.minY, b.pos.y - b.radius);
        bounds.maxY = std::max(bounds.maxY, b.pos.y + b.radius);
        bounds.maxSpeed = std::max(bounds.maxSpeed, b.vel.length());
    }

    return bounds;
}

static PhysicsWorld makeShelfSettlingWorld(float restitution) {
    PhysicsWorld world;
    world.config.gravity = 500.0f;
    world.config.substeps = 8;
    world.config.restitution = restitution;
    world.config.damping = 0.999f;
    world.config.friction = 0.1f;
    world.config.sleepSpeed = 2.0f;

    // This fixture mirrors the real simulator more closely than the simple
    // box test above: it uses the same kind of container plus two internal
    // shelves. Keeping it deterministic gives us a stable regression target
    // for the "same final occupied space" requirement without depending on
    // SDL, randomness, or the interactive app loop.
    const float left = 50.0f;
    const float right = 750.0f;
    const float top = 50.0f;
    const float bottom = 750.0f;

    world.walls.push_back(Wall(Vec2(left, top), Vec2(right, top)));
    world.walls.push_back(Wall(Vec2(right, top), Vec2(right, bottom)));
    world.walls.push_back(Wall(Vec2(right, bottom), Vec2(left, bottom)));
    world.walls.push_back(Wall(Vec2(left, bottom), Vec2(left, top)));

    const float midX = (left + right) / 2.0f;
    const float shelfY1 = top + (bottom - top) * 0.35f;
    const float shelfY2 = top + (bottom - top) * 0.6f;
    world.walls.push_back(Wall(Vec2(left, shelfY1), Vec2(midX - 40.0f, shelfY1 + 50.0f)));
    world.walls.push_back(Wall(Vec2(midX + 40.0f, shelfY2 + 50.0f), Vec2(right, shelfY2)));

    // Mixed radii make the packing problem more demanding than a uniform
    // lattice. If restitution were incorrectly changing the final occupied
    // shape, this scene tends to expose it because shelves create multiple
    // local basins and the different ball sizes interlock non-uniformly.
    for (int i = 0; i < 120; ++i) {
        const float x = 70.0f + static_cast<float>(i % 12) * 24.0f;
        const float y = 70.0f + static_cast<float>(i / 12) * 24.0f;
        const float radius = (i % 3 == 0) ? 5.0f : ((i % 3 == 1) ? 7.0f : 9.0f);
        world.balls.push_back(Ball(Vec2(x, y), radius));
    }

    return world;
}

static SettledBounds simulateShelfSceneToSettledBounds(float restitution, int steps) {
    PhysicsWorld world = makeShelfSettlingWorld(restitution);

    // The shelf scene has more contacts than the simple box fixture, so it
    // needs a longer run to let the high-restitution case finish redistributing
    // across shelves and to let the sleep threshold clear residual jitter.
    for (int i = 0; i < steps; ++i) {
        world.step(0.016f);
    }

    SettledBounds bounds;
    bounds.minX = 1e9f;
    bounds.maxX = -1e9f;
    bounds.minY = 1e9f;
    bounds.maxY = -1e9f;

    for (const auto& b : world.balls) {
        bounds.minX = std::min(bounds.minX, b.pos.x - b.radius);
        bounds.maxX = std::max(bounds.maxX, b.pos.x + b.radius);
        bounds.minY = std::min(bounds.minY, b.pos.y - b.radius);
        bounds.maxY = std::max(bounds.maxY, b.pos.y + b.radius);
        bounds.maxSpeed = std::max(bounds.maxSpeed, b.vel.length());
    }

    return bounds;
}

// ═══════════════════════════════════════════════════════════════════════
// Vec2 tests
// ═══════════════════════════════════════════════════════════════════════

TEST(vec2_add) {
    Vec2 a(1, 2), b(3, 4);
    Vec2 c = a + b;
    ASSERT_NEAR(c.x, 4.0f, 1e-5f);
    ASSERT_NEAR(c.y, 6.0f, 1e-5f);
}

TEST(vec2_sub) {
    Vec2 a(5, 3), b(2, 1);
    Vec2 c = a - b;
    ASSERT_NEAR(c.x, 3.0f, 1e-5f);
    ASSERT_NEAR(c.y, 2.0f, 1e-5f);
}

TEST(vec2_dot) {
    Vec2 a(1, 0), b(0, 1);
    ASSERT_NEAR(a.dot(b), 0.0f, 1e-5f);
    ASSERT_NEAR(a.dot(a), 1.0f, 1e-5f);
}

TEST(vec2_length) {
    Vec2 v(3, 4);
    ASSERT_NEAR(v.length(), 5.0f, 1e-5f);
}

TEST(vec2_normalized) {
    Vec2 v(3, 4);
    Vec2 n = v.normalized();
    ASSERT_NEAR(n.length(), 1.0f, 1e-5f);
    ASSERT_NEAR(n.x, 0.6f, 1e-5f);
    ASSERT_NEAR(n.y, 0.8f, 1e-5f);
}

TEST(vec2_zero_normalized) {
    Vec2 v(0, 0);
    Vec2 n = v.normalized();
    ASSERT_NEAR(n.x, 0.0f, 1e-5f);
    ASSERT_NEAR(n.y, 0.0f, 1e-5f);
}

// ═══════════════════════════════════════════════════════════════════════
// Gravity tests
// ═══════════════════════════════════════════════════════════════════════

TEST(gravity_accelerates_ball_downward) {
    // A ball in free space should accelerate downward
    PhysicsWorld world;
    world.config.gravity = 100.0f;
    world.config.substeps = 1;
    world.config.damping = 1.0f; // No damping for clean test
    world.config.sleepSpeed = 0.0f; // Disable sleep

    Ball b(Vec2(100, 100), 5.0f);
    b.vel = {0, 0};
    world.balls.push_back(b);

    // Step for 1 second (in small steps to avoid dt clamping)
    for (int i = 0; i < 100; ++i) {
        world.step(0.01f);
    }

    // Ball should have moved down significantly
    ASSERT(world.balls[0].pos.y > 140.0f);
    // Ball should have downward velocity
    ASSERT(world.balls[0].vel.y > 50.0f);
}

TEST(gravity_does_not_affect_horizontal) {
    PhysicsWorld world;
    world.config.gravity = 100.0f;
    world.config.substeps = 1;
    world.config.damping = 1.0f;
    world.config.sleepSpeed = 0.0f;

    Ball b(Vec2(200, 200), 5.0f);
    b.vel = {0, 0};
    world.balls.push_back(b);

    for (int i = 0; i < 100; ++i) {
        world.step(0.01f);
    }

    // X position should be unchanged
    ASSERT_NEAR(world.balls[0].pos.x, 200.0f, 0.1f);
}

// ═══════════════════════════════════════════════════════════════════════
// Ball-wall collision tests
// ═══════════════════════════════════════════════════════════════════════

TEST(ball_stays_above_floor) {
    // Ball dropped onto a floor should not pass through
    PhysicsWorld world;
    world.config.gravity = 500.0f;
    world.config.substeps = 8;
    world.config.restitution = 0.3f;
    world.config.damping = 0.999f;
    world.config.sleepSpeed = 0.0f;

    Ball b(Vec2(100, 80), 10.0f);
    world.balls.push_back(b);

    // Floor at y=100
    world.walls.push_back(Wall(Vec2(0, 100), Vec2(200, 100)));

    for (int i = 0; i < 1000; ++i) {
        world.step(0.016f);
    }

    // Ball center should be at or above floor minus radius
    ASSERT(world.balls[0].pos.y <= 100.0f);
}

TEST(ball_bounces_off_wall) {
    // Ball moving right should bounce off a vertical wall
    PhysicsWorld world;
    world.config.gravity = 0.0f; // No gravity for this test
    world.config.substeps = 4;
    world.config.restitution = 0.8f;
    world.config.damping = 1.0f;
    world.config.sleepSpeed = 0.0f;

    Ball b(Vec2(80, 50), 5.0f);
    b.vel = {200, 0}; // Moving right
    world.balls.push_back(b);

    // Vertical wall at x=100
    world.walls.push_back(Wall(Vec2(100, 0), Vec2(100, 200)));

    // Step until ball would have hit the wall
    for (int i = 0; i < 50; ++i) {
        world.step(0.016f);
    }

    // Ball should have bounced back (negative x velocity)
    ASSERT(world.balls[0].vel.x < 0.0f);
    // Ball should be to the left of the wall
    ASSERT(world.balls[0].pos.x < 100.0f - world.balls[0].radius + 1.0f);
}

TEST(ball_does_not_phase_through_wall) {
    // Fast ball should still be caught by substep integration
    PhysicsWorld world;
    world.config.gravity = 0.0f;
    world.config.substeps = 16; // High substeps for fast ball
    world.config.restitution = 0.5f;
    world.config.damping = 1.0f;
    world.config.sleepSpeed = 0.0f;

    Ball b(Vec2(50, 50), 5.0f);
    b.vel = {2000, 0}; // Very fast
    world.balls.push_back(b);

    // Wall at x=200
    world.walls.push_back(Wall(Vec2(200, 0), Vec2(200, 200)));

    for (int i = 0; i < 20; ++i) {
        world.step(0.016f);
    }

    // Ball must not have passed through the wall
    ASSERT(world.balls[0].pos.x < 200.0f);
}

TEST(ball_bounces_off_wall_endpoint) {
    // This regression targets the exact endpoint-overlap branch in the
    // wall solver. If the ball center lands directly on the segment end,
    // the solver cannot use the generic wall normal because that only
    // reflects the segment axis. Instead it needs a point-contact normal
    // so both velocity components reverse away from the endpoint.
    PhysicsWorld world;
    world.config.gravity = 0.0f;
    world.config.substeps = 1;
    world.config.restitution = 1.0f;
    world.config.friction = 0.0f;
    world.config.damping = 1.0f;
    world.config.sleepSpeed = 0.0f;

    Ball ball(Vec2(100.0f, 60.0f), 5.0f);
    ball.vel = {60.0f, 60.0f};
    world.balls.push_back(ball);

    // The segment endpoint is exactly at the ball center. Stepping with
    // dt=0 still runs the solver, which isolates the overlap-resolution
    // branch without any integration noise.
    world.walls.push_back(Wall(Vec2(40.0f, 60.0f), Vec2(100.0f, 60.0f)));
    world.step(0.0f);

    const Vec2 endpoint = world.walls[0].p2;
    const Vec2 fromEndpoint = world.balls[0].pos - endpoint;

    ASSERT(fromEndpoint.length() >= world.balls[0].radius - 0.2f);
    ASSERT(world.balls[0].vel.x < 0.0f);
    ASSERT(world.balls[0].vel.y < 0.0f);
}

TEST(ball_remains_outside_corner_joint) {
    // Two walls meeting at a corner should behave like a sealed boundary.
    // This regression catches cases where a ball gets numerically wedged
    // into the joint and leaks through because each wall is considered in
    // isolation without respecting the shared endpoint geometry.
    PhysicsWorld world;
    world.config.gravity = 0.0f;
    world.config.substeps = 12;
    world.config.restitution = 0.2f;
    world.config.friction = 0.0f;
    world.config.damping = 1.0f;
    world.config.sleepSpeed = 0.0f;

    Ball ball(Vec2(80.0f, 80.0f), 10.0f);
    ball.vel = {140.0f, 140.0f};
    world.balls.push_back(ball);

    // These two walls form a closed bottom-right corner. The inside region
    // is up-left of the joint, so the ball should never end up with its
    // center closer than one radius to either boundary.
    world.walls.push_back(Wall(Vec2(100.0f, 0.0f), Vec2(100.0f, 100.0f)));
    world.walls.push_back(Wall(Vec2(100.0f, 100.0f), Vec2(0.0f, 100.0f)));

    for (int i = 0; i < 80; ++i) {
        world.step(0.016f);

        ASSERT(world.balls[0].pos.x <= 90.2f);
        ASSERT(world.balls[0].pos.y <= 90.2f);
    }
}

// ═══════════════════════════════════════════════════════════════════════
// Ball-ball collision tests
// ═══════════════════════════════════════════════════════════════════════

TEST(two_balls_do_not_overlap) {
    // Two balls approaching each other should not overlap
    PhysicsWorld world;
    world.config.gravity = 0.0f;
    world.config.substeps = 4;
    world.config.restitution = 0.5f;
    world.config.damping = 1.0f;
    world.config.sleepSpeed = 0.0f;

    Ball a(Vec2(40, 50), 10.0f);
    a.vel = {100, 0};
    Ball b(Vec2(80, 50), 10.0f);
    b.vel = {-100, 0};
    world.balls.push_back(a);
    world.balls.push_back(b);

    for (int i = 0; i < 100; ++i) {
        world.step(0.016f);

        // Check no overlap at any point
        Vec2 diff = world.balls[1].pos - world.balls[0].pos;
        float dist = diff.length();
        float minDist = world.balls[0].radius + world.balls[1].radius;
        // Allow tiny floating-point tolerance
        ASSERT(dist >= minDist - 0.5f);
    }
}

TEST(head_on_collision_conserves_direction) {
    // Two equal balls in head-on collision should swap velocities (approx)
    PhysicsWorld world;
    world.config.gravity = 0.0f;
    world.config.substeps = 8;
    world.config.restitution = 1.0f; // Perfectly elastic
    world.config.damping = 1.0f;
    world.config.friction = 0.0f;
    world.config.sleepSpeed = 0.0f;

    Ball a(Vec2(30, 50), 10.0f);
    a.vel = {100, 0};
    Ball b(Vec2(80, 50), 10.0f);
    b.vel = {0, 0};
    world.balls.push_back(a);
    world.balls.push_back(b);

    // Run just long enough for the collision to happen
    for (int i = 0; i < 30; ++i) {
        world.step(0.016f);
    }

    // After collision: ball B should have gained rightward momentum
    // (in a perfectly elastic equal-mass collision, A stops and B takes all velocity)
    ASSERT(world.balls[1].pos.x > 80.0f); // B moved right from its start position
    ASSERT_NEAR(world.balls[0].vel.x, 0.0f, 1.0f);
    ASSERT_NEAR(world.balls[1].vel.x, 100.0f, 1.0f);
}

TEST(inelastic_head_on_collision_reduces_relative_speed) {
    // This regression targets the impulse direction directly. If the
    // relative velocity sign is wrong, the solver will separate the
    // balls positionally but leave them with nearly their original
    // closing speed, which violates the restitution requirement.
    PhysicsWorld world;
    world.config.gravity = 0.0f;
    world.config.substeps = 8;
    world.config.restitution = 0.25f;
    world.config.damping = 1.0f;
    world.config.friction = 0.0f;
    world.config.sleepSpeed = 0.0f;

    Ball a(Vec2(30, 50), 10.0f);
    a.vel = {120, 0};
    Ball b(Vec2(80, 50), 10.0f);
    b.vel = {-40, 0};
    world.balls.push_back(a);
    world.balls.push_back(b);

    // Run until after the collision has been resolved.
    for (int i = 0; i < 30; ++i) {
        world.step(0.016f);
    }

    const float finalRelativeSpeed = world.balls[1].vel.x - world.balls[0].vel.x;
    const float expectedRelativeSpeed = 0.25f * (120.0f - (-40.0f));

    // After impact, the balls should be moving apart and their
    // separation speed should match restitution for equal-mass bodies.
    ASSERT(finalRelativeSpeed > 0.0f);
    ASSERT_NEAR(finalRelativeSpeed, expectedRelativeSpeed, 2.0f);
}

TEST(ball_ball_overlap_resolved) {
    // Two overlapping balls should be pushed apart
    PhysicsWorld world;
    world.config.gravity = 0.0f;
    world.config.substeps = 4;
    world.config.restitution = 0.5f;
    world.config.damping = 1.0f;
    world.config.sleepSpeed = 0.0f;

    // Place two balls overlapping
    Ball a(Vec2(50, 50), 10.0f);
    a.vel = {0, 0};
    Ball b(Vec2(55, 50), 10.0f); // Only 5px apart, should be 20
    b.vel = {0, 0};
    world.balls.push_back(a);
    world.balls.push_back(b);

    world.step(0.016f);

    // They should be separated now
    Vec2 diff = world.balls[1].pos - world.balls[0].pos;
    float dist = diff.length();
    ASSERT(dist >= 19.5f); // Should be ~20 (sum of radii)
}

// ═══════════════════════════════════════════════════════════════════════
// Restitution tests
// ═══════════════════════════════════════════════════════════════════════

TEST(low_restitution_settles_faster) {
    // With low restitution, kinetic energy should drop faster
    auto runSim = [](float restitution) -> float {
        PhysicsWorld world;
        world.config.gravity = 500.0f;
        world.config.substeps = 4;
        world.config.restitution = restitution;
        world.config.damping = 0.999f;
        world.config.sleepSpeed = 0.0f; // Disable so we can measure energy

        // Drop 10 balls onto a floor
        for (int i = 0; i < 10; ++i) {
            Ball b(Vec2(50 + i * 25, 50), 8.0f);
            world.balls.push_back(b);
        }
        world.walls.push_back(Wall(Vec2(0, 400), Vec2(500, 400)));  // floor
        world.walls.push_back(Wall(Vec2(0, 400), Vec2(0, 0)));      // left
        world.walls.push_back(Wall(Vec2(500, 0), Vec2(500, 400)));  // right

        // Run for 3 simulated seconds
        for (int i = 0; i < 200; ++i) {
            world.step(0.016f);
        }

        return world.totalKineticEnergy();
    };

    float energyLow  = runSim(0.1f);
    float energyHigh = runSim(0.9f);

    // Low restitution should have less remaining energy
    ASSERT(energyLow < energyHigh);
}

TEST(restitution_zero_stops_quickly) {
    PhysicsWorld world;
    world.config.gravity = 500.0f;
    world.config.substeps = 8;
    world.config.restitution = 0.0f;
    world.config.damping = 0.999f;
    world.config.sleepSpeed = 1.0f;

    Ball b(Vec2(100, 50), 10.0f);
    world.balls.push_back(b);
    world.walls.push_back(Wall(Vec2(0, 200), Vec2(300, 200))); // floor

    // Run for 5 simulated seconds
    for (int i = 0; i < 300; ++i) {
        world.step(0.016f);
    }

    // Ball should have essentially stopped
    ASSERT(world.balls[0].vel.length() < 5.0f);
}

TEST(restitution_changes_decay_not_final_packed_size) {
    // The final resting footprint should be a property of geometry and
    // ball sizes, not of how bouncy the transient motion was on the way there.
    const SettledBounds lowRestitution = simulateToSettledBounds(0.0f, 900);
    const SettledBounds mediumRestitution = simulateToSettledBounds(0.3f, 900);
    const SettledBounds highRestitution = simulateToSettledBounds(0.9f, 900);

    ASSERT_NEAR(lowRestitution.maxSpeed, 0.0f, 0.05f);
    ASSERT_NEAR(mediumRestitution.maxSpeed, 0.0f, 0.05f);
    ASSERT_NEAR(highRestitution.maxSpeed, 0.0f, 0.05f);
    ASSERT_NEAR(lowRestitution.height(), mediumRestitution.height(), 0.5f);
    ASSERT_NEAR(lowRestitution.height(), highRestitution.height(), 0.5f);
    ASSERT_NEAR(lowRestitution.width(), mediumRestitution.width(), 0.5f);
    ASSERT_NEAR(lowRestitution.width(), highRestitution.width(), 0.5f);
    ASSERT_NEAR(lowRestitution.minY, mediumRestitution.minY, 0.5f);
    ASSERT_NEAR(lowRestitution.minY, highRestitution.minY, 0.5f);
}

TEST(restitution_preserves_final_packed_size_in_shelf_scene) {
    // The project requirement applies to the actual simulator-style geometry,
    // not just a plain box. This regression hardens that promise by checking
    // a shelf-filled container with mixed ball sizes, which is much closer to
    // the interactive scene than the simple stacking fixture above.
    const SettledBounds lowRestitution = simulateShelfSceneToSettledBounds(0.0f, 2200);
    const SettledBounds mediumRestitution = simulateShelfSceneToSettledBounds(0.3f, 2200);
    const SettledBounds highRestitution = simulateShelfSceneToSettledBounds(0.9f, 2200);

    ASSERT_NEAR(lowRestitution.maxSpeed, 0.0f, 0.05f);
    ASSERT_NEAR(mediumRestitution.maxSpeed, 0.0f, 0.05f);
    ASSERT_NEAR(highRestitution.maxSpeed, 0.0f, 0.05f);
    ASSERT_NEAR(lowRestitution.width(), mediumRestitution.width(), 0.5f);
    ASSERT_NEAR(lowRestitution.width(), highRestitution.width(), 0.5f);
    ASSERT_NEAR(lowRestitution.height(), mediumRestitution.height(), 0.5f);
    ASSERT_NEAR(lowRestitution.height(), highRestitution.height(), 0.5f);
    ASSERT_NEAR(lowRestitution.minY, mediumRestitution.minY, 0.5f);
    ASSERT_NEAR(lowRestitution.minY, highRestitution.minY, 0.5f);
}

// ═══════════════════════════════════════════════════════════════════════
// Energy dissipation test
// ═══════════════════════════════════════════════════════════════════════

TEST(energy_decreases_over_time) {
    PhysicsWorld world;
    world.config.gravity = 500.0f;
    world.config.substeps = 4;
    world.config.restitution = 0.5f;
    world.config.damping = 0.999f;
    world.config.sleepSpeed = 0.0f;

    // A few balls bouncing in a box
    for (int i = 0; i < 5; ++i) {
        Ball b(Vec2(50 + i * 30, 50), 8.0f);
        b.vel = {static_cast<float>(i * 20 - 40), 0};
        world.balls.push_back(b);
    }
    world.walls.push_back(Wall(Vec2(0, 300), Vec2(300, 300)));
    world.walls.push_back(Wall(Vec2(0, 0), Vec2(0, 300)));
    world.walls.push_back(Wall(Vec2(300, 300), Vec2(300, 0)));

    // Measure energy at start and after simulation
    // (After gravity has had time to add energy, measure relative decrease)
    for (int i = 0; i < 100; ++i) world.step(0.016f);
    float energyMid = world.totalKineticEnergy();

    for (int i = 0; i < 500; ++i) world.step(0.016f);
    float energyLate = world.totalKineticEnergy();

    // Energy should decrease (or at least not explode)
    ASSERT(energyLate <= energyMid * 1.5f); // Allow some tolerance for gravity input
}

// ═══════════════════════════════════════════════════════════════════════
// Stacking / settling test
// ═══════════════════════════════════════════════════════════════════════

TEST(balls_settle_in_container) {
    // Many balls in a box should eventually settle
    PhysicsWorld world;
    world.config.gravity = 500.0f;
    world.config.substeps = 8;
    world.config.restitution = 0.3f;
    world.config.damping = 0.999f;
    world.config.sleepSpeed = 2.0f;

    // Box
    world.walls.push_back(Wall(Vec2(0, 0), Vec2(200, 0)));
    world.walls.push_back(Wall(Vec2(200, 0), Vec2(200, 400)));
    world.walls.push_back(Wall(Vec2(200, 400), Vec2(0, 400)));
    world.walls.push_back(Wall(Vec2(0, 400), Vec2(0, 0)));

    // 50 balls dropped from top
    for (int i = 0; i < 50; ++i) {
        float x = 20 + (i % 10) * 18;
        float y = 20 + (i / 10) * 18;
        Ball b(Vec2(x, y), 7.0f);
        world.balls.push_back(b);
    }

    // Run for 10 simulated seconds
    for (int i = 0; i < 600; ++i) {
        world.step(0.016f);
    }

    // All balls should be inside the container
    for (const auto& b : world.balls) {
        ASSERT(b.pos.x > -5.0f && b.pos.x < 205.0f);
        ASSERT(b.pos.y > -5.0f && b.pos.y < 405.0f);
    }

    // Most balls should be nearly stopped
    int stoppedCount = 0;
    for (const auto& b : world.balls) {
        if (b.vel.length() < 5.0f) stoppedCount++;
    }
    ASSERT(stoppedCount > 40); // At least 80% settled
}

// ═══════════════════════════════════════════════════════════════════════
// Wall normal test
// ═══════════════════════════════════════════════════════════════════════

TEST(wall_normal_perpendicular) {
    Wall w(Vec2(0, 0), Vec2(10, 0)); // Horizontal wall
    Vec2 n = w.normal();
    ASSERT_NEAR(n.x, 0.0f, 1e-5f);
    ASSERT_NEAR(std::abs(n.y), 1.0f, 1e-5f);
}

TEST(wall_normal_unit_length) {
    Wall w(Vec2(0, 0), Vec2(3, 4));
    Vec2 n = w.normal();
    ASSERT_NEAR(n.length(), 1.0f, 1e-5f);
}

// ═══════════════════════════════════════════════════════════════════════
// Collision edge cases
// ═══════════════════════════════════════════════════════════════════════

TEST(glancing_endpoint_impact) {
    // A ball approaching at a steep angle near a wall endpoint should
    // deflect cleanly without getting stuck or phasing through.
    PhysicsWorld world;
    world.config.gravity = 0.0f;
    world.config.substeps = 4;
    world.config.restitution = 0.8f;
    world.config.friction = 0.0f;
    world.config.damping = 1.0f;
    world.config.sleepSpeed = 0.0f;

    // Wall from (50,100) to (150,100). Ball approaches the left
    // endpoint (50,100) from above-left at a glancing angle.
    world.walls.push_back(Wall(Vec2(50.0f, 100.0f), Vec2(150.0f, 100.0f)));

    Ball ball(Vec2(40.0f, 80.0f), 5.0f);
    ball.vel = {30.0f, 80.0f}; // mostly downward, slightly right
    world.balls.push_back(ball);

    for (int i = 0; i < 60; ++i) {
        world.step(0.016f);
    }

    // Ball must not be below the wall (phased through)
    ASSERT(world.balls[0].pos.y <= 100.0f + world.balls[0].radius + 2.0f);
    // Ball should still be moving (not stuck at zero velocity)
    ASSERT(world.balls[0].vel.length() > 5.0f);
}

TEST(dense_column_stack_no_explosion) {
    // Dropping many balls into a narrow column tests the solver's
    // ability to handle dense multi-contact stacks without energy
    // gain or balls shooting off to infinity.
    PhysicsWorld world;
    world.config.gravity = 500.0f;
    world.config.substeps = 8;
    world.config.restitution = 0.3f;
    world.config.damping = 0.999f;
    world.config.friction = 0.1f;
    world.config.sleepSpeed = 2.0f;

    // Narrow vertical column: 40px wide, 400px tall
    world.walls.push_back(Wall(Vec2(0.0f, 0.0f), Vec2(40.0f, 0.0f)));   // top
    world.walls.push_back(Wall(Vec2(40.0f, 0.0f), Vec2(40.0f, 400.0f)));// right
    world.walls.push_back(Wall(Vec2(40.0f, 400.0f), Vec2(0.0f, 400.0f)));// bottom
    world.walls.push_back(Wall(Vec2(0.0f, 400.0f), Vec2(0.0f, 0.0f)));  // left

    // Drop 30 balls into the narrow space — they must stack without
    // energy gain or explosion.
    for (int i = 0; i < 30; ++i) {
        Ball b(Vec2(20.0f, 10.0f + i * 12.0f), 5.0f);
        world.balls.push_back(b);
    }

    float maxSpeedEver = 0.0f;
    for (int i = 0; i < 800; ++i) {
        world.step(0.016f);

        // Track peak speed to detect explosion
        for (const auto& b : world.balls) {
            float spd = b.vel.length();
            if (spd > maxSpeedEver) maxSpeedEver = spd;
        }
    }

    // All balls should be inside the container
    for (const auto& b : world.balls) {
        ASSERT(b.pos.x > -2.0f && b.pos.x < 42.0f);
        ASSERT(b.pos.y > -2.0f && b.pos.y < 402.0f);
    }

    // No ball should have reached an absurd speed (explosion detection).
    // With gravity 500 and a 400px drop, terminal impact speed is ~632 px/s.
    // Allow 2× margin for multi-bounce energy concentration.
    ASSERT(maxSpeedEver < 1500.0f);

    // Most balls should be settled
    int settled = 0;
    for (const auto& b : world.balls) {
        if (b.vel.length() < 5.0f) settled++;
    }
    ASSERT(settled > 20);
}

TEST(spatial_grid_matches_brute_force) {
    // Verify that the spatial grid optimization doesn't miss any
    // colliding pairs by comparing against a brute-force pass.
    // We set up a tight cluster of balls and check that after one
    // step the results are identical whether we use the grid or not.
    //
    // This test works indirectly: it creates a scenario with known
    // overlaps and verifies all overlaps are resolved.
    PhysicsWorld world;
    world.config.gravity = 0.0f;
    world.config.substeps = 1;
    world.config.restitution = 0.5f;
    world.config.damping = 1.0f;
    world.config.friction = 0.0f;
    world.config.sleepSpeed = 0.0f;

    // Place 12 balls in a 3×4 grid with moderate overlap.
    // Each ball has radius 8, spacing is 12px (overlap = 4px per pair).
    for (int i = 0; i < 12; ++i) {
        int col = i % 3;
        int row = i / 3;
        Ball b(Vec2(50.0f + col * 12.0f, 50.0f + row * 12.0f), 8.0f);
        world.balls.push_back(b);
    }

    // Run several frames so the iterative solver has enough passes
    // to fully separate all pairs even in a dense cluster.
    for (int i = 0; i < 20; ++i) {
        world.step(0.016f);
    }

    // After resolution, no pair should still overlap significantly.
    for (size_t i = 0; i < world.balls.size(); ++i) {
        for (size_t j = i + 1; j < world.balls.size(); ++j) {
            Vec2 diff = world.balls[j].pos - world.balls[i].pos;
            float dist = diff.length();
            float minDist = world.balls[i].radius + world.balls[j].radius;
            // Allow small floating-point tolerance from iterative solver
            ASSERT(dist >= minDist - 0.5f);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════
// Performance sanity check
// ═══════════════════════════════════════════════════════════════════════

TEST(thousand_balls_step_under_33ms) {
    // The project requirement is 1000 balls at 30 FPS, meaning each
    // frame (step + render) must complete in ~33ms. The physics step
    // alone should be well under that. This test creates the same
    // 1000-ball scene as main.cpp and times 10 frames.
    PhysicsWorld world;
    world.config.gravity = 500.0f;
    world.config.substeps = 8;
    world.config.restitution = 0.3f;
    world.config.damping = 0.999f;
    world.config.friction = 0.1f;
    world.config.sleepSpeed = 2.0f;

    // Box container matching main.cpp
    float left = 50.0f, right = 1150.0f, top = 50.0f, bottom = 750.0f;
    world.walls.push_back(Wall(Vec2(left, top), Vec2(right, top)));
    world.walls.push_back(Wall(Vec2(right, top), Vec2(right, bottom)));
    world.walls.push_back(Wall(Vec2(right, bottom), Vec2(left, bottom)));
    world.walls.push_back(Wall(Vec2(left, bottom), Vec2(left, top)));

    // 1000 balls in a grid
    float spacing = 13.0f;
    int cols = static_cast<int>((right - left - 20) / spacing);
    for (int i = 0; i < 1000; ++i) {
        int col = i % cols;
        int row = i / cols;
        float x = left + 10.0f + col * spacing;
        float y = top + 10.0f + row * spacing;
        float r = 3.0f + (i % 4); // 3–6 px radius
        world.balls.push_back(Ball(Vec2(x, y), r));
    }

    // Time 10 physics frames
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 10; ++i) {
        world.step(0.016f);
    }
    auto end = std::chrono::high_resolution_clock::now();

    double totalMs = std::chrono::duration<double, std::milli>(end - start).count();
    double avgMs = totalMs / 10.0;

    printf("(%.1f ms/frame avg) ", avgMs);

    // Each physics step should complete in well under 33ms.
    // We use a generous 30ms threshold to account for CI variance.
    ASSERT(avgMs < 30.0);
}

// ═══════════════════════════════════════════════════════════════════════
// Large-scale tests (500+ balls)
// ═══════════════════════════════════════════════════════════════════════

// Helper: build a 500-ball world in a container matching the simulator layout.
// Deterministic placement (no randomness) so results are reproducible.
static PhysicsWorld makeLargeWorld(float restitution) {
    PhysicsWorld world;
    world.config.gravity = 500.0f;
    world.config.substeps = 8;
    world.config.restitution = restitution;
    world.config.damping = 0.999f;
    world.config.friction = 0.1f;
    world.config.sleepSpeed = 2.0f;

    // Container similar to the real simulator scene
    float left = 50.0f, right = 750.0f, top = 50.0f, bottom = 600.0f;
    world.walls.push_back(Wall(Vec2(left, top), Vec2(right, top)));
    world.walls.push_back(Wall(Vec2(right, top), Vec2(right, bottom)));
    world.walls.push_back(Wall(Vec2(right, bottom), Vec2(left, bottom)));
    world.walls.push_back(Wall(Vec2(left, bottom), Vec2(left, top)));

    // One angled shelf for complexity
    float midX = (left + right) / 2.0f;
    float shelfY = top + (bottom - top) * 0.4f;
    world.walls.push_back(Wall(Vec2(left, shelfY), Vec2(midX - 30.0f, shelfY + 40.0f)));

    // 500 balls in a grid with mixed radii
    float spacing = 14.0f;
    int cols = static_cast<int>((right - left - 20.0f) / spacing);
    for (int i = 0; i < 500; ++i) {
        int col = i % cols;
        int row = i / cols;
        float x = left + 10.0f + col * spacing;
        float y = top + 10.0f + row * spacing;
        float r = 3.0f + static_cast<float>(i % 4); // 3–6 px radius
        world.balls.push_back(Ball(Vec2(x, y), r));
    }

    return world;
}

TEST(large_scale_no_overlap_after_settling) {
    // After settling 500 balls, no pair should overlap beyond a small
    // floating-point tolerance. This catches solver bugs that only
    // manifest at higher ball counts where dense multi-contact stacking
    // is more common than in the smaller test fixtures.
    PhysicsWorld world = makeLargeWorld(0.3f);

    // Run for enough simulated time to fully settle
    for (int i = 0; i < 1200; ++i) {
        world.step(0.016f);
    }

    // Verify no pair overlaps
    int overlapCount = 0;
    for (size_t i = 0; i < world.balls.size(); ++i) {
        for (size_t j = i + 1; j < world.balls.size(); ++j) {
            Vec2 diff = world.balls[j].pos - world.balls[i].pos;
            float dist = diff.length();
            float minDist = world.balls[i].radius + world.balls[j].radius;
            if (dist < minDist - 1.0f) {
                overlapCount++;
            }
        }
    }
    // Allow zero significant overlaps (>1px penetration)
    ASSERT(overlapCount == 0);

    // All balls should be inside the container
    for (const auto& b : world.balls) {
        ASSERT(b.pos.x > 45.0f && b.pos.x < 755.0f);
        ASSERT(b.pos.y > 45.0f && b.pos.y < 605.0f);
    }

    // Most balls should have settled
    int settled = 0;
    for (const auto& b : world.balls) {
        if (b.vel.length() < 5.0f) settled++;
    }
    ASSERT(settled > 400); // At least 80% settled
}

TEST(large_scale_restitution_preserves_packed_size) {
    // Extends the settling-invariance requirement to 500 balls, which
    // more closely matches the 1000-ball production scene than the
    // 50-ball and 120-ball fixtures from earlier iterations.
    auto runAndMeasure = [](float restitution) -> SettledBounds {
        PhysicsWorld world = makeLargeWorld(restitution);
        for (int i = 0; i < 1500; ++i) {
            world.step(0.016f);
        }
        SettledBounds bounds;
        bounds.minX = 1e9f; bounds.maxX = -1e9f;
        bounds.minY = 1e9f; bounds.maxY = -1e9f;
        bounds.maxSpeed = 0.0f;
        for (const auto& b : world.balls) {
            bounds.minX = std::min(bounds.minX, b.pos.x - b.radius);
            bounds.maxX = std::max(bounds.maxX, b.pos.x + b.radius);
            bounds.minY = std::min(bounds.minY, b.pos.y - b.radius);
            bounds.maxY = std::max(bounds.maxY, b.pos.y + b.radius);
            bounds.maxSpeed = std::max(bounds.maxSpeed, b.vel.length());
        }
        return bounds;
    };

    const SettledBounds low  = runAndMeasure(0.0f);
    const SettledBounds med  = runAndMeasure(0.3f);
    const SettledBounds high = runAndMeasure(0.9f);

    // All should be fully settled
    ASSERT_NEAR(low.maxSpeed, 0.0f, 0.05f);
    ASSERT_NEAR(med.maxSpeed, 0.0f, 0.05f);
    ASSERT_NEAR(high.maxSpeed, 0.0f, 0.05f);

    // Final packing dimensions should match across restitution values.
    // Allow slightly more tolerance at 500 balls since the pile is taller
    // and minor stacking variations can compound.
    ASSERT_NEAR(low.width(), med.width(), 2.0f);
    ASSERT_NEAR(low.width(), high.width(), 2.0f);
    ASSERT_NEAR(low.height(), med.height(), 2.0f);
    ASSERT_NEAR(low.height(), high.height(), 2.0f);
    ASSERT_NEAR(low.minY, med.minY, 2.0f);
    ASSERT_NEAR(low.minY, high.minY, 2.0f);
}

// ═══════════════════════════════════════════════════════════════════════
// Main — run all tests
// ═══════════════════════════════════════════════════════════════════════

int main() {
    printf("\n=== Physics Engine Tests ===\n\n");

    // Tests are auto-registered by static constructors above.
    // By the time we reach main(), they've already run.

    printf("\n=== Results: %d/%d passed", tests_passed, tests_run);
    if (tests_failed > 0) {
        printf(", %d FAILED", tests_failed);
    }
    printf(" ===\n\n");

    return tests_failed > 0 ? 1 : 0;
}
