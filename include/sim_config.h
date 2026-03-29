#pragma once
// sim_config.h — Shared simulation constants.
// These are used by the simulator, color_assign, scene_gen, and tests.
// Kept separate from renderer.h to avoid SDL dependency for non-rendering tools.

// ── Window / simulation area dimensions ─────────────────────────────
// These define the coordinate space for the simulation. All tools
// (simulator, color_assign, scene_gen) use these same dimensions so
// ball positions, wall coordinates, and image color sampling are consistent.
constexpr int WINDOW_WIDTH  = 1200;
constexpr int WINDOW_HEIGHT = 800;

// ── Default physics configuration ───────────────────────────────────
// Centralized defaults so main.cpp and color_assign.cpp stay in sync.
// Individual tools can override these if needed.
struct DefaultPhysicsConfig {
    static constexpr float gravity           = 500.0f;
    static constexpr float restitution       = 0.3f;
    static constexpr int   substeps          = 8;
    static constexpr int   solverIterations  = 8;
    static constexpr float damping           = 0.998f;
    static constexpr float friction          = 0.1f;
    static constexpr float sleepSpeed        = 5.0f;
    static constexpr float bounceThreshold   = 30.0f;
};

// applyDefaultConfig is declared in physics.h (which includes this header)
// so that it has access to the full PhysicsConfig definition. It applies
// all DefaultPhysicsConfig values, letting callers override individual fields.
