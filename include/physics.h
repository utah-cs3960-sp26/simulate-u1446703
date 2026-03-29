#pragma once
// physics.h — Core physics types and simulation engine.
// Contains Ball, Wall, and PhysicsWorld. All physics logic lives here,
// completely independent of SDL so it can be tested in isolation.

#include "sim_config.h"  // DefaultPhysicsConfig, WINDOW_WIDTH/HEIGHT
#include <vector>
#include <unordered_map>
#include <cmath>
#include <cstdint>
#include <utility>

// ── 2D Vector helper ────────────────────────────────────────────────
struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;

    Vec2() = default;
    Vec2(float x, float y) : x(x), y(y) {}

    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    Vec2 operator*(float s) const { return {x * s, y * s}; }
    Vec2& operator+=(const Vec2& o) { x += o.x; y += o.y; return *this; }
    Vec2& operator-=(const Vec2& o) { x -= o.x; y -= o.y; return *this; }

    float dot(const Vec2& o) const { return x * o.x + y * o.y; }
    float length() const { return std::sqrt(x * x + y * y); }
    float lengthSq() const { return x * x + y * y; }

    // Returns normalized vector; returns {0,0} if length is ~zero.
    Vec2 normalized() const {
        float len = length();
        if (len < 1e-8f) return {0.0f, 0.0f};
        return {x / len, y / len};
    }
};

// ── Ball color ─────────────────────────────────────────────────────
// RGB color for rendering. If hasColor is false, the renderer uses its
// default speed-based coloring. CSV I/O and the color-assign tool set
// this field so balls can carry persistent colors through the pipeline.
struct BallColor {
    uint8_t r = 0, g = 0, b = 0;
    bool hasColor = false; // false → use default speed-based coloring
};

// ── Ball ────────────────────────────────────────────────────────────
// A circular body affected by gravity and collisions.
struct Ball {
    Vec2 pos;       // Center position in world coords
    Vec2 vel;       // Velocity in pixels/second
    float radius;   // Radius in pixels
    float mass;     // Mass (proportional to area by default)
    BallColor color; // Optional persistent color for CSV/rendering

    // Sleep system: two-phase approach.
    // Phase 1 (never been active): counter-based delay gives gravity
    //   sleepDelay substeps to build velocity above the threshold.
    //   sleepCounter counts up from 0; sleep triggers at sleepDelay.
    // Phase 2 (has been active): instant sleep when speed < threshold.
    //   This aggressively kills constraint-solver micro-vibrations.
    // hasBeenActive is set to true the first time speed exceeds the
    // threshold. Once set, the ball never gets the grace period again.
    int sleepCounter = 0;
    bool hasBeenActive = false;

    // Resting contact tracking: set to true each substep when the ball
    // has a collision overlap resolved. Cleared at the start of each
    // substep. Used by the contact-aware sleep pass to apply an elevated
    // sleep threshold to balls sliding slowly on surfaces.
    bool inRestingContact = false;

    // Previous position: saved at the start of each substep for stuck
    // detection. If a ball is in contact but its position hasn't changed,
    // it's trapped at terminal velocity and should be zeroed.
    Vec2 prevPos;

    Ball() : radius(5.0f), mass(1.0f) {}
    Ball(Vec2 pos, float radius)
        : pos(pos), vel({0, 0}), radius(radius),
          mass(radius * radius) {} // mass ~ area (pi cancels out in ratios)
};

// ── Wall ────────────────────────────────────────────────────────────
// An immovable line segment defined by two endpoints.
// The "inside" of the simulation is on the LEFT side when traveling
// from p1 to p2, determined by the outward normal.
struct Wall {
    Vec2 p1, p2;

    Wall() = default;
    Wall(Vec2 p1, Vec2 p2) : p1(p1), p2(p2) {}

    // Outward-facing normal (points LEFT of p1→p2 direction).
    // For a box where walls go clockwise, this points inward — which is
    // actually what we want for pushing balls back inside.
    Vec2 normal() const {
        Vec2 d = p2 - p1;
        Vec2 n = {-d.y, d.x}; // rotate 90° CCW = left-hand normal
        return n.normalized();
    }
};

// ── Physics configuration ───────────────────────────────────────────
struct PhysicsConfig {
    float gravity       = 500.0f;   // Downward acceleration (px/s^2)
    float restitution   = 0.3f;     // Coefficient of restitution [0..1]
    float friction      = 0.1f;     // Tangential friction coefficient
    float damping       = 0.998f;   // Per-substep velocity damping (multiplicative)
    int   substeps      = 8;        // Physics substeps per frame
    int   solverIterations = 8;     // Constraint solver iterations per substep
    float sleepSpeed    = 5.0f;     // Velocity threshold to zero-out balls
    int   sleepDelay    = 8;        // Consecutive low-speed substeps before sleeping
                                    // (matches default substeps, giving gravity a full
                                    //  frame to build velocity before sleep triggers)

    // ── Settling stabilization parameters ────────────────────────────
    // These reduce energy injection from position corrections in dense stacks.

    // Baumgarte factor: fraction of penetration corrected per solver pass.
    // 1.0 = full correction (most stable for resting stacks).
    // Lower values converge more softly but leave residual overlap.
    float positionCorrectionFactor = 1.0f;

    // Slop: minimum penetration before position correction kicks in.
    // Small overlaps (< slop) are tolerated to avoid constant micro-corrections
    // that inject energy. Visually unnoticeable at 0.5px.
    float positionSlop = 0.5f;

    // Restitution velocity threshold: approach speeds below this value
    // are treated as resting contacts (restitution=0). This prevents
    // micro-bounces in dense stacks from injecting energy. Typical
    // gravity-induced approach in a substep is ~1-2 px/s, so a threshold
    // of ~20 px/s catches resting contacts while preserving real bounces.
    float bounceThreshold = 30.0f;

    // Contact sleep speed: elevated sleep threshold for balls in contact.
    // Balls sliding on angled surfaces reach an equilibrium speed where
    // gravity input = damping+friction loss. That speed (typically 5-40
    // px/s) is above the normal sleepSpeed. For balls in contact, we
    // use this higher threshold to zero them out, simulating static
    // friction. Only applies to Phase 2 balls (hasBeenActive=true).
    float contactSleepSpeed = 40.0f;

    // Position-based stuck detection threshold: if a ball is in contact
    // and its position moved less than this distance during a substep, the
    // ball is stuck at terminal velocity against a surface (gravity pushes
    // it in, collision correction pushes it back). Zero its velocity.
    // This catches the rare case where a ball vibrates at 250 px/s but
    // its net displacement is zero each substep.
    float stuckThreshold = 0.1f;
};

// ── Apply centralized default config ────────────────────────────────
// Sets all PhysicsConfig fields to the values from DefaultPhysicsConfig.
// Callers can then override individual fields (e.g., restitution).
// This eliminates boilerplate in main.cpp, color_assign.cpp, and tests.
inline void applyDefaultConfig(PhysicsConfig& config) {
    config.gravity          = DefaultPhysicsConfig::gravity;
    config.restitution      = DefaultPhysicsConfig::restitution;
    config.substeps         = DefaultPhysicsConfig::substeps;
    config.solverIterations = DefaultPhysicsConfig::solverIterations;
    config.damping          = DefaultPhysicsConfig::damping;
    config.friction         = DefaultPhysicsConfig::friction;
    config.sleepSpeed       = DefaultPhysicsConfig::sleepSpeed;
    config.bounceThreshold  = DefaultPhysicsConfig::bounceThreshold;
}

// ── Spatial hash grid ───────────────────────────────────────────────
// Divides space into uniform cells so ball-ball collision only checks
// nearby pairs instead of all O(n²) combinations. Each ball is inserted
// into every cell its bounding box overlaps. Then only pairs that share
// at least one cell are tested. For 1000 balls of radius 3–6 px in a
// 1200×800 window, a cell size of ~20–30 px keeps each cell sparse.
struct CellKey {
    int32_t cx, cy;
    bool operator==(const CellKey& o) const { return cx == o.cx && cy == o.cy; }
};

struct CellKeyHash {
    // Fast spatial hash: multiply-shift on two 32-bit coordinates.
    std::size_t operator()(const CellKey& k) const {
        // Large primes give good distribution for small integer keys.
        return static_cast<std::size_t>(k.cx * 73856093) ^
               static_cast<std::size_t>(k.cy * 19349663);
    }
};

// ── Per-cell data with generation counter ────────────────────────────
// Instead of clearing every cell each solver iteration, we track a
// generation counter. A cell is only "active" if its generation matches
// the grid's current generation. This turns clear() from O(cells) to O(1).
struct CellData {
    std::vector<int> indices;
    uint32_t generation = 0;   // Last generation this cell was written to
};

class SpatialGrid {
public:
    // cellSize should be >= 2 * max ball radius so every ball touches
    // at most 4 cells (the 2×2 neighborhood of its center cell).
    float cellSize = 20.0f;

    // Clear all cells — O(1) via generation counter bump.
    void clear();

    // Insert ball index into every cell its bounding box overlaps.
    void insert(int index, const Ball& ball);

    // Iterate over every unique (i, j) pair that shares a cell and invoke
    // the callback. The callback signature is void(int i, int j).
    template<typename Func>
    void forEachPair(Func&& fn) const;

private:
    uint32_t generation_ = 0;  // Incremented on each clear()

    // Maps cell coordinate → list of ball indices + generation stamp.
    std::unordered_map<CellKey, CellData, CellKeyHash> cells_;
};

// ── Template implementation (must be in header) ─────────────────────
template<typename Func>
void SpatialGrid::forEachPair(Func&& fn) const {
    // For each cell active in the current generation, test all pairs
    // within it. Stale cells (generation mismatch) are skipped.
    // Duplicate callbacks for the same pair from different cells are
    // possible; the caller handles this via idempotency.
    for (const auto& [key, cellData] : cells_) {
        if (cellData.generation != generation_) continue;
        const auto& indices = cellData.indices;
        const size_t n = indices.size();
        for (size_t a = 0; a < n; ++a) {
            for (size_t b = a + 1; b < n; ++b) {
                int i = indices[a];
                int j = indices[b];
                if (i > j) { int tmp = i; i = j; j = tmp; }
                fn(i, j);
            }
        }
    }
}

// ── PhysicsWorld ────────────────────────────────────────────────────
// Owns all balls and walls; advances the simulation each frame.
// Uses substep integration + iterative position correction to keep
// balls from overlapping or phasing through geometry.
class PhysicsWorld {
public:
    PhysicsConfig config;
    std::vector<Ball> balls;
    std::vector<Wall> walls;

    PhysicsWorld() = default;

    // Advance simulation by dt seconds.
    // Internally subdivides into config.substeps sub-frames for stability.
    void step(float dt);

    // Return total kinetic energy (useful for tests / settling detection).
    float totalKineticEnergy() const;

private:
    // Spatial grid reused across solver iterations to avoid per-frame alloc.
    SpatialGrid grid_;

    // ── Per-substep helpers ──────────────────────────────────────────

    // Apply gravity and damping to all ball velocities.
    void integrateVelocities(float subDt);

    // Move balls by their velocity * subDt.
    void integratePositions(float subDt);

    // Detect and resolve ball-vs-wall penetrations.
    // Pushes ball out and reflects velocity component along wall normal.
    void solveBallWallCollisions();

    // Detect and resolve ball-vs-ball overlaps using the spatial grid.
    // Only tests pairs that share a grid cell, bringing average cost
    // from O(n²) down to ~O(n) for uniformly distributed balls.
    // Duplicate pairs from overlapping cells are handled by idempotency
    // (resolved pairs no longer overlap, so re-checks are cheap no-ops).
    void solveBallBallCollisions();

    // Zero out velocity of nearly-stopped balls to help settling.
    void applySleepThreshold();

    // Apply additional velocity damping after the constraint solver to
    // absorb energy injected by position corrections in dense stacks.
    void applyPostSolverDamping();

    // Apply stronger damping to balls that are in contact with walls or
    // other balls. This breaks the equilibrium where gravity input equals
    // damping loss for balls sliding on angled surfaces.
    void applyContactDamping();
};
