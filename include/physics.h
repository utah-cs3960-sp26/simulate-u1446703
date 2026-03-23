#pragma once
// physics.h — Core physics types and simulation engine.
// Contains Ball, Wall, and PhysicsWorld. All physics logic lives here,
// completely independent of SDL so it can be tested in isolation.

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

// ── Ball ────────────────────────────────────────────────────────────
// A circular body affected by gravity and collisions.
struct Ball {
    Vec2 pos;       // Center position in world coords
    Vec2 vel;       // Velocity in pixels/second
    float radius;   // Radius in pixels
    float mass;     // Mass (proportional to area by default)

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
    float damping       = 0.999f;   // Per-frame velocity damping
    int   substeps      = 8;        // Physics substeps per frame
    float sleepSpeed    = 2.0f;     // Velocity threshold to zero-out balls
};

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

class SpatialGrid {
public:
    // cellSize should be >= 2 * max ball radius so every ball touches
    // at most 4 cells (the 2×2 neighborhood of its center cell).
    float cellSize = 20.0f;

    // Clear all cells — called once per solver iteration before re-insert.
    void clear();

    // Insert ball index into every cell its bounding box overlaps.
    void insert(int index, const Ball& ball);

    // Iterate over every unique (i, j) pair that shares a cell and invoke
    // the callback. The callback signature is void(int i, int j).
    template<typename Func>
    void forEachPair(Func&& fn) const;

private:
    // Maps cell coordinate → list of ball indices in that cell.
    std::unordered_map<CellKey, std::vector<int>, CellKeyHash> cells_;
};

// ── Template implementation (must be in header) ─────────────────────
template<typename Func>
void SpatialGrid::forEachPair(Func&& fn) const {
    // For each cell, test all pairs within it. To avoid reporting the
    // same (i,j) pair from two different cells, we only call the callback
    // when i < j and rely on the fact that every colliding pair shares
    // at least one cell. Duplicate callbacks for the same pair from
    // different cells are possible; the caller must guard against that
    // (e.g., by checking a visited set or by accepting idempotent work).
    for (const auto& [key, indices] : cells_) {
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
    void solveBallBallCollisions();

    // Zero out velocity of nearly-stopped balls to help settling.
    void applySleepThreshold();
};
