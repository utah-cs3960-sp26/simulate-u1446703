// physics.cpp — Implementation of the physics simulation engine.
// Key design decisions:
//   1. Substep integration: each frame is divided into N substeps so that
//      fast-moving balls don't tunnel through thin walls.
//   2. Iterative constraint solving: after moving balls, we run multiple
//      passes of collision resolution to prevent overlaps.
//   3. Position-based correction: overlapping balls/walls are pushed apart
//      directly, then an impulse is applied to the velocity. This prevents
//      the "jitter then explode" failure mode common with pure impulse methods.

#include "physics.h"
#include <algorithm>
#include <cmath>

// ── Utility: closest point on a line segment to a point ─────────────
// Used for ball-vs-wall collision detection.
static Vec2 closestPointOnSegment(const Vec2& p, const Vec2& a, const Vec2& b) {
    Vec2 ab = b - a;
    float abLenSq = ab.lengthSq();
    if (abLenSq < 1e-12f) return a; // Degenerate segment

    // Project p onto line ab, clamp parameter t to [0,1]
    float t = (p - a).dot(ab) / abLenSq;
    t = std::max(0.0f, std::min(1.0f, t));
    return a + ab * t;
}

// ═══════════════════════════════════════════════════════════════════════
// PhysicsWorld::step  — main entry point per frame
// ═══════════════════════════════════════════════════════════════════════
void PhysicsWorld::step(float dt) {
    // Clamp dt to avoid spiral-of-death if frame takes too long
    if (dt > 0.033f) dt = 0.033f;

    float subDt = dt / static_cast<float>(config.substeps);

    for (int s = 0; s < config.substeps; ++s) {
        integrateVelocities(subDt);
        integratePositions(subDt);

        // Multiple constraint-solving iterations per substep for stability.
        // More iterations = more accurate stacking, but more CPU cost.
        constexpr int solverIterations = 4;
        for (int iter = 0; iter < solverIterations; ++iter) {
            solveBallWallCollisions();
            solveBallBallCollisions();
        }

        applySleepThreshold();
    }
}

// ═══════════════════════════════════════════════════════════════════════
// Kinetic energy — sum of 0.5 * m * v^2 for all balls
// ═══════════════════════════════════════════════════════════════════════
float PhysicsWorld::totalKineticEnergy() const {
    float total = 0.0f;
    for (const auto& b : balls) {
        total += 0.5f * b.mass * b.vel.lengthSq();
    }
    return total;
}

// ═══════════════════════════════════════════════════════════════════════
// integrateVelocities — apply gravity + damping to velocity
// ═══════════════════════════════════════════════════════════════════════
void PhysicsWorld::integrateVelocities(float subDt) {
    for (auto& b : balls) {
        // Gravity acts downward (positive Y)
        b.vel.y += config.gravity * subDt;

        // Light damping to help settling (models air resistance)
        b.vel = b.vel * config.damping;
    }
}

// ═══════════════════════════════════════════════════════════════════════
// integratePositions — Euler step: pos += vel * dt
// ═══════════════════════════════════════════════════════════════════════
void PhysicsWorld::integratePositions(float subDt) {
    for (auto& b : balls) {
        b.pos += b.vel * subDt;
    }
}

// ═══════════════════════════════════════════════════════════════════════
// solveBallWallCollisions
// For each ball, check distance to each wall segment. If the ball
// overlaps the wall, push it out along the wall's normal and reflect
// the velocity component that points into the wall.
// ═══════════════════════════════════════════════════════════════════════
void PhysicsWorld::solveBallWallCollisions() {
    for (auto& ball : balls) {
        for (const auto& wall : walls) {
            // Recompute the clamped segment parameter locally instead of only
            // calling the helper so we can distinguish interior contacts from
            // endpoint contacts. That distinction matters when the ball center
            // lands exactly on the closest point: segment interiors should use
            // the wall normal, while exact endpoint hits need a point-contact
            // fallback normal or they only reflect one axis.
            Vec2 wallDelta = wall.p2 - wall.p1;
            float wallLengthSq = wallDelta.lengthSq();
            float t = 0.0f;
            if (wallLengthSq > 1e-12f) {
                t = (ball.pos - wall.p1).dot(wallDelta) / wallLengthSq;
                t = std::max(0.0f, std::min(1.0f, t));
            }

            // Find closest point on wall segment to ball center.
            Vec2 closest = wall.p1 + wallDelta * t;
            Vec2 diff = ball.pos - closest;
            float dist = diff.length();

            // Check for overlap
            if (dist < ball.radius) {
                Vec2 normal;
                if (dist < 1e-6f) {
                    const bool atEndpoint = (t <= 1e-4f) || (t >= 1.0f - 1e-4f);

                    if (atEndpoint && ball.vel.lengthSq() > 1e-8f) {
                        // Exact endpoint hits have no unique geometric normal
                        // because the center sits on the corner point. In that
                        // case, reflect along the incoming direction so the
                        // response behaves like a circle colliding with a point.
                        normal = ball.vel * -1.0f;
                        normal = normal.normalized();
                    } else {
                        // Exact interior contacts still have a well-defined wall
                        // normal, so use the segment orientation as before.
                        normal = wall.normal();
                    }
                } else {
                    // Normal points from wall toward ball center
                    normal = diff.normalized();
                }

                // Push ball out so it just touches the wall
                float penetration = ball.radius - dist;
                ball.pos += normal * penetration;

                // Reflect velocity along normal with restitution
                float velAlongNormal = ball.vel.dot(normal);
                if (velAlongNormal < 0.0f) {
                    // Decompose velocity into normal and tangent components
                    Vec2 velNormal = normal * velAlongNormal;
                    Vec2 velTangent = ball.vel - velNormal;

                    // Apply restitution to normal component, friction to tangent
                    ball.vel = velTangent * (1.0f - config.friction)
                             - velNormal * config.restitution;
                }
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════
// solveBallBallCollisions
// O(n^2) pairwise check. For ~1000 balls this is ~500K checks per
// solver iteration, which is acceptable at this scale. If we needed
// more, we'd add a spatial hash grid.
//
// For each overlapping pair:
//   1. Separate them along the center-to-center axis (proportional to
//      inverse mass so lighter balls move more).
//   2. Apply an impulse to swap/reduce the velocity components along
//      the collision normal, scaled by restitution.
// ═══════════════════════════════════════════════════════════════════════
void PhysicsWorld::solveBallBallCollisions() {
    const size_t n = balls.size();

    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            Ball& a = balls[i];
            Ball& b = balls[j];

            Vec2 diff = b.pos - a.pos;
            float distSq = diff.lengthSq();
            float minDist = a.radius + b.radius;

            // Early-out: no overlap
            if (distSq >= minDist * minDist) continue;

            float dist = std::sqrt(distSq);
            Vec2 normal;

            if (dist < 1e-6f) {
                // Balls are exactly overlapping — pick arbitrary direction
                normal = {0.0f, 1.0f};
                dist = 0.0f;
            } else {
                normal = diff * (1.0f / dist); // b-a direction, normalized
            }

            // ── Position correction: push apart ──────────────────────
            float penetration = minDist - dist;
            const float invMassA = 1.0f / a.mass;
            const float invMassB = 1.0f / b.mass;
            const float totalInvMass = invMassA + invMassB;

            // Each ball moves proportional to its inverse mass
            a.pos -= normal * (penetration * invMassA / totalInvMass);
            b.pos += normal * (penetration * invMassB / totalInvMass);

            // ── Velocity impulse ─────────────────────────────────────
            // Use the standard relative velocity of B with respect to A.
            // With the collision normal pointing from A to B, a negative
            // dot product means the pair is closing and needs an impulse.
            Vec2 relVel = b.vel - a.vel;
            float velAlongNormal = relVel.dot(normal);

            // Only resolve if balls are approaching each other
            if (velAlongNormal > 0.0f) continue;

            // Impulse magnitude with restitution
            float impulseMag = -(1.0f + config.restitution) * velAlongNormal / totalInvMass;

            Vec2 impulse = normal * impulseMag;
            a.vel -= impulse * invMassA;
            b.vel += impulse * invMassB;

            // ── Tangential friction ──────────────────────────────────
            Vec2 tangent = relVel - normal * velAlongNormal;
            float tangentLen = tangent.length();
            if (tangentLen > 1e-6f) {
                tangent = tangent * (1.0f / tangentLen);
                float frictionImpulse = -relVel.dot(tangent) / totalInvMass;
                // Clamp friction to Coulomb model
                float maxFriction = config.friction * std::abs(impulseMag);
                frictionImpulse = std::max(-maxFriction, std::min(maxFriction, frictionImpulse));

                Vec2 fricVec = tangent * frictionImpulse;
                a.vel -= fricVec * invMassA;
                b.vel += fricVec * invMassB;
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════
// applySleepThreshold — zero out velocity of very slow balls.
// This prevents endless micro-vibrations and helps the simulation
// converge to a stable resting state.
// ═══════════════════════════════════════════════════════════════════════
void PhysicsWorld::applySleepThreshold() {
    float threshold = config.sleepSpeed * config.sleepSpeed;
    for (auto& b : balls) {
        if (b.vel.lengthSq() < threshold) {
            b.vel = {0.0f, 0.0f};
        }
    }
}
