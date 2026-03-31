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

// ═══════════════════════════════════════════════════════════════════════
// SpatialGrid implementation
// ═══════════════════════════════════════════════════════════════════════

void SpatialGrid::clear() {
    // O(1) clear: just bump the generation counter. Stale cells are
    // treated as empty by insert() (which clears-on-first-touch) and
    // by forEachPair() (which skips cells with a stale generation).
    ++generation_;
}

void SpatialGrid::insert(int index, const Ball& ball) {
    // Compute the range of cells overlapped by the ball's bounding box.
    float invCell = 1.0f / cellSize;
    int x0 = static_cast<int>(std::floor((ball.pos.x - ball.radius) * invCell));
    int y0 = static_cast<int>(std::floor((ball.pos.y - ball.radius) * invCell));
    int x1 = static_cast<int>(std::floor((ball.pos.x + ball.radius) * invCell));
    int y1 = static_cast<int>(std::floor((ball.pos.y + ball.radius) * invCell));

    for (int cy = y0; cy <= y1; ++cy) {
        for (int cx = x0; cx <= x1; ++cx) {
            CellData& cell = cells_[{cx, cy}];
            // If this cell hasn't been touched this generation, clear it
            // and stamp the current generation.
            if (cell.generation != generation_) {
                cell.indices.clear();
                cell.generation = generation_;
            }
            cell.indices.push_back(index);
        }
    }
}

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

// ── Helper: classify whether a wall contact is on a slope ───────────
// A wall is "sloped" if gravity has a significant tangential (downslope)
// component along its surface. On such walls, balls should slide rather
// than freeze. We check the wall normal's vertical component: for a
// perfectly horizontal floor normal.y = -1 (upward), so the tangential
// gravity is zero. For a tilted wall the tangential gravity is
// g * sqrt(1 - ny²). We flag walls where that exceeds a threshold.
//
// The threshold is chosen relative to the friction and damping forces
// that the solver applies: if the downslope acceleration exceeds what
// friction can hold, the ball should be sliding and must NOT be
// aggressively slept.
static bool isWallSloped(const Vec2& contactNormal, float gravity) {
    // contactNormal points from wall toward ball center (outward).
    // For a horizontal floor, this is (0, -1) (upward).
    // ny is the vertical component: -1 for floor, 0 for vertical wall.
    float ny = contactNormal.y;

    // Only consider walls that provide some vertical support (not vertical
    // side walls). Vertical walls (|ny| < 0.1) are not "shelves."
    if (std::abs(ny) < 0.1f) return false;

    // Tangential gravity component along the wall surface:
    // g_tangential = g * sin(angle) = g * sqrt(1 - cos²(angle))
    // where cos(angle) = |ny| (normal dot gravity_dir).
    float sinAngle = std::sqrt(std::max(0.0f, 1.0f - ny * ny));
    float tangentialAccel = gravity * sinAngle;

    // If downslope gravity exceeds ~10 px/s², the ball should be sliding
    // and shouldn't be aggressively slept. This corresponds to ~1.1° for
    // g=500, which catches any meaningfully sloped surface.
    return tangentialAccel > 10.0f;
}

// ── CCD: swept circle vs line segment ───────────────────────────────
// Given a ball moving from oldPos to newPos, determine if it crossed
// through the wall during this substep. If it did, returns the
// fraction t ∈ [0,1] at which the ball's edge first touches the wall's
// infinite line. Returns -1 if no crossing occurs.
//
// This is a conservative check: it uses the wall's infinite line for
// the swept test, then clamps to the segment in the caller. This
// prevents tunneling through walls without the expense of a full
// swept-circle-vs-segment intersection.
static float sweptCircleVsLine(const Vec2& oldPos, const Vec2& newPos,
                               float radius, const Wall& wall) {
    // Wall normal (unit length, pointing "inward" for CW-wound boxes)
    Vec2 n = wall.normal();

    // Signed distance from old and new positions to the wall's infinite line.
    // dist = dot(pos - wall.p1, normal)
    float distOld = (oldPos - wall.p1).dot(n);
    float distNew = (newPos - wall.p1).dot(n);

    // If ball was already on the "wrong" side or didn't cross, skip.
    // We only care about transitions from outside (dist > radius) to
    // inside (dist < radius).
    if (distOld >= radius && distNew >= radius) return -1.0f;
    if (distOld <= -radius) return -1.0f; // Was already deep inside

    // Find the fraction t where the signed distance equals radius
    // (the ball's edge just touches the line).
    float denom = distOld - distNew;
    if (std::abs(denom) < 1e-8f) return -1.0f; // Parallel motion

    float t = (distOld - radius) / denom;
    if (t < 0.0f || t > 1.0f) return -1.0f;

    // Verify the contact point is within the segment bounds (not just
    // the infinite line). Project the ball center at time t onto the
    // segment and check that the projection parameter is in [0,1].
    Vec2 hitPos = oldPos + (newPos - oldPos) * t;
    Vec2 wallDelta = wall.p2 - wall.p1;
    float wallLenSq = wallDelta.lengthSq();
    if (wallLenSq < 1e-12f) return -1.0f;
    float s = (hitPos - wall.p1).dot(wallDelta) / wallLenSq;

    // Allow a small margin beyond the endpoints to catch near-endpoint
    // tunneling that would be resolved by the normal solver.
    if (s < -0.05f || s > 1.05f) return -1.0f;

    return t;
}

// ═══════════════════════════════════════════════════════════════════════
// PhysicsWorld::step  — main entry point per frame
// ═══════════════════════════════════════════════════════════════════════
void PhysicsWorld::step(float dt) {
    // Clamp dt to avoid spiral-of-death if frame takes too long
    if (dt > 0.033f) dt = 0.033f;

    float subDt = dt / static_cast<float>(config.substeps);

    // Save pre-frame positions for per-frame stuck detection.
    // Stuck balls have high velocity but zero net displacement over
    // a full frame because collision corrections cancel their movement.
    // Also clear the per-frame contact flag (set during substeps below).
    for (auto& b : balls) {
        b.prevPos = b.pos;
        b.inContactThisFrame = false; // Cleared once per frame, set during any substep
        b.onSlopedWallThisFrame = false; // Cleared once per frame
    }

    for (int s = 0; s < config.substeps; ++s) {
        // Clear per-substep contact flags.
        for (auto& b : balls) {
            b.inRestingContact = false;
            b.onSlopedWall = false;
        }

        integrateVelocities(subDt);
        integratePositions(subDt);

        // Multiple constraint-solving iterations per substep for stability.
        // More iterations = more accurate stacking, but more CPU cost.
        // 8 iterations handles stacks ~8 balls deep per substep, which
        // combined with 8 substeps covers ~64 layers of pile depth.
        for (int iter = 0; iter < config.solverIterations; ++iter) {
            solveBallWallCollisions();
            solveBallBallCollisions();
        }

        // Post-solver damping: absorb energy injected by position corrections.
        // Applied after the solver so it directly targets the velocity
        // artifacts from constraint resolution in dense stacks.
        applyPostSolverDamping();

        // Contact-aware sleep: zero velocity of balls in resting contact
        // that are sliding slowly (below contactSleepSpeed), and detect
        // balls stuck at terminal velocity (high speed but zero displacement).
        applyContactDamping();

        // Apply sleep each substep to aggressively kill micro-vibrations
        // from the iterative constraint solver. This prevents energy
        // accumulation in dense stacks. The sleep is also applied at
        // frame-end (below) for final cleanup.
        applySleepThreshold();
    }

    // Per-frame stuck detection: if a ball has high velocity but hasn't
    // moved since the start of the frame, it's trapped against a surface.
    // Zero its velocity. This catches balls at terminal velocity (~250
    // px/s) where gravity pushes them into the pile and collision
    // correction pushes them back every substep, yielding zero net
    // displacement. Only check balls moving fast enough that their
    // expected displacement (speed × dt) should be >> stuckThreshold.
    // A speed threshold of 100 px/s × 0.016s = 1.6 px expected
    // displacement, well above the 0.1 px stuckThreshold.
    if (config.sleepSpeed > 0.0f) {
        float stuckThresholdSq = config.stuckThreshold * config.stuckThreshold;
        constexpr float STUCK_SPEED_MIN = 100.0f; // Only check fast balls
        float stuckSpeedMinSq = STUCK_SPEED_MIN * STUCK_SPEED_MIN;
        for (auto& b : balls) {
            if (!b.hasBeenActive) continue;
            if (b.vel.lengthSq() < stuckSpeedMinSq) continue;
            Vec2 displacement = b.pos - b.prevPos;
            if (displacement.lengthSq() < stuckThresholdSq) {
                b.vel = {0.0f, 0.0f};
            }
        }
    }

    // Final per-frame sleep pass.
    applySleepThreshold();
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
// integratePositions — Euler step with CCD guard against wall tunneling.
// After the naive pos += vel*dt, check each ball against every wall for
// tunneling. If a ball would have crossed a wall, clip it back to the
// wall surface and reflect its velocity. This handles the case where
// substeps alone aren't enough (e.g., a ball accelerated to extreme
// speed by many overlapping contacts in a single frame).
// ═══════════════════════════════════════════════════════════════════════
void PhysicsWorld::integratePositions(float subDt) {
    for (auto& b : balls) {
        Vec2 oldPos = b.pos;
        b.pos += b.vel * subDt;

        // CCD: check if the ball tunneled through any wall this substep.
        for (const auto& wall : walls) {
            float t = sweptCircleVsLine(oldPos, b.pos, b.radius, wall);
            if (t >= 0.0f) {
                // Treat swept wall hits exactly like overlap-based wall hits
                // for settling purposes. Without these flags, a ball clipped
                // back to the wall by CCD looks like it had "no contact this
                // frame", so gravity rebuilds velocity every frame while the
                // ball stays pinned in place. That showed up as residual KE in
                // the large settling tests even though the ball never escaped.
                b.inRestingContact = true;
                b.inContactThisFrame = true;

                // Move ball back to the contact point (edge touches wall)
                b.pos = oldPos + (b.pos - oldPos) * t;

                // Classify slope for the CCD wall contact too, so balls
                // swept onto shelves still get the slope exemption.
                Vec2 n_ccd = wall.normal();
                if (isWallSloped(n_ccd, config.gravity)) {
                    b.onSlopedWall = true;
                    b.onSlopedWallThisFrame = true;
                }

                // Reflect velocity along wall normal.
                // Keep the CCD response aligned with solveBallWallCollisions():
                // slow contacts use zero restitution so resting balls do not
                // keep re-bouncing purely because they touched the wall via the
                // swept path instead of the overlap path.
                Vec2 n = wall.normal();
                float velN = b.vel.dot(n);
                if (velN < 0.0f) {
                    float effectiveRestitution = config.restitution;
                    if (std::abs(velN) < config.bounceThreshold) {
                        effectiveRestitution = 0.0f;
                    }

                    Vec2 velNormal = n * velN;
                    Vec2 velTangent = b.vel - velNormal;
                    b.vel = velTangent * (1.0f - config.friction)
                          - velNormal * effectiveRestitution;
                }
                // Don't break — check remaining walls too (corner cases)
            }
        }
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
                // Any overlap means the ball is in contact — mark for
                // contact-aware sleep regardless of approach speed.
                ball.inRestingContact = true;
                ball.inContactThisFrame = true; // Persists across all substeps this frame

                Vec2 normal;
                // We'll classify slope after computing the contact normal below.
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

                // Push ball out so it just touches the wall.
                // Full correction for walls — they're immovable boundaries.
                float penetration = ball.radius - dist;
                ball.pos += normal * penetration;

                // Classify whether this wall contact is on a slope.
                // Balls on sloped walls should slide off, not freeze.
                if (isWallSloped(normal, config.gravity)) {
                    ball.onSlopedWall = true;
                    ball.onSlopedWallThisFrame = true;
                }

                // Reflect velocity along normal with restitution.
                // Use restitution=0 for slow contacts (resting under gravity)
                // to prevent micro-bounces from injecting energy in stacks.
                float velAlongNormal = ball.vel.dot(normal);
                if (velAlongNormal < 0.0f) {
                    float effectiveRestitution = config.restitution;
                    if (std::abs(velAlongNormal) < config.bounceThreshold) {
                        effectiveRestitution = 0.0f;
                    }

                    Vec2 velNormal = normal * velAlongNormal;
                    Vec2 velTangent = ball.vel - velNormal;

                    ball.vel = velTangent * (1.0f - config.friction)
                             - velNormal * effectiveRestitution;
                }
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════
// solveBallBallCollisions
// Uses a spatial hash grid to narrow the search to nearby pairs instead
// of the naive O(n²) pairwise check. Each ball is inserted into the
// grid cells its bounding box overlaps, then only pairs sharing a cell
// are tested. A pair-visited set prevents duplicate resolution when a
// pair spans multiple cells.
//
// For each overlapping pair:
//   1. Separate them along the center-to-center axis (proportional to
//      inverse mass so lighter balls move more).
//   2. Apply an impulse to swap/reduce the velocity components along
//      the collision normal, scaled by restitution.
// ═══════════════════════════════════════════════════════════════════════

void PhysicsWorld::solveBallBallCollisions() {
    const int n = static_cast<int>(balls.size());
    if (n < 2) return;

    // Determine optimal cell size: 2× the largest ball radius ensures
    // each ball touches at most 4 cells (2×2 neighborhood).
    float maxRadius = 0.0f;
    for (const auto& b : balls) {
        if (b.radius > maxRadius) maxRadius = b.radius;
    }
    grid_.cellSize = std::max(maxRadius * 2.0f, 1.0f);

    // Populate the grid with all balls.
    grid_.clear();
    for (int i = 0; i < n; ++i) {
        grid_.insert(i, balls[i]);
    }

    // NOTE: forEachPair may report the same (i,j) from multiple cells.
    // We do NOT use a visited set — instead we rely on idempotency:
    // after the first resolution pushes a pair apart, subsequent calls
    // for the same pair find distSq >= minDist² and early-out cheaply.
    // This avoids per-frame hash-set allocation and lookup overhead.

    grid_.forEachPair([&](int i, int j) {

        Ball& a = balls[i];
        Ball& b = balls[j];

        Vec2 diff = b.pos - a.pos;
        float distSq = diff.lengthSq();
        float minDist = a.radius + b.radius;

        // Early-out: no overlap
        if (distSq >= minDist * minDist) return;

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
        // Use Baumgarte-style partial correction with slop to reduce
        // energy injection from cascading corrections in dense stacks.
        // Only penetration exceeding positionSlop is corrected, and
        // only a fraction (positionCorrectionFactor) per solver pass.
        float penetration = minDist - dist;
        float correctable = std::max(0.0f, penetration - config.positionSlop);
        float correction = correctable * config.positionCorrectionFactor;

        const float invMassA = 1.0f / a.mass;
        const float invMassB = 1.0f / b.mass;
        const float totalInvMass = invMassA + invMassB;

        // Each ball moves proportional to its inverse mass
        a.pos -= normal * (correction * invMassA / totalInvMass);
        b.pos += normal * (correction * invMassB / totalInvMass);

        // ── Velocity impulse ─────────────────────────────────────
        // Use the standard relative velocity of B with respect to A.
        // With the collision normal pointing from A to B, a negative
        // dot product means the pair is closing and needs an impulse.
        Vec2 relVel = b.vel - a.vel;
        float velAlongNormal = relVel.dot(normal);

        // Any overlap means these balls are in contact — mark for
        // contact-aware sleep. This catches all cases: slow resting
        // contacts, fast impacts that zero velocity (r=0), and
        // position corrections for separating pairs.
        a.inRestingContact = true;
        b.inRestingContact = true;
        a.inContactThisFrame = true; // Persists across all substeps this frame
        b.inContactThisFrame = true;

        // Only resolve if balls are approaching each other
        if (velAlongNormal > 0.0f) return;

        // Use restitution=0 for slow contacts (resting in a stack) to
        // prevent micro-bounces from accumulating energy in dense packing.
        float effectiveRestitution = config.restitution;
        if (std::abs(velAlongNormal) < config.bounceThreshold) {
            effectiveRestitution = 0.0f;
        }

        // Impulse magnitude with restitution
        float impulseMag = -(1.0f + effectiveRestitution) * velAlongNormal / totalInvMass;

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
    }); // end forEachPair lambda
}

// ═══════════════════════════════════════════════════════════════════════
// applyPostSolverDamping — absorb energy injected by position corrections.
// The constraint solver moves overlapping balls apart, which implicitly
// creates velocity (the ball is at a new position). In dense stacks,
// these corrections cascade and inject net kinetic energy. This damping
// pass runs AFTER the solver to directly target that injected energy,
// helping tall piles converge to rest.
// ═══════════════════════════════════════════════════════════════════════
void PhysicsWorld::applyPostSolverDamping() {
    for (auto& b : balls) {
        b.vel = b.vel * config.damping;
    }
}

// ═══════════════════════════════════════════════════════════════════════
// applyContactDamping — contact-aware sleep (static friction).
// Balls in resting contact moving below contactSleepSpeed are zeroed.
// This catches shelf-sliding equilibria where gravity's slope component
// balances damping/friction at a steady 5-40 px/s. Only applies to
// Phase 2 balls (hasBeenActive=true). Disabled when sleepSpeed=0.
// ═══════════════════════════════════════════════════════════════════════
void PhysicsWorld::applyContactDamping() {
    if (config.sleepSpeed <= 0.0f || config.contactSleepSpeed <= 0.0f) return;

    float contactThresholdSq = config.contactSleepSpeed * config.contactSleepSpeed;
    for (auto& b : balls) {
        if (!b.inRestingContact || !b.hasBeenActive) continue;
        // Skip balls on sloped walls — they should slide off, not freeze.
        // Without this check, balls on angled shelves get caught by the
        // elevated contact sleep threshold and pile up permanently.
        if (b.onSlopedWall) continue;
        if (b.vel.lengthSq() < contactThresholdSq) {
            b.vel = {0.0f, 0.0f};
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════
// applySleepThreshold — two-phase sleep system for balls.
//
// Phase 1 — Never been active (!hasBeenActive):
//   Counter-based delay. Each substep with speed < threshold increments
//   sleepCounter. Sleep triggers at sleepDelay. If speed exceeds the
//   threshold before the counter fills, hasBeenActive becomes true and
//   the ball transitions to Phase 2 permanently.
//   This gives gravity sleepDelay substeps to build velocity so balls
//   starting from rest can actually fall.
//
// Phase 2 — Has been active (hasBeenActive):
//   Uses inContactThisFrame (not per-substep inRestingContact) to distinguish:
//   A) Ball had contact during ANY substep this frame (inContactThisFrame=true):
//      → Instant sleep. Aggressively kills constraint-solver micro-vibrations.
//      This covers settled pile balls and those briefly separated by the solver
//      (they still have inContactThisFrame=true from earlier substeps this frame).
//   B) Ball had NO contacts in ALL substeps this frame (inContactThisFrame=false):
//      → Low-threshold sleep (0.3 px/s) instead of instant-sleep (5.0 px/s).
//      The threshold is deliberately set BELOW one substep of gravity:
//        gravity * subDt = 500 * (0.016/8) = 1.0 px/s > 0.3 px/s.
//      After any zeroing, gravity immediately builds the velocity to 1.0 px/s
//      which exceeds the low threshold → the ball falls freely. This prevents
//      Phase 2 balls floating in air from being permanently frozen by instant
//      sleep (which zeroed each 1.0 px/s gravity contribution before it could
//      accumulate to the full 5.0 px/s threshold).
//      Solver residuals (typically < 0.1 px/s) are still zeroed by the 0.3
//      threshold, preventing floating garbage noise.
//
// Using inContactThisFrame (per-frame) instead of inRestingContact (per-substep)
// prevents the oscillation cycle: solver separates a pile ball → no contact in
// last substep → counter-based sleep → gravity builds → ball falls into pile →
// repeat. With inContactThisFrame, the ball is "in pile" for the full frame.
// ═══════════════════════════════════════════════════════════════════════
void PhysicsWorld::applySleepThreshold() {
    float threshold = config.sleepSpeed * config.sleepSpeed;
    // Low threshold for Phase 2 balls not in contact this frame.
    // Must be < gravity * subDt (typically 1.0 px/s) so gravity can
    // immediately escape it after zeroing. 0.3 px/s is safely below this.
    static constexpr float FLOATING_SLEEP_SQ = 0.09f; // 0.3 px/s

    for (auto& b : balls) {
        if (b.vel.lengthSq() < threshold) {
            if (!b.hasBeenActive) {
                // Phase 1: counter-based delay for initial gravity buildup
                b.sleepCounter++;
                if (b.sleepCounter >= config.sleepDelay) {
                    b.vel = {0.0f, 0.0f};
                    b.sleepCounter = 0;
                }
            } else if (b.inContactThisFrame && !b.onSlopedWallThisFrame) {
                // Phase 2 with contact on FLAT surfaces: instant sleep to
                // kill solver micro-vibrations in dense stacks. Balls on
                // flat floors/walls can be aggressively slept at sleepSpeed
                // because gravity has no tangential component to drive motion.
                b.vel = {0.0f, 0.0f};
                b.sleepCounter = 0;
            } else if (b.inContactThisFrame && b.onSlopedWallThisFrame) {
                // Phase 2 on a SLOPED wall: use the floating threshold
                // instead of instant sleep. This lets gravity build enough
                // velocity to slide the ball downhill. Without this, the
                // 5 px/s instant-sleep threshold catches the 1 px/s gravity
                // contribution each substep and the ball freezes on the slope.
                // The 0.3 px/s threshold is below one substep of gravity
                // (g * subDt = 500 * 0.002 = 1.0 px/s), so gravity escapes
                // it immediately after any zeroing.
                if (b.vel.lengthSq() < FLOATING_SLEEP_SQ) {
                    b.vel = {0.0f, 0.0f};
                }
                b.sleepCounter = 0;
            } else {
                // Phase 2 with NO contact this frame (floating in air):
                // Only zero if velocity is below the floating threshold so
                // gravity can immediately overcome it next substep.
                if (b.vel.lengthSq() < FLOATING_SLEEP_SQ) {
                    b.vel = {0.0f, 0.0f};
                }
                b.sleepCounter = 0;
            }
        } else {
            // Ball is moving fast enough — mark as active permanently
            b.hasBeenActive = true;
            b.sleepCounter = 0;
        }
    }
}
