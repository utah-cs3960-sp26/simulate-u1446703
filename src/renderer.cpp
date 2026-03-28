// renderer.cpp — SDL3 rendering implementation.
// Draws balls as filled circles and walls as thick lines.
// Uses SDL3's geometry rendering (triangles for circles).

#include "renderer.h"
#include <cmath>
#include <vector>
#include <cstdio>

// Number of segments used to approximate a circle
constexpr int CIRCLE_SEGMENTS = 16;

Renderer::Renderer() = default;

Renderer::~Renderer() {
    if (renderer_) SDL_DestroyRenderer(renderer_);
    if (window_)   SDL_DestroyWindow(window_);
    SDL_Quit();
}

bool Renderer::init() {
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return false;
    }

    window_ = SDL_CreateWindow("Physics Simulator",
                               WINDOW_WIDTH, WINDOW_HEIGHT,
                               SDL_WINDOW_RESIZABLE);
    if (!window_) {
        fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        return false;
    }

    renderer_ = SDL_CreateRenderer(window_, nullptr);
    if (!renderer_) {
        fprintf(stderr, "SDL_CreateRenderer failed: %s\n", SDL_GetError());
        return false;
    }

    return true;
}

bool Renderer::pollEvents() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_EVENT_QUIT) return false;
        if (event.type == SDL_EVENT_KEY_DOWN) {
            switch (event.key.key) {
                case SDLK_ESCAPE:
                case SDLK_Q:
                    return false;

                // SPACE: toggle pause/resume
                case SDLK_SPACE:
                    paused_ = !paused_;
                    break;

                // RIGHT ARROW or N: single-step (only when paused)
                case SDLK_RIGHT:
                case SDLK_N:
                    if (paused_) stepRequested_ = true;
                    break;

                // R: restart simulation
                case SDLK_R:
                    restartRequested_ = true;
                    break;

                // UP ARROW or EQUALS/PLUS: increase speed (max 4x)
                case SDLK_UP:
                case SDLK_EQUALS:
                    if (speedMultiplier_ < 4.0f) speedMultiplier_ *= 2.0f;
                    break;

                // DOWN ARROW or MINUS: decrease speed (min 0.25x)
                case SDLK_DOWN:
                case SDLK_MINUS:
                    if (speedMultiplier_ > 0.25f) speedMultiplier_ *= 0.5f;
                    break;

                // 1: reset speed to 1x
                case SDLK_1:
                    speedMultiplier_ = 1.0f;
                    break;

                default:
                    break;
            }
        }
    }
    return true;
}

void Renderer::clear() {
    SDL_SetRenderDrawColor(renderer_, 20, 20, 30, 255);
    SDL_RenderClear(renderer_);
}

void Renderer::present() {
    SDL_RenderPresent(renderer_);
}

void Renderer::draw(const PhysicsWorld& world) {
    // Draw walls as thick white lines (rendered as quads for visibility).
    // Each wall is drawn as a filled rectangle oriented along the wall segment,
    // with a configurable half-width for thickness.
    constexpr float WALL_HALF_WIDTH = 2.0f; // 4px total thickness
    for (const auto& wall : world.walls) {
        // Compute a perpendicular offset from the wall direction
        Vec2 d = wall.p2 - wall.p1;
        float len = d.length();
        if (len < 1e-6f) continue;
        Vec2 perp = {-d.y / len * WALL_HALF_WIDTH, d.x / len * WALL_HALF_WIDTH};

        // 4 corners of the thick wall quad
        SDL_Vertex wallVerts[4];
        SDL_FColor wallColor = {0.78f, 0.78f, 0.78f, 1.0f}; // light gray

        wallVerts[0].position = {wall.p1.x - perp.x, wall.p1.y - perp.y};
        wallVerts[0].color = wallColor;
        wallVerts[1].position = {wall.p1.x + perp.x, wall.p1.y + perp.y};
        wallVerts[1].color = wallColor;
        wallVerts[2].position = {wall.p2.x + perp.x, wall.p2.y + perp.y};
        wallVerts[2].color = wallColor;
        wallVerts[3].position = {wall.p2.x - perp.x, wall.p2.y - perp.y};
        wallVerts[3].color = wallColor;

        // Two triangles to fill the quad
        int wallIndices[6] = {0, 1, 2, 0, 2, 3};
        SDL_RenderGeometry(renderer_, nullptr, wallVerts, 4, wallIndices, 6);
    }

    // Draw balls — outline then fill for visual separation in packs.
    // The outline is a slightly larger dark circle drawn behind the fill.
    constexpr float OUTLINE_WIDTH = 0.8f; // px beyond ball radius
    for (const auto& ball : world.balls) {
        // Draw dark outline circle (slightly larger than the ball)
        drawFilledCircle(ball.pos.x, ball.pos.y,
                         ball.radius + OUTLINE_WIDTH, 10, 10, 15);
    }

    for (const auto& ball : world.balls) {
        uint8_t r, g, b;
        if (ball.color.hasColor) {
            // Use the ball's assigned color (from CSV or color-assign tool)
            r = ball.color.r;
            g = ball.color.g;
            b = ball.color.b;
        } else {
            // Default: map speed to color (slow=blue, medium=green, fast=red)
            float speed = ball.vel.length();
            float t = std::min(speed / 300.0f, 1.0f);
            r = static_cast<uint8_t>(50 + 200 * t);
            g = static_cast<uint8_t>(100 + 100 * (1.0f - std::abs(t - 0.5f) * 2.0f));
            b = static_cast<uint8_t>(200 * (1.0f - t));
        }
        drawFilledCircle(ball.pos.x, ball.pos.y, ball.radius, r, g, b);
    }
}

// ── drawHUD — FPS, ball count, KE, and controls overlay ──────────────
void Renderer::drawHUD(float fps, int ballCount, float ke) {
    // SDL3 provides SDL_RenderDebugText for simple text without loading fonts.
    // It renders 8×8 monospaced characters at the given position.
    SDL_SetRenderScale(renderer_, 2.0f, 2.0f);

    // Line 1: FPS, ball count, kinetic energy
    char buf[128];
    snprintf(buf, sizeof(buf), "FPS: %.0f  Balls: %d  KE: %.0f", fps, ballCount, ke);
    SDL_SetRenderDrawColor(renderer_, 240, 240, 100, 255); // yellow
    SDL_RenderDebugText(renderer_, 4.0f, 4.0f, buf);

    // Line 2: Speed and pause state
    if (paused_) {
        snprintf(buf, sizeof(buf), "PAUSED  Speed: %.2fx", speedMultiplier_);
        SDL_SetRenderDrawColor(renderer_, 255, 100, 100, 255); // red
    } else {
        snprintf(buf, sizeof(buf), "Speed: %.2fx", speedMultiplier_);
        SDL_SetRenderDrawColor(renderer_, 150, 220, 150, 255); // green
    }
    SDL_RenderDebugText(renderer_, 4.0f, 16.0f, buf);

    // Line 3: Controls help (bottom of screen, small)
    SDL_SetRenderScale(renderer_, 1.5f, 1.5f);
    SDL_SetRenderDrawColor(renderer_, 160, 160, 180, 255); // dim gray
    snprintf(buf, sizeof(buf),
             "SPACE:pause  RIGHT/N:step  UP/DOWN:speed  R:restart  1:reset  Q:quit");
    // Position at the bottom of the window (accounting for scale)
    float helpY = (WINDOW_HEIGHT / 1.5f) - 12.0f;
    SDL_RenderDebugText(renderer_, 4.0f, helpY, buf);

    SDL_SetRenderScale(renderer_, 1.0f, 1.0f);
}

// ── saveScreenshot — capture framebuffer to BMP ─────────────────────
bool Renderer::saveScreenshot(const char* filename) {
    // Create a surface matching the window dimensions, read pixels into it,
    // then save as BMP. Works with offscreen/software renderers too.
    SDL_Surface* surface = SDL_RenderReadPixels(renderer_, nullptr);
    if (!surface) {
        fprintf(stderr, "SDL_RenderReadPixels failed: %s\n", SDL_GetError());
        return false;
    }

    bool ok = SDL_SaveBMP(surface, filename);
    if (!ok) {
        fprintf(stderr, "SDL_SaveBMP failed: %s\n", SDL_GetError());
    }
    SDL_DestroySurface(surface);
    return ok;
}

// ── Precomputed unit circle for drawFilledCircle ────────────────────
// Sine/cosine values and the triangle-fan index list are constant for
// every circle, so we compute them once at startup. Only the vertex
// positions and colors vary per call.
static struct CircleGeometry {
    float cosTable[CIRCLE_SEGMENTS + 1];
    float sinTable[CIRCLE_SEGMENTS + 1];
    int   indices[CIRCLE_SEGMENTS * 3];

    CircleGeometry() {
        for (int i = 0; i <= CIRCLE_SEGMENTS; ++i) {
            float angle = (2.0f * static_cast<float>(M_PI) * i) / CIRCLE_SEGMENTS;
            cosTable[i] = cosf(angle);
            sinTable[i] = sinf(angle);
        }
        for (int i = 0; i < CIRCLE_SEGMENTS; ++i) {
            indices[i * 3 + 0] = 0;       // center
            indices[i * 3 + 1] = i + 1;   // current ring vertex
            indices[i * 3 + 2] = i + 2;   // next ring vertex
        }
    }
} circleGeo;

// ── drawFilledCircle ────────────────────────────────────────────────
// Renders a filled circle as a triangle fan using SDL_RenderGeometry.
// Uses precomputed trig tables and a static vertex buffer to avoid
// per-call heap allocations (important when drawing 1000+ balls/frame).
void Renderer::drawFilledCircle(float cx, float cy, float r,
                                 uint8_t red, uint8_t green, uint8_t blue) {
    constexpr int numVerts = CIRCLE_SEGMENTS + 2; // center + ring + closing
    static SDL_Vertex verts[numVerts];

    SDL_FColor color = {red / 255.0f, green / 255.0f, blue / 255.0f, 1.0f};

    // Center vertex
    verts[0].position = {cx, cy};
    verts[0].color = color;

    // Ring vertices using precomputed trig
    for (int i = 0; i <= CIRCLE_SEGMENTS; ++i) {
        verts[i + 1].position = {cx + r * circleGeo.cosTable[i],
                                  cy + r * circleGeo.sinTable[i]};
        verts[i + 1].color = color;
    }

    SDL_RenderGeometry(renderer_, nullptr,
                       verts, numVerts,
                       circleGeo.indices, CIRCLE_SEGMENTS * 3);
}
