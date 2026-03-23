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
            if (event.key.key == SDLK_ESCAPE || event.key.key == SDLK_Q) {
                return false;
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
    // Draw walls as white lines
    SDL_SetRenderDrawColor(renderer_, 200, 200, 200, 255);
    for (const auto& wall : world.walls) {
        SDL_RenderLine(renderer_, wall.p1.x, wall.p1.y, wall.p2.x, wall.p2.y);
    }

    // Draw balls — color based on speed for visual interest
    for (const auto& ball : world.balls) {
        float speed = ball.vel.length();
        // Map speed to color: slow=blue, medium=green, fast=red
        float t = std::min(speed / 300.0f, 1.0f);
        uint8_t r = static_cast<uint8_t>(50 + 200 * t);
        uint8_t g = static_cast<uint8_t>(100 + 100 * (1.0f - std::abs(t - 0.5f) * 2.0f));
        uint8_t b = static_cast<uint8_t>(200 * (1.0f - t));

        drawFilledCircle(ball.pos.x, ball.pos.y, ball.radius, r, g, b);
    }
}

// ── drawHUD — FPS and ball count overlay ────────────────────────────
void Renderer::drawHUD(float fps, int ballCount) {
    // SDL3 provides SDL_RenderDebugText for simple text without loading fonts.
    // It renders 8×8 monospaced characters at the given position.
    char buf[64];
    snprintf(buf, sizeof(buf), "FPS: %.0f  Balls: %d", fps, ballCount);

    // Scale up the debug text for readability (2×).
    SDL_SetRenderScale(renderer_, 2.0f, 2.0f);
    SDL_SetRenderDrawColor(renderer_, 240, 240, 100, 255); // yellow
    SDL_RenderDebugText(renderer_, 4.0f, 4.0f, buf);
    SDL_SetRenderScale(renderer_, 1.0f, 1.0f);
}

// ── drawFilledCircle ────────────────────────────────────────────────
// Renders a filled circle as a triangle fan using SDL_RenderGeometry.
void Renderer::drawFilledCircle(float cx, float cy, float r,
                                 uint8_t red, uint8_t green, uint8_t blue) {
    // Build triangle fan: center vertex + ring of vertices around it
    const int numVerts = CIRCLE_SEGMENTS + 2; // center + ring + closing
    std::vector<SDL_Vertex> verts(numVerts);

    SDL_FColor color = {red / 255.0f, green / 255.0f, blue / 255.0f, 1.0f};

    // Center vertex
    verts[0].position = {cx, cy};
    verts[0].color = color;

    // Ring vertices
    for (int i = 0; i <= CIRCLE_SEGMENTS; ++i) {
        float angle = (2.0f * M_PI * i) / CIRCLE_SEGMENTS;
        verts[i + 1].position = {cx + r * cosf(angle), cy + r * sinf(angle)};
        verts[i + 1].color = color;
    }

    // Build index list for triangle fan
    std::vector<int> indices;
    indices.reserve(CIRCLE_SEGMENTS * 3);
    for (int i = 0; i < CIRCLE_SEGMENTS; ++i) {
        indices.push_back(0);       // center
        indices.push_back(i + 1);   // current ring vertex
        indices.push_back(i + 2);   // next ring vertex
    }

    SDL_RenderGeometry(renderer_, nullptr,
                       verts.data(), numVerts,
                       indices.data(), static_cast<int>(indices.size()));
}
