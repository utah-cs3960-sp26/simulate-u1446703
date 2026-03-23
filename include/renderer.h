#pragma once
// renderer.h — SDL3-based rendering for the physics simulator.
// Handles window creation, drawing balls and walls, and the main loop.

#include "physics.h"
#include <SDL3/SDL.h>

// Window dimensions
constexpr int WINDOW_WIDTH  = 1200;
constexpr int WINDOW_HEIGHT = 800;

// ── Renderer class ──────────────────────────────────────────────────
// Wraps SDL3 window + renderer. Draws the physics world each frame.
class Renderer {
public:
    Renderer();
    ~Renderer();

    // Initialize SDL, create window and renderer. Returns false on failure.
    bool init();

    // Draw the current state of the world.
    void draw(const PhysicsWorld& world);

    // Poll events. Returns false if the user wants to quit.
    bool pollEvents();

    // Present the backbuffer to the screen.
    void present();

    // Clear the screen with background color.
    void clear();

    // Draw FPS and ball count overlay in top-left corner.
    void drawHUD(float fps, int ballCount);

private:
    SDL_Window*   window_   = nullptr;
    SDL_Renderer* renderer_ = nullptr;

    // Draw a filled circle using SDL3 (approximated with triangles).
    void drawFilledCircle(float cx, float cy, float r,
                          uint8_t red, uint8_t green, uint8_t blue);
};
