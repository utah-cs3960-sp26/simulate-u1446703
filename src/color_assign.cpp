// color_assign.cpp — Color assignment tool for the physics simulator.
//
// Takes an initial scene CSV file and an image (BMP, PNG, JPG, TGA).
// Runs the simulation until the balls settle, then assigns each ball a
// color based on the pixel in the image at the ball's final position.
// Writes the result to an output CSV with the original starting positions
// but the new colors derived from the final resting positions.
//
// Usage:
//   ./color_assign <input.csv> <image> <output.csv> [restitution] [frames]
//
// Arguments:
//   input.csv    — Initial scene CSV (balls + walls)
//   image        — Image to sample colors from (BMP, PNG, JPG, TGA)
//   output.csv   — Output CSV with colors assigned based on final positions
//   restitution  — Coefficient of restitution [0..1], default 0.3
//   frames       — Number of simulation frames to run, default 600
//
// The tool works in 3 phases:
//   1. Load the initial scene from CSV
//   2. Run the physics simulation for the specified number of frames
//   3. For each ball, sample the image color at its final position
//      and write the original scene with the new colors to the output CSV

#include "physics.h"
#include "csv_io.h"
#include "sim_config.h"  // DefaultPhysicsConfig, WINDOW_WIDTH, WINDOW_HEIGHT

// stb_image for multi-format image loading (PNG, JPG, BMP, TGA, etc.)
// The implementation is compiled here; other files only include the header.
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <string>

// ── Image wrapper ──────────────────────────────────────────────────
// Holds loaded image data from stb_image. Provides pixel sampling at
// arbitrary (x, y) coordinates. Supports any format stb_image can
// decode: PNG, JPG, BMP, TGA, PSD, GIF, HDR, PIC, PNM.
struct Image {
    unsigned char* data = nullptr;
    int width = 0;
    int height = 0;
    int channels = 0; // Actual channels in loaded data (always forced to 3)

    ~Image() {
        if (data) stbi_image_free(data);
    }

    // Load an image file. Returns true on success.
    bool load(const char* path) {
        // Force 3 channels (RGB) regardless of source format.
        // This simplifies pixel sampling — no alpha handling needed.
        data = stbi_load(path, &width, &height, &channels, 3);
        if (!data) {
            fprintf(stderr, "Failed to load image '%s': %s\n",
                    path, stbi_failure_reason());
            return false;
        }
        channels = 3; // We forced 3 channels above
        return true;
    }

    // Sample RGB color at pixel (x, y). Returns false if out of bounds.
    bool sample(int x, int y, uint8_t& r, uint8_t& g, uint8_t& b) const {
        if (x < 0 || x >= width || y < 0 || y >= height) return false;
        const unsigned char* pixel = data + (y * width + x) * 3;
        r = pixel[0];
        g = pixel[1];
        b = pixel[2];
        return true;
    }

    // Non-copyable (owns stb-allocated memory)
    Image() = default;
    Image(const Image&) = delete;
    Image& operator=(const Image&) = delete;
};

int main(int argc, char* argv[]) {
    if (argc < 4) {
        fprintf(stderr, "Usage: %s <input.csv> <image> <output.csv> [restitution] [frames]\n",
                argv[0]);
        fprintf(stderr, "\nSupported image formats: BMP, PNG, JPG, TGA, PSD, GIF, HDR, PIC, PNM\n");
        return 1;
    }

    const char* inputCSV  = argv[1];
    const char* imagePath  = argv[2];
    const char* outputCSV = argv[3];

    float restitution = 0.3f;
    if (argc > 4) {
        restitution = static_cast<float>(atof(argv[4]));
        if (restitution < 0.0f) restitution = 0.0f;
        if (restitution > 1.0f) restitution = 1.0f;
    }

    int totalFrames = 600;
    if (argc > 5) {
        totalFrames = atoi(argv[5]);
        if (totalFrames < 1) totalFrames = 600;
    }

    printf("Color assignment tool:\n");
    printf("  Input:       %s\n", inputCSV);
    printf("  Image:       %s\n", imagePath);
    printf("  Output:      %s\n", outputCSV);
    printf("  Restitution: %.2f\n", restitution);
    printf("  Frames:      %d\n", totalFrames);

    // ── Phase 1: Load the initial scene ─────────────────────────────
    // Use the same physics config as the main simulator for consistent behavior.
    PhysicsWorld world;
    applyDefaultConfig(world.config);
    world.config.restitution = restitution;

    if (!loadSceneFromCSV(inputCSV, world)) {
        fprintf(stderr, "Failed to load scene from '%s'\n", inputCSV);
        return 1;
    }

    // Save the original starting positions so we can write them back
    // to the output CSV with the new colors.
    struct OriginalBall {
        Vec2 pos;
        float radius;
    };
    std::vector<OriginalBall> originals;
    originals.reserve(world.balls.size());
    for (const auto& ball : world.balls) {
        originals.push_back({ball.pos, ball.radius});
    }

    // ── Phase 2: Run the simulation ─────────────────────────────────
    constexpr float dt = 1.0f / 60.0f;
    printf("Simulating %d frames...\n", totalFrames);

    for (int frame = 0; frame < totalFrames; ++frame) {
        world.step(dt);

        // Progress reporting every 20%
        if (frame > 0 && frame % (totalFrames / 5) == 0) {
            float ke = world.totalKineticEnergy();
            printf("  Frame %d/%d — KE=%.1f\n", frame, totalFrames, ke);
        }
    }

    printf("Simulation complete. KE=%.1f\n", world.totalKineticEnergy());

    // ── Phase 3: Load the image and sample colors ───────────────────
    // Uses stb_image for multi-format support (PNG, JPG, BMP, TGA, etc.)
    Image image;
    if (!image.load(imagePath)) {
        return 1;
    }

    printf("Image loaded: %dx%d\n", image.width, image.height);

    // For each ball, sample the image at its final position.
    // If the final position is out of bounds, assign a default gray.
    int outOfBounds = 0;
    for (size_t i = 0; i < world.balls.size(); ++i) {
        Ball& ball = world.balls[i];

        // Map ball's final world position to image pixel coordinates.
        // The image is assumed to cover the same coordinate space as
        // the simulation window (WINDOW_WIDTH x WINDOW_HEIGHT).
        // Scale proportionally if the image is a different size.
        float scaleX = static_cast<float>(image.width) / static_cast<float>(WINDOW_WIDTH);
        float scaleY = static_cast<float>(image.height) / static_cast<float>(WINDOW_HEIGHT);

        int px = static_cast<int>(ball.pos.x * scaleX);
        int py = static_cast<int>(ball.pos.y * scaleY);

        uint8_t r, g, b;
        if (image.sample(px, py, r, g, b)) {
            ball.color.r = r;
            ball.color.g = g;
            ball.color.b = b;
            ball.color.hasColor = true;
        } else {
            // Out of bounds — assign default gray
            ball.color.r = 128;
            ball.color.g = 128;
            ball.color.b = 128;
            ball.color.hasColor = true;
            ++outOfBounds;
        }

        // Restore original position for the output CSV so the file
        // describes the initial scene with assigned colors.
        ball.pos = originals[i].pos;
        ball.radius = originals[i].radius;
    }

    if (outOfBounds > 0) {
        printf("Warning: %d balls had final positions outside the image bounds\n", outOfBounds);
    }

    // ── Phase 4: Save the colored scene ─────────────────────────────
    if (!saveSceneToCSV(outputCSV, world)) {
        fprintf(stderr, "Failed to save colored scene to '%s'\n", outputCSV);
        return 1;
    }

    printf("Done! Colored scene written to '%s'\n", outputCSV);
    return 0;
}
