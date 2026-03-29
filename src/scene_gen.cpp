// scene_gen.cpp — Procedural scene generator for the physics simulator.
//
// Generates initial scene CSV files with various layouts of balls and walls.
// Useful for testing different configurations without manual CSV editing.
//
// Usage:
//   ./scene_gen <output.csv> [options]
//
// Options:
//   --balls N          Number of balls (default: 1000)
//   --radius-min R     Minimum ball radius (default: 3.0)
//   --radius-max R     Maximum ball radius (default: 6.0)
//   --layout TYPE      Layout type: grid, rain, funnel, pile (default: grid)
//   --width W          Container width (default: 1100)
//   --height H         Container height (default: 700)
//   --margin M         Wall margin from edge (default: 50)
//   --shelves N        Number of angled shelves (default: 2)
//   --seed S           Random seed (default: time-based)
//
// Layout types:
//   grid   — Balls placed in a regular grid filling the container
//   rain   — Balls placed in random positions across the top half
//   funnel — Balls concentrated at top-center, funneled by angled walls
//   pile   — Balls stacked in a narrow column above a V-shaped funnel

#include "sim_config.h"  // WINDOW_WIDTH, WINDOW_HEIGHT for default container size
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

// ── Random float in [lo, hi] ─────────────────────────────────────────
static float randFloat(float lo, float hi) {
    return lo + static_cast<float>(rand()) / RAND_MAX * (hi - lo);
}

// ── Container walls with optional shelves ─────────────────────────────
struct SceneConfig {
    int numBalls      = 1000;
    float radiusMin   = 3.0f;
    float radiusMax   = 6.0f;
    // Default container fills the simulation window minus a margin on each side.
    // Derived from WINDOW_WIDTH/HEIGHT in sim_config.h so all tools agree.
    float margin      = 50.0f;
    float width       = static_cast<float>(WINDOW_WIDTH)  - 2 * 50.0f;  // 1100
    float height      = static_cast<float>(WINDOW_HEIGHT) - 2 * 50.0f;  // 700
    int numShelves    = 2;
    std::string layout = "grid";
    unsigned seed     = 0;        // 0 = time-based
};

// ── Write wall to file ───────────────────────────────────────────────
static void writeWall(std::ofstream& f, float x1, float y1, float x2, float y2) {
    f << "wall," << x1 << "," << y1 << "," << x2 << "," << y2 << "\n";
}

// ── Write ball to file ───────────────────────────────────────────────
static void writeBall(std::ofstream& f, float x, float y, float r) {
    // No color assigned — the simulator or color_assign tool will handle that
    f << "ball," << x << "," << y << "," << r << "\n";
}

// ── Generate container walls + shelves ───────────────────────────────
// Creates a rectangular box with evenly-spaced angled shelves that
// alternate left-to-right. Shelves create interesting ball flow patterns.
static void generateWalls(std::ofstream& f, const SceneConfig& cfg) {
    float left   = cfg.margin;
    float right  = cfg.margin + cfg.width;
    float top    = cfg.margin;
    float bottom = cfg.margin + cfg.height;

    // Main rectangular container (clockwise winding)
    writeWall(f, left,  top,    right, top);     // Top
    writeWall(f, right, top,    right, bottom);  // Right
    writeWall(f, right, bottom, left,  bottom);  // Bottom
    writeWall(f, left,  bottom, left,  top);     // Left

    // Add angled shelves: evenly spaced, alternating direction.
    // Each shelf spans ~60% of the container width and has a slight angle.
    if (cfg.numShelves > 0) {
        float shelfSpacing = cfg.height / (cfg.numShelves + 1);
        float shelfWidth = cfg.width * 0.55f;
        float shelfAngle = 30.0f; // Vertical drop in pixels over the shelf length

        for (int i = 0; i < cfg.numShelves; ++i) {
            float y = top + shelfSpacing * (i + 1);
            bool fromLeft = (i % 2 == 0);

            if (fromLeft) {
                // Shelf from left wall angled down-right, with a gap on the right
                float x1 = left;
                float y1 = y;
                float x2 = left + shelfWidth;
                float y2 = y + shelfAngle;
                writeWall(f, x1, y1, x2, y2);
            } else {
                // Shelf from right wall angled down-left, with a gap on the left
                float x1 = right - shelfWidth;
                float y1 = y + shelfAngle;
                float x2 = right;
                float y2 = y;
                writeWall(f, x1, y1, x2, y2);
            }
        }
    }
}

// ── Grid layout ──────────────────────────────────────────────────────
// Balls placed in a regular grid with slight random offsets and velocities.
// This is the default layout matching the built-in simulator scene.
static void layoutGrid(std::ofstream& f, const SceneConfig& cfg) {
    float left  = cfg.margin + 10.0f;
    float right = cfg.margin + cfg.width - 10.0f;
    float top   = cfg.margin + 10.0f;

    float spacing = (cfg.radiusMax * 2.0f) + 1.0f;
    int cols = static_cast<int>((right - left) / spacing);
    if (cols < 1) cols = 1;

    for (int i = 0; i < cfg.numBalls; ++i) {
        int col = i % cols;
        int row = i / cols;
        float x = left + col * spacing + randFloat(-0.5f, 0.5f);
        float y = top + row * spacing + randFloat(-0.5f, 0.5f);
        float r = randFloat(cfg.radiusMin, cfg.radiusMax);
        writeBall(f, x, y, r);
    }
}

// ── Rain layout ──────────────────────────────────────────────────────
// Balls placed at random positions across the top half of the container.
// Creates a natural "raining down" effect when simulated.
static void layoutRain(std::ofstream& f, const SceneConfig& cfg) {
    float left   = cfg.margin + cfg.radiusMax + 2.0f;
    float right  = cfg.margin + cfg.width - cfg.radiusMax - 2.0f;
    float top    = cfg.margin + cfg.radiusMax + 2.0f;
    float midY   = cfg.margin + cfg.height * 0.4f;

    for (int i = 0; i < cfg.numBalls; ++i) {
        float x = randFloat(left, right);
        float y = randFloat(top, midY);
        float r = randFloat(cfg.radiusMin, cfg.radiusMax);
        writeBall(f, x, y, r);
    }
}

// ── Funnel layout ────────────────────────────────────────────────────
// Balls concentrated at top-center, with additional funnel walls that
// channel them into a narrow stream. Creates a dramatic pouring effect.
static void layoutFunnel(std::ofstream& f, const SceneConfig& cfg) {
    float centerX = cfg.margin + cfg.width / 2.0f;
    float top     = cfg.margin + cfg.radiusMax + 2.0f;
    float spread  = cfg.width * 0.3f; // How wide the initial ball cluster is

    // Add funnel walls: V-shape near the top-center
    float funnelTop = cfg.margin + cfg.height * 0.25f;
    float funnelBot = cfg.margin + cfg.height * 0.4f;
    float funnelWide = cfg.width * 0.4f;
    float funnelNarrow = 40.0f;

    writeWall(f, centerX - funnelWide / 2, funnelTop,
                 centerX - funnelNarrow / 2, funnelBot);
    writeWall(f, centerX + funnelWide / 2, funnelTop,
                 centerX + funnelNarrow / 2, funnelBot);

    for (int i = 0; i < cfg.numBalls; ++i) {
        float x = centerX + randFloat(-spread, spread);
        float y = randFloat(top, funnelTop - 5.0f);
        float r = randFloat(cfg.radiusMin, cfg.radiusMax);
        writeBall(f, x, y, r);
    }
}

// ── Pile layout ──────────────────────────────────────────────────────
// Balls stacked in a narrow column above a V-shaped funnel at the bottom.
// Tests dense stacking and pressure from many balls on top.
static void layoutPile(std::ofstream& f, const SceneConfig& cfg) {
    float centerX = cfg.margin + cfg.width / 2.0f;
    float top     = cfg.margin + cfg.radiusMax + 2.0f;
    float colWidth = cfg.width * 0.25f; // Narrow column

    // V-shaped funnel at the bottom
    float funnelTop = cfg.margin + cfg.height * 0.65f;
    float funnelBot = cfg.margin + cfg.height - 10.0f;
    writeWall(f, cfg.margin, funnelTop,
                 centerX - 20.0f, funnelBot);
    writeWall(f, cfg.margin + cfg.width, funnelTop,
                 centerX + 20.0f, funnelBot);

    // Place balls in a narrow column
    float spacing = (cfg.radiusMax * 2.0f) + 1.0f;
    int cols = static_cast<int>(colWidth / spacing);
    if (cols < 1) cols = 1;

    for (int i = 0; i < cfg.numBalls; ++i) {
        int col = i % cols;
        int row = i / cols;
        float x = centerX - colWidth / 2 + col * spacing + randFloat(-0.3f, 0.3f);
        float y = top + row * spacing + randFloat(-0.3f, 0.3f);
        float r = randFloat(cfg.radiusMin, cfg.radiusMax);
        writeBall(f, x, y, r);
    }
}

// ═══════════════════════════════════════════════════════════════════════
// main
// ═══════════════════════════════════════════════════════════════════════
int main(int argc, char* argv[]) {
    if (argc < 2) {
        fprintf(stderr,
            "Usage: %s <output.csv> [options]\n"
            "\n"
            "Options:\n"
            "  --balls N         Number of balls (default: 1000)\n"
            "  --radius-min R    Minimum ball radius (default: 3.0)\n"
            "  --radius-max R    Maximum ball radius (default: 6.0)\n"
            "  --layout TYPE     Layout: grid, rain, funnel, pile (default: grid)\n"
            "  --width W         Container width (default: 1100)\n"
            "  --height H        Container height (default: 700)\n"
            "  --margin M        Wall margin (default: 50)\n"
            "  --shelves N       Number of angled shelves (default: 2)\n"
            "  --seed S          Random seed (default: time-based)\n",
            argv[0]);
        return 1;
    }

    const char* outputPath = argv[1];
    SceneConfig cfg;

    // Parse options
    for (int i = 2; i < argc; ++i) {
        if (strcmp(argv[i], "--balls") == 0 && i + 1 < argc) {
            cfg.numBalls = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--radius-min") == 0 && i + 1 < argc) {
            cfg.radiusMin = static_cast<float>(atof(argv[++i]));
        } else if (strcmp(argv[i], "--radius-max") == 0 && i + 1 < argc) {
            cfg.radiusMax = static_cast<float>(atof(argv[++i]));
        } else if (strcmp(argv[i], "--layout") == 0 && i + 1 < argc) {
            cfg.layout = argv[++i];
        } else if (strcmp(argv[i], "--width") == 0 && i + 1 < argc) {
            cfg.width = static_cast<float>(atof(argv[++i]));
        } else if (strcmp(argv[i], "--height") == 0 && i + 1 < argc) {
            cfg.height = static_cast<float>(atof(argv[++i]));
        } else if (strcmp(argv[i], "--margin") == 0 && i + 1 < argc) {
            cfg.margin = static_cast<float>(atof(argv[++i]));
        } else if (strcmp(argv[i], "--shelves") == 0 && i + 1 < argc) {
            cfg.numShelves = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--seed") == 0 && i + 1 < argc) {
            cfg.seed = static_cast<unsigned>(atoi(argv[++i]));
        } else {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            return 1;
        }
    }

    // Seed the random number generator
    if (cfg.seed == 0) {
        cfg.seed = static_cast<unsigned>(time(nullptr));
    }
    srand(cfg.seed);

    // Validate
    if (cfg.numBalls < 1) cfg.numBalls = 1;
    if (cfg.radiusMin < 1.0f) cfg.radiusMin = 1.0f;
    if (cfg.radiusMax < cfg.radiusMin) cfg.radiusMax = cfg.radiusMin;

    // Open output file
    std::ofstream f(outputPath);
    if (!f.is_open()) {
        fprintf(stderr, "Cannot open '%s' for writing\n", outputPath);
        return 1;
    }

    // Write CSV header and comments
    f << "# Physics simulator scene — generated by scene_gen\n";
    f << "# Layout: " << cfg.layout << ", Balls: " << cfg.numBalls
      << ", Seed: " << cfg.seed << "\n";
    f << "# Ball format: ball,x,y,radius,r,g,b\n";
    f << "# Wall format: wall,x1,y1,x2,y2\n";
    f << "type,param1,param2,param3,param4,param5,param6\n";

    // Generate walls
    generateWalls(f, cfg);

    // Generate balls based on layout type
    if (cfg.layout == "grid") {
        layoutGrid(f, cfg);
    } else if (cfg.layout == "rain") {
        layoutRain(f, cfg);
    } else if (cfg.layout == "funnel") {
        layoutFunnel(f, cfg);
    } else if (cfg.layout == "pile") {
        layoutPile(f, cfg);
    } else {
        fprintf(stderr, "Unknown layout type: '%s'\n", cfg.layout.c_str());
        fprintf(stderr, "Available: grid, rain, funnel, pile\n");
        return 1;
    }

    f.close();

    printf("Scene generated: %s\n", outputPath);
    printf("  Layout: %s\n", cfg.layout.c_str());
    printf("  Balls: %d (radius %.1f–%.1f)\n", cfg.numBalls, cfg.radiusMin, cfg.radiusMax);
    printf("  Container: %.0fx%.0f + margin %.0f\n", cfg.width, cfg.height, cfg.margin);
    printf("  Shelves: %d\n", cfg.numShelves);
    printf("  Seed: %u\n", cfg.seed);

    return 0;
}
