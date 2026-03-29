// csv_io.cpp — CSV scene file I/O implementation.
//
// Supports loading and saving scenes with balls and walls in a simple
// CSV format. Each row is either a "ball" or "wall" type, identified
// by the first column. This keeps the file format simple and human-
// readable while supporting mixed entity types.

#include "csv_io.h"
#include "sim_config.h"  // WINDOW_WIDTH, WINDOW_HEIGHT for metadata
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <cstdio>

// ── Helper: trim whitespace from both ends of a string ─────────────
static std::string trim(const std::string& s) {
    size_t start = 0;
    while (start < s.size() && std::isspace(static_cast<unsigned char>(s[start]))) ++start;
    size_t end = s.size();
    while (end > start && std::isspace(static_cast<unsigned char>(s[end - 1]))) --end;
    return s.substr(start, end - start);
}

// ── Split a CSV line into tokens ───────────────────────────────────
std::vector<std::string> splitCSVLine(const std::string& line) {
    std::vector<std::string> tokens;
    std::stringstream ss(line);
    std::string token;
    while (std::getline(ss, token, ',')) {
        tokens.push_back(trim(token));
    }
    return tokens;
}

// ── Load scene from CSV ────────────────────────────────────────────
bool loadSceneFromCSV(const std::string& filename, PhysicsWorld& world) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        fprintf(stderr, "CSV load error: cannot open '%s'\n", filename.c_str());
        return false;
    }

    // Clear existing scene data before loading
    world.balls.clear();
    world.walls.clear();

    std::string line;
    int lineNum = 0;
    bool headerSkipped = false;

    while (std::getline(file, line)) {
        ++lineNum;

        // Skip empty lines and comments
        std::string trimmed = trim(line);
        if (trimmed.empty() || trimmed[0] == '#') continue;

        // Skip header row (first non-comment line)
        if (!headerSkipped) {
            headerSkipped = true;
            // Check if this looks like a header (starts with "type" or non-numeric)
            auto tokens = splitCSVLine(trimmed);
            if (!tokens.empty() && (tokens[0] == "type" || tokens[0] == "Type")) {
                continue; // It's a header, skip it
            }
            // If it doesn't look like a header, fall through and parse it
        }

        auto tokens = splitCSVLine(trimmed);
        if (tokens.empty()) continue;

        std::string type = tokens[0];
        // Normalize type to lowercase
        std::transform(type.begin(), type.end(), type.begin(),
                       [](unsigned char c) { return std::tolower(c); });

        if (type == "ball") {
            // Expected: ball,x,y,radius,r,g,b (7 columns)
            if (tokens.size() < 4) {
                fprintf(stderr, "CSV line %d: ball row needs at least 4 columns (type,x,y,radius)\n", lineNum);
                continue;
            }

            float x = std::stof(tokens[1]);
            float y = std::stof(tokens[2]);
            float radius = std::stof(tokens[3]);

            Ball ball(Vec2(x, y), radius);

            // Optional color columns (r,g,b)
            if (tokens.size() >= 7) {
                int r = std::stoi(tokens[4]);
                int g = std::stoi(tokens[5]);
                int b = std::stoi(tokens[6]);
                ball.color.r = static_cast<uint8_t>(std::max(0, std::min(255, r)));
                ball.color.g = static_cast<uint8_t>(std::max(0, std::min(255, g)));
                ball.color.b = static_cast<uint8_t>(std::max(0, std::min(255, b)));
                ball.color.hasColor = true;
            }

            world.balls.push_back(ball);

        } else if (type == "wall") {
            // Expected: wall,x1,y1,x2,y2 (5 columns)
            if (tokens.size() < 5) {
                fprintf(stderr, "CSV line %d: wall row needs 5 columns (type,x1,y1,x2,y2)\n", lineNum);
                continue;
            }

            float x1 = std::stof(tokens[1]);
            float y1 = std::stof(tokens[2]);
            float x2 = std::stof(tokens[3]);
            float y2 = std::stof(tokens[4]);

            world.walls.push_back(Wall(Vec2(x1, y1), Vec2(x2, y2)));

        } else {
            fprintf(stderr, "CSV line %d: unknown type '%s' (expected 'ball' or 'wall')\n",
                    lineNum, tokens[0].c_str());
        }
    }

    printf("CSV loaded: %zu balls, %zu walls from '%s'\n",
           world.balls.size(), world.walls.size(), filename.c_str());
    return true;
}

// ── Save scene to CSV ──────────────────────────────────────────────
bool saveSceneToCSV(const std::string& filename, const PhysicsWorld& world) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        fprintf(stderr, "CSV save error: cannot open '%s'\n", filename.c_str());
        return false;
    }

    // Write header with comment explaining both formats and metadata
    file << "# Physics simulator scene file\n";
    file << "# Window: " << WINDOW_WIDTH << "x" << WINDOW_HEIGHT << "\n";
    file << "# Ball format: ball,x,y,radius,r,g,b\n";
    file << "# Wall format: wall,x1,y1,x2,y2\n";
    file << "type,param1,param2,param3,param4,param5,param6\n";

    // Write walls first (they define the container)
    for (const auto& wall : world.walls) {
        file << "wall," << wall.p1.x << "," << wall.p1.y << ","
             << wall.p2.x << "," << wall.p2.y << "\n";
    }

    // Write balls with their current positions and colors.
    // Balls with hasColor=true get full 7-column rows (ball,x,y,radius,r,g,b).
    // Balls with hasColor=false get 4-column rows (ball,x,y,radius) so they
    // preserve speed-based coloring after a CSV roundtrip.
    for (const auto& ball : world.balls) {
        file << "ball," << ball.pos.x << "," << ball.pos.y << ","
             << ball.radius;
        if (ball.color.hasColor) {
            file << "," << static_cast<int>(ball.color.r)
                 << "," << static_cast<int>(ball.color.g)
                 << "," << static_cast<int>(ball.color.b);
        }
        file << "\n";
    }

    printf("CSV saved: %zu balls, %zu walls to '%s'\n",
           world.balls.size(), world.walls.size(), filename.c_str());
    return true;
}
