cmake_minimum_required(VERSION 3.20)
project(PhysicsSim VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find SDL3
find_package(SDL3 REQUIRED)

# ── Physics library (no SDL dependency) ──────────────────────────────
add_library(physics_lib STATIC
    src/physics.cpp
)
target_include_directories(physics_lib PUBLIC include)

# ── CSV I/O library (no SDL dependency) ──────────────────────────────
add_library(csv_io_lib STATIC
    src/csv_io.cpp
)
target_include_directories(csv_io_lib PUBLIC include)
target_link_libraries(csv_io_lib PUBLIC physics_lib)

# ── Main simulator executable ────────────────────────────────────────
add_executable(simulator
    src/main.cpp
    src/renderer.cpp
)
target_include_directories(simulator PRIVATE include)
target_link_libraries(simulator PRIVATE physics_lib csv_io_lib SDL3::SDL3)

# ── Color assignment tool ────────────────────────────────────────────
# Takes an initial scene CSV + an image (BMP/PNG/JPG/TGA), runs the
# simulation, and assigns each ball a color based on where it ends up
# in the image. Uses stb_image for multi-format loading (no SDL needed).
add_executable(color_assign
    src/color_assign.cpp
)
target_include_directories(color_assign PRIVATE include)
target_link_libraries(color_assign PRIVATE physics_lib csv_io_lib)
# stb_image needs math library on some platforms
target_link_libraries(color_assign PRIVATE m)

# ── Scene generator tool ────────────────────────────────────────────
# Generates procedural scene CSV files with various ball/wall layouts.
# No SDL dependency — pure C++ file I/O.
add_executable(scene_gen
    src/scene_gen.cpp
)
target_include_directories(scene_gen PRIVATE include)
target_link_libraries(scene_gen PRIVATE physics_lib)

# ── Unit tests (no SDL needed for physics; CSV tests need csv_io_lib) ─
add_executable(tests
    tests/test_physics.cpp
)
target_include_directories(tests PRIVATE include)
target_link_libraries(tests PRIVATE physics_lib csv_io_lib)
