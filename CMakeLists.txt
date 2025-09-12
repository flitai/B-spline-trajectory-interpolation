cmake_minimum_required(VERSION 3.10)
project(BsplineTest)

# --- Compiler Settings ---
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Find Eigen Library ---
# Eigen is a header-only library, so we just need to find its include directory.
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# --- Source Files ---
# Add your b-spline implementation files here
add_library(bspline_lib non_uniform_bspline.cpp)
target_include_directories(bspline_lib PUBLIC ${EIGEN3_INCLUDE_DIR})

# --- Executable ---
add_executable(bspline_test main.cpp)

# Link the executable against your b-spline library
target_link_libraries(bspline_test bspline_lib)

# Add include directory for the executable itself to find the library header
target_include_directories(bspline_test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR})

# --- Output Message ---
message(STATUS "Eigen3 include dir: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "To build, run: mkdir build && cd build && cmake .. && make")
message(STATUS "To run, execute: ./bspline_test")
