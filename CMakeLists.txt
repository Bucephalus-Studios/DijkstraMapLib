cmake_minimum_required(VERSION 3.14)
project(DijkstraMapLib VERSION 1.0.0 LANGUAGES CXX)

# Set C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler warnings
if(MSVC)
    add_compile_options(/W4 /WX)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Include FetchContent for downloading Google Test and Benchmark
include(FetchContent)

# Download Google Test
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Download Google Benchmark
FetchContent_Declare(
    googlebenchmark
    GIT_REPOSITORY https://github.com/google/benchmark.git
    GIT_TAG v1.8.3
)
set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googlebenchmark)

# Header-only library
add_library(DijkstraMapLib INTERFACE)
target_include_directories(DijkstraMapLib INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(DijkstraMapLib INTERFACE cxx_std_17)

# Enable testing
enable_testing()

# Add tests subdirectory
add_subdirectory(tests)

# Add benchmarks subdirectory
add_subdirectory(benchmarks)

# Installation rules
install(TARGETS DijkstraMapLib
    EXPORT DijkstraMapLibTargets
)

install(FILES
    DijkstraMapLib.hpp
    DESTINATION include
)

install(DIRECTORY classes/
    DESTINATION include/classes
    FILES_MATCHING PATTERN "*.hpp"
)
