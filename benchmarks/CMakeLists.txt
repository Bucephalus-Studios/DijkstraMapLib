cmake_minimum_required(VERSION 3.14)
project(DijkstraMapBenchmarks VERSION 1.0.0 LANGUAGES CXX)

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

# Include FetchContent for downloading Google Benchmark
include(FetchContent)

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
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
)
target_compile_features(DijkstraMapLib INTERFACE cxx_std_17)

# Benchmark executable
add_executable(benchmarks
    benchmark_main.cpp
)

target_link_libraries(benchmarks
    PRIVATE
        DijkstraMapLib
        benchmark::benchmark
)
