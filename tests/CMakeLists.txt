cmake_minimum_required(VERSION 3.14)
project(DijkstraMapTests VERSION 1.0.0 LANGUAGES CXX)

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

# Include FetchContent for downloading Google Test
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

# Header-only library
add_library(DijkstraMapLib INTERFACE)
target_include_directories(DijkstraMapLib INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
)
target_compile_features(DijkstraMapLib INTERFACE cxx_std_17)

# Enable testing
enable_testing()

# Test executable
add_executable(tests
    test_dijkstra_map.cpp
    test_api.cpp
    test_distance_types.cpp
)

target_link_libraries(tests
    PRIVATE
        DijkstraMapLib
        GTest::gtest
        GTest::gtest_main
)

# Discover tests
include(GoogleTest)
gtest_discover_tests(tests)
