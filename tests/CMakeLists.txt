# Test executable
add_executable(dijkstra_tests
    test_dijkstra_map.cpp
    test_api.cpp
    test_distance_types.cpp
)

target_link_libraries(dijkstra_tests
    PRIVATE
        DijkstraMapLib
        GTest::gtest
        GTest::gtest_main
)

# Discover tests
include(GoogleTest)
gtest_discover_tests(dijkstra_tests)
