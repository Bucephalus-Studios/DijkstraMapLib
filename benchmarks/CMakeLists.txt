# Benchmark executable
add_executable(dijkstra_benchmarks
    benchmark_main.cpp
)

target_link_libraries(dijkstra_benchmarks
    PRIVATE
        DijkstraMapLib
        benchmark::benchmark
)
