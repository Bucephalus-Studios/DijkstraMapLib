#include <benchmark/benchmark.h>
#include "DijkstraMapLib.hpp"

using namespace DijkstraMapLib;

// Simple walkable function for benchmarks
static bool allWalkable(int, int) {
    return true;
}

// Benchmark: Single goal, small map (10x10)
static void BM_SingleGoalSmallMap(benchmark::State& state) {
    constexpr int size = 10;
    DijkstraMap map(size, size, DistanceType::Manhattan);
    CoordList goals = {{5, 5}};

    for (auto _ : state) {
        generateDijkstraMap(map, goals, allWalkable);
        benchmark::DoNotOptimize(map.getDistance(0, 0));
    }

    state.SetItemsProcessed(state.iterations() * size * size);
}
BENCHMARK(BM_SingleGoalSmallMap);

// Benchmark: Single goal, medium map (50x50)
static void BM_SingleGoalMediumMap(benchmark::State& state) {
    constexpr int size = 50;
    DijkstraMap map(size, size, DistanceType::Manhattan);
    CoordList goals = {{25, 25}};

    for (auto _ : state) {
        generateDijkstraMap(map, goals, allWalkable);
        benchmark::DoNotOptimize(map.getDistance(0, 0));
    }

    state.SetItemsProcessed(state.iterations() * size * size);
}
BENCHMARK(BM_SingleGoalMediumMap);

// Benchmark: Single goal, large map (100x100)
static void BM_SingleGoalLargeMap(benchmark::State& state) {
    constexpr int size = 100;
    DijkstraMap map(size, size, DistanceType::Manhattan);
    CoordList goals = {{50, 50}};

    for (auto _ : state) {
        generateDijkstraMap(map, goals, allWalkable);
        benchmark::DoNotOptimize(map.getDistance(0, 0));
    }

    state.SetItemsProcessed(state.iterations() * size * size);
}
BENCHMARK(BM_SingleGoalLargeMap);

// Benchmark: Single goal, very large map (200x200)
static void BM_SingleGoalVeryLargeMap(benchmark::State& state) {
    constexpr int size = 200;
    DijkstraMap map(size, size, DistanceType::Manhattan);
    CoordList goals = {{100, 100}};

    for (auto _ : state) {
        generateDijkstraMap(map, goals, allWalkable);
        benchmark::DoNotOptimize(map.getDistance(0, 0));
    }

    state.SetItemsProcessed(state.iterations() * size * size);
}
BENCHMARK(BM_SingleGoalVeryLargeMap);

// Benchmark: Multiple goals (4 corners), medium map
static void BM_MultipleGoalsMediumMap(benchmark::State& state) {
    constexpr int size = 50;
    DijkstraMap map(size, size, DistanceType::Manhattan);
    CoordList goals = {{0, 0}, {0, size - 1}, {size - 1, 0}, {size - 1, size - 1}};

    for (auto _ : state) {
        generateDijkstraMap(map, goals, allWalkable);
        benchmark::DoNotOptimize(map.getDistance(25, 25));
    }

    state.SetItemsProcessed(state.iterations() * size * size);
}
BENCHMARK(BM_MultipleGoalsMediumMap);

// Benchmark: Many goals (10 random positions), medium map
static void BM_ManyGoalsMediumMap(benchmark::State& state) {
    constexpr int size = 50;
    DijkstraMap map(size, size, DistanceType::Manhattan);
    CoordList goals = {
        {5, 5}, {15, 15}, {25, 25}, {35, 35}, {45, 45},
        {5, 45}, {15, 35}, {25, 25}, {35, 15}, {45, 5}
    };

    for (auto _ : state) {
        generateDijkstraMap(map, goals, allWalkable);
        benchmark::DoNotOptimize(map.getDistance(0, 0));
    }

    state.SetItemsProcessed(state.iterations() * size * size);
}
BENCHMARK(BM_ManyGoalsMediumMap);

// Benchmark: Manhattan distance
static void BM_ManhattanDistance(benchmark::State& state) {
    constexpr int size = 100;
    DijkstraMap map(size, size, DistanceType::Manhattan);
    CoordList goals = {{50, 50}};

    for (auto _ : state) {
        generateDijkstraMap(map, goals, allWalkable);
        benchmark::DoNotOptimize(map.getDistance(0, 0));
    }

    state.SetItemsProcessed(state.iterations() * size * size);
}
BENCHMARK(BM_ManhattanDistance);

// Benchmark: Chebyshev distance (8-directional)
static void BM_ChebyshevDistance(benchmark::State& state) {
    constexpr int size = 100;
    DijkstraMap map(size, size, DistanceType::Chebyshev);
    CoordList goals = {{50, 50}};

    for (auto _ : state) {
        generateDijkstraMap(map, goals, allWalkable);
        benchmark::DoNotOptimize(map.getDistance(0, 0));
    }

    state.SetItemsProcessed(state.iterations() * size * size);
}
BENCHMARK(BM_ChebyshevDistance);

// Benchmark: Euclidean distance
static void BM_EuclideanDistance(benchmark::State& state) {
    constexpr int size = 100;
    DijkstraMap map(size, size, DistanceType::Euclidean);
    CoordList goals = {{50, 50}};

    for (auto _ : state) {
        generateDijkstraMap(map, goals, allWalkable);
        benchmark::DoNotOptimize(map.getDistance(0, 0));
    }

    state.SetItemsProcessed(state.iterations() * size * size);
}
BENCHMARK(BM_EuclideanDistance);

// Benchmark: Map clearing
static void BM_MapClear(benchmark::State& state) {
    constexpr int size = 100;
    DijkstraMap map(size, size, DistanceType::Manhattan);

    for (auto _ : state) {
        map.clear();
        benchmark::DoNotOptimize(map.getDistance(0, 0));
    }

    state.SetItemsProcessed(state.iterations() * size * size);
}
BENCHMARK(BM_MapClear);

// Benchmark: Finding unreachable tiles with obstacles
static void BM_FindUnreachableTiles(benchmark::State& state) {
    constexpr int size = 100;
    DijkstraMap map(size, size, DistanceType::Manhattan);

    // Walkable with a vertical wall in the middle
    auto walkableWithWall = [](int x, int) {
        return x != 50;
    };

    CoordList goals = {{25, 50}};
    generateDijkstraMap(map, goals, walkableWithWall);

    for (auto _ : state) {
        auto unreachable = findUnreachableTiles(map, walkableWithWall);
        benchmark::DoNotOptimize(unreachable.size());
    }

    state.SetItemsProcessed(state.iterations() * size * size);
}
BENCHMARK(BM_FindUnreachableTiles);

// Benchmark: Pathfinding with complex maze (checkerboard obstacles)
static void BM_ComplexMaze(benchmark::State& state) {
    constexpr int size = 50;
    DijkstraMap map(size, size, DistanceType::Manhattan);

    // Checkerboard pattern - every other tile is walkable
    auto checkerboardWalkable = [](int x, int y) {
        return (x + y) % 2 == 0;
    };

    CoordList goals = {{0, 0}};

    for (auto _ : state) {
        generateDijkstraMap(map, goals, checkerboardWalkable);
        benchmark::DoNotOptimize(map.getDistance(size - 2, size - 2));
    }

    state.SetItemsProcessed(state.iterations() * size * size);
}
BENCHMARK(BM_ComplexMaze);

// Benchmark: Rectangular map (wide)
static void BM_RectangularMapWide(benchmark::State& state) {
    constexpr int width = 200;
    constexpr int height = 50;
    DijkstraMap map(width, height, DistanceType::Manhattan);
    CoordList goals = {{100, 25}};

    for (auto _ : state) {
        generateDijkstraMap(map, goals, allWalkable);
        benchmark::DoNotOptimize(map.getDistance(0, 0));
    }

    state.SetItemsProcessed(state.iterations() * width * height);
}
BENCHMARK(BM_RectangularMapWide);

// Benchmark: Rectangular map (tall)
static void BM_RectangularMapTall(benchmark::State& state) {
    constexpr int width = 50;
    constexpr int height = 200;
    DijkstraMap map(width, height, DistanceType::Manhattan);
    CoordList goals = {{25, 100}};

    for (auto _ : state) {
        generateDijkstraMap(map, goals, allWalkable);
        benchmark::DoNotOptimize(map.getDistance(0, 0));
    }

    state.SetItemsProcessed(state.iterations() * width * height);
}
BENCHMARK(BM_RectangularMapTall);

// Benchmark: Single goal at corner vs center
static void BM_CornerGoal(benchmark::State& state) {
    constexpr int size = 100;
    DijkstraMap map(size, size, DistanceType::Manhattan);
    CoordList goals = {{0, 0}};

    for (auto _ : state) {
        generateDijkstraMap(map, goals, allWalkable);
        benchmark::DoNotOptimize(map.getDistance(size - 1, size - 1));
    }

    state.SetItemsProcessed(state.iterations() * size * size);
}
BENCHMARK(BM_CornerGoal);

// Benchmark: Distance calculation only (no map generation)
static void BM_DistanceCalculationManhattan(benchmark::State& state) {
    DijkstraMap map(10, 10, DistanceType::Manhattan);

    for (auto _ : state) {
        int dist = map.calculateDistance(0, 0, 9, 9);
        benchmark::DoNotOptimize(dist);
    }
}
BENCHMARK(BM_DistanceCalculationManhattan);

static void BM_DistanceCalculationChebyshev(benchmark::State& state) {
    DijkstraMap map(10, 10, DistanceType::Chebyshev);

    for (auto _ : state) {
        int dist = map.calculateDistance(0, 0, 9, 9);
        benchmark::DoNotOptimize(dist);
    }
}
BENCHMARK(BM_DistanceCalculationChebyshev);

static void BM_DistanceCalculationEuclidean(benchmark::State& state) {
    DijkstraMap map(10, 10, DistanceType::Euclidean);

    for (auto _ : state) {
        int dist = map.calculateDistance(0, 0, 9, 9);
        benchmark::DoNotOptimize(dist);
    }
}
BENCHMARK(BM_DistanceCalculationEuclidean);

// Benchmark: Memory access patterns - getDistance calls
static void BM_GetDistanceAccess(benchmark::State& state) {
    constexpr int size = 100;
    DijkstraMap map(size, size, DistanceType::Manhattan);
    CoordList goals = {{50, 50}};
    generateDijkstraMap(map, goals, allWalkable);

    int sum = 0;
    for (auto _ : state) {
        for (int x = 0; x < size; ++x) {
            for (int y = 0; y < size; ++y) {
                sum += map.getDistance(x, y);
            }
        }
        benchmark::DoNotOptimize(sum);
    }

    state.SetItemsProcessed(state.iterations() * size * size);
}
BENCHMARK(BM_GetDistanceAccess);

BENCHMARK_MAIN();
