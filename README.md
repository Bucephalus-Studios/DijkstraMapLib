# DijkstraMapLib

A modern, header-only C++17 library for Dijkstra map generation and pathfinding. Dijkstra maps (also known as distance maps or goal maps) are used in games and AI for efficient pathfinding, field-of-influence calculations, and spatial reasoning.

## Features

- **Header-only library** - Easy to integrate, just include and use
- **Multiple distance metrics** - Manhattan, Chebyshev, and Euclidean
- **Flexible walkability** - Custom walkability functions via templates
- **Multiple goals** - Support for single or multiple goal positions
- **Modern C++17** - Uses structured bindings, const correctness, and clean code
- **Well-tested** - 48 comprehensive unit tests with Google Test
- **Performance benchmarks** - Google Benchmark suite included

## Quick Start

### Building and Testing

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j
ctest
```

### Running Benchmarks

```bash
./benchmarks/dijkstra_benchmarks
```

### Basic Usage Example

```cpp
#include "DijkstraMapLib.hpp"
using namespace DijkstraMapLib;

// Create a 20x20 map with Manhattan distance
DijkstraMap map(20, 20, DistanceType::Manhattan);

// Define walkable tiles (all tiles walkable in this example)
auto isWalkable = [](int x, int y) {
    return true;
};

// Set goal position
CoordList goals = {{10, 10}};

// Generate the Dijkstra map
generateDijkstraMap(map, goals, isWalkable);

// Query distances
int distance = map.getDistance(15, 15);  // Distance from goal to (15,15)
bool reachable = map.isReachable(15, 15);  // Can we reach this tile?
```

## Distance Metrics

### Manhattan Distance
- **Movement:** 4-directional (up, down, left, right)
- **Formula:** `|dx| + |dy|`
- **Use case:** Grid-based games with cardinal movement only

### Chebyshev Distance
- **Movement:** 8-directional (includes diagonals)
- **Formula:** `max(|dx|, |dy|)`
- **Use case:** Games allowing diagonal movement (chess king movement)

### Euclidean Distance
- **Movement:** 4-directional
- **Formula:** `sqrt(dx² + dy²)` (rounded to integer)
- **Use case:** More accurate distance calculations on grids

## API Reference

### DijkstraMap Class

```cpp
// Constructor
DijkstraMap(int width, int height, DistanceType distType = DistanceType::Euclidean);

// Get/Set distances
int getDistance(int x, int y) const;
void setDistance(int x, int y, int distance);

// Utility methods
bool isWithinBounds(int x, int y) const;
bool isReachable(int x, int y) const;
std::tuple<int, int> getDimensions() const;
void clear();

// Distance type
DistanceType getDistanceType() const;
void setDistanceType(DistanceType distType);
int calculateDistance(int x1, int y1, int x2, int y2) const;

// Constants
static constexpr int UNREACHABLE;
```

### API Functions

```cpp
// Generate Dijkstra map from multiple goals
template<typename WalkableFunc>
void generateDijkstraMap(DijkstraMap& dijkstraMap,
                        const CoordList& goals,
                        WalkableFunc isWalkable);

// Convenience function for single goal
template<typename WalkableFunc>
void generateDijkstraMapFromSingleGoal(DijkstraMap& dijkstraMap,
                                      int goalX, int goalY,
                                      WalkableFunc isWalkable);

// Find unreachable tiles
template<typename WalkableFunc>
CoordList findUnreachableTiles(const DijkstraMap& dijkstraMap,
                              WalkableFunc isWalkable);
```

## Advanced Examples

### Multiple Goals

```cpp
DijkstraMap map(50, 50, DistanceType::Manhattan);
CoordList goals = {{5, 5}, {45, 45}, {25, 25}};  // Multiple goals

generateDijkstraMap(map, goals, [](int x, int y) { return true; });
// Each tile will have distance to nearest goal
```

### Custom Walkability with Walls

```cpp
DijkstraMap map(50, 50, DistanceType::Manhattan);

// Define walls
auto isWalkable = [](int x, int y) {
    // Vertical wall at x=25
    if (x == 25 && y < 40) return false;
    return true;
};

CoordList goals = {{0, 0}};
generateDijkstraMap(map, goals, isWalkable);

// Find tiles that can't be reached from goals
auto unreachable = findUnreachableTiles(map, isWalkable);
```

### Using Chebyshev for Diagonal Movement

```cpp
DijkstraMap map(30, 30, DistanceType::Chebyshev);
CoordList goals = {{15, 15}};

generateDijkstraMap(map, goals, [](int x, int y) { return true; });

// Diagonal movement is now allowed
// Distance to (20, 20) will be 5 (not 10 as with Manhattan)
```

## Refactoring Improvements

This library has been significantly refactored for better quality:

### Code Quality Improvements

1. **Reduced Nesting** - Maximum 2 levels of nesting in all functions
2. **Structured Bindings** - Replaced all `std::get<N>()` calls with modern C++17 syntax
3. **DRY Principle** - Extracted helper functions to eliminate code duplication
4. **Better Variable Names** - Clear, descriptive naming throughout
5. **Const Correctness** - Proper use of const for immutable data

### Performance Optimizations

1. **Static Direction Arrays** - Direction vectors cached as static data
2. **Efficient Clear** - Uses `std::fill` instead of nested loops
3. **Early Returns** - Reduces unnecessary computation
4. **Move Semantics** - Eliminates unnecessary copies

### Code Organization

1. **Helper Functions** - Logic extracted into focused helper functions:
   - `getDirections()` - Cached direction lookup
   - `initializeGoals()` - Goal initialization logic
   - `processNeighbor()` - Neighbor processing logic

2. **Type Aliases** - Improved readability with clear type names:
   - `Coord` - Coordinate tuple
   - `CoordList` - Vector of coordinates
   - `QueueEntry` - Priority queue entry

## Testing

The library includes 48 comprehensive tests covering:

- Constructor and initialization
- Bounds checking
- Distance calculations
- All three distance metrics
- Multiple goals
- Wall interactions
- Edge cases (1x1 maps, large maps)
- API functionality
- Pathfinding correctness

Run tests with:
```bash
cd build
ctest --output-on-failure
```

## Benchmarks

Performance benchmarks included for:

- Various map sizes (10x10 to 200x200)
- Single vs multiple goals
- Different distance metrics
- Complex maze scenarios
- Rectangular maps

Sample results on test system:
- 10x10 map: ~2.7μs (38.8M items/sec)
- 50x50 map: ~154μs (17.5M items/sec)
- 100x100 map: ~777μs (13.5M items/sec)

Run benchmarks with:
```bash
cd build
./benchmarks/dijkstra_benchmarks
```

## Requirements

- C++17 compatible compiler
- CMake 3.14+
- Internet connection (for fetching Google Test and Benchmark during build)

## License

This is an open-source project. Feel free to use and modify as needed.

## Contributing

Contributions are welcome! Please ensure:
- All tests pass
- Code follows the existing style
- New features include tests
- Documentation is updated

## References

- [Dijkstra Maps Explained](http://www.roguebasin.com/index.php?title=The_Incredible_Power_of_Dijkstra_Maps)
- Used in games for AI, pathfinding, and influence maps
