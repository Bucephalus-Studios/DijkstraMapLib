#include <gtest/gtest.h>
#include "DijkstraMapLib.hpp"

using namespace DijkstraMapLib;

// Test fixture for API tests
class DijkstraMapAPITest : public ::testing::Test {
protected:
    static constexpr int mapWidth = 10;
    static constexpr int mapHeight = 10;

    // Simple walkable function: all tiles are walkable
    static bool allWalkable(int, int) {
        return true;
    }

    // Walkable function with walls
    static bool walkableWithWalls(int x, int) {
        // Create a vertical wall at x=5
        return x != 5;
    }

    // Walkable function: only center tile is not walkable
    static bool centerBlocked(int x, int y) {
        return !(x == 5 && y == 5);
    }
};

// Test generateDijkstraMap with single goal
TEST_F(DijkstraMapAPITest, SingleGoalInCenter) {
    DijkstraMap map(mapWidth, mapHeight, DistanceType::Manhattan);
    CoordList goals = {{5, 5}};

    generateDijkstraMap(map, goals, allWalkable);

    // Goal should have distance 0
    EXPECT_EQ(map.getDistance(5, 5), 0);

    // Adjacent tiles should have distance 1
    EXPECT_EQ(map.getDistance(5, 6), 1);
    EXPECT_EQ(map.getDistance(5, 4), 1);
    EXPECT_EQ(map.getDistance(6, 5), 1);
    EXPECT_EQ(map.getDistance(4, 5), 1);

    // Corner should have distance 10 (Manhattan)
    EXPECT_EQ(map.getDistance(0, 0), 10);
    EXPECT_EQ(map.getDistance(9, 9), 8);
}

// Test generateDijkstraMap with multiple goals
TEST_F(DijkstraMapAPITest, MultipleGoals) {
    DijkstraMap map(mapWidth, mapHeight, DistanceType::Manhattan);
    CoordList goals = {{0, 0}, {9, 9}};

    generateDijkstraMap(map, goals, allWalkable);

    // Both goals should have distance 0
    EXPECT_EQ(map.getDistance(0, 0), 0);
    EXPECT_EQ(map.getDistance(9, 9), 0);

    // All tiles should be reachable
    for (int x = 0; x < mapWidth; ++x) {
        for (int y = 0; y < mapHeight; ++y) {
            EXPECT_TRUE(map.isReachable(x, y));
        }
    }
}

// Test with walls
TEST_F(DijkstraMapAPITest, PathfindingWithWalls) {
    DijkstraMap map(mapWidth, mapHeight, DistanceType::Manhattan);
    CoordList goals = {{0, 0}};

    generateDijkstraMap(map, goals, walkableWithWalls);

    // Left side should be reachable
    EXPECT_TRUE(map.isReachable(0, 0));
    EXPECT_TRUE(map.isReachable(4, 5));

    // Wall at x=5 should not be reachable (not walkable)
    EXPECT_FALSE(map.isReachable(5, 5));

    // Right side should be unreachable from (0,0) due to wall
    EXPECT_FALSE(map.isReachable(6, 5));
    EXPECT_FALSE(map.isReachable(9, 9));
}

// Test generateDijkstraMapFromSingleGoal convenience function
TEST_F(DijkstraMapAPITest, SingleGoalConvenienceFunction) {
    DijkstraMap map(mapWidth, mapHeight, DistanceType::Manhattan);

    generateDijkstraMapFromSingleGoal(map, 3, 3, allWalkable);

    EXPECT_EQ(map.getDistance(3, 3), 0);
    EXPECT_EQ(map.getDistance(3, 4), 1);
    EXPECT_EQ(map.getDistance(4, 3), 1);
}

// Test findUnreachableTiles
TEST_F(DijkstraMapAPITest, FindUnreachableTilesWithWalls) {
    DijkstraMap map(mapWidth, mapHeight, DistanceType::Manhattan);
    CoordList goals = {{0, 0}};

    generateDijkstraMap(map, goals, walkableWithWalls);

    auto unreachable = findUnreachableTiles(map, walkableWithWalls);

    // The right side (x >= 6) should be unreachable (5 columns * 10 rows = 50 tiles)
    // Wall at x=5 is not counted as it's not walkable
    EXPECT_EQ(unreachable.size(), 40); // 4 columns (6,7,8,9) * 10 rows
}

TEST_F(DijkstraMapAPITest, FindUnreachableTilesNoWalls) {
    DijkstraMap map(mapWidth, mapHeight, DistanceType::Manhattan);
    CoordList goals = {{5, 5}};

    generateDijkstraMap(map, goals, allWalkable);

    auto unreachable = findUnreachableTiles(map, allWalkable);

    // All tiles should be reachable
    EXPECT_EQ(unreachable.size(), 0);
}

TEST_F(DijkstraMapAPITest, FindUnreachableTilesNoGoals) {
    DijkstraMap map(mapWidth, mapHeight, DistanceType::Manhattan);
    CoordList goals = {};

    generateDijkstraMap(map, goals, allWalkable);

    auto unreachable = findUnreachableTiles(map, allWalkable);

    // All tiles should be unreachable
    EXPECT_EQ(unreachable.size(), mapWidth * mapHeight);
}

// Test with Chebyshev distance (8-directional movement)
TEST_F(DijkstraMapAPITest, ChebyshevDistanceAllowsDiagonals) {
    DijkstraMap map(mapWidth, mapHeight, DistanceType::Chebyshev);
    CoordList goals = {{0, 0}};

    generateDijkstraMap(map, goals, allWalkable);

    // With Chebyshev, diagonal movement is allowed
    // Distance to (1, 1) should be 1 (diagonal)
    EXPECT_EQ(map.getDistance(1, 1), 1);

    // Distance to (5, 5) should be 5 (max of x and y differences)
    EXPECT_EQ(map.getDistance(5, 5), 5);
}

// Test with Euclidean distance
TEST_F(DijkstraMapAPITest, EuclideanDistance) {
    DijkstraMap map(mapWidth, mapHeight, DistanceType::Euclidean);
    CoordList goals = {{0, 0}};

    generateDijkstraMap(map, goals, allWalkable);

    // With 4-directional movement and Euclidean costs:
    // To reach (3, 4), we need 3 horizontal + 4 vertical steps = 7 steps of cost 1 each
    EXPECT_EQ(map.getDistance(3, 4), 7);

    // Distance to (1, 0) should be 1 (one step)
    EXPECT_EQ(map.getDistance(1, 0), 1);
}

// Test map clearing
TEST_F(DijkstraMapAPITest, MapIsClearedBetweenGenerations) {
    DijkstraMap map(mapWidth, mapHeight, DistanceType::Manhattan);
    CoordList goals1 = {{0, 0}};
    CoordList goals2 = {{9, 9}};

    // First generation
    generateDijkstraMap(map, goals1, allWalkable);
    EXPECT_EQ(map.getDistance(0, 0), 0);
    EXPECT_TRUE(map.isReachable(9, 9));

    // Second generation should clear the map first
    generateDijkstraMap(map, goals2, allWalkable);
    EXPECT_EQ(map.getDistance(9, 9), 0);
    EXPECT_NE(map.getDistance(0, 0), 0);
}

// Test goal outside map bounds
TEST_F(DijkstraMapAPITest, GoalOutsideBoundsIsIgnored) {
    DijkstraMap map(mapWidth, mapHeight, DistanceType::Manhattan);
    CoordList goals = {{-1, -1}, {5, 5}, {100, 100}};

    generateDijkstraMap(map, goals, allWalkable);

    // Only the valid goal at (5, 5) should be set
    EXPECT_EQ(map.getDistance(5, 5), 0);

    // Invalid goals should not cause any reachable tiles at those positions
    EXPECT_EQ(map.getDistance(-1, -1), DijkstraMap::UNREACHABLE);
}

// Test goal on non-walkable tile
TEST_F(DijkstraMapAPITest, GoalOnNonWalkableTileIsIgnored) {
    DijkstraMap map(mapWidth, mapHeight, DistanceType::Manhattan);
    CoordList goals = {{5, 5}, {7, 7}};

    // (5, 5) is blocked by centerBlocked function
    generateDijkstraMap(map, goals, centerBlocked);

    // (5, 5) should not be set as a goal
    EXPECT_FALSE(map.isReachable(5, 5));

    // (7, 7) should be a valid goal
    EXPECT_EQ(map.getDistance(7, 7), 0);
}

// Test lambda as walkable function
TEST_F(DijkstraMapAPITest, LambdaWalkableFunction) {
    DijkstraMap map(mapWidth, mapHeight, DistanceType::Manhattan);
    CoordList goals = {{0, 0}};

    // Create a custom walkable function - only tiles where both x AND y are divisible by 3
    auto customWalkable = [](int x, int y) {
        return (x % 3 == 0) && (y % 3 == 0);
    };

    generateDijkstraMap(map, goals, customWalkable);

    // (0, 0) is walkable and should be goal
    EXPECT_EQ(map.getDistance(0, 0), 0);

    // (3, 0) and (0, 3) cannot be reached because there's no valid path
    // (would need to go through non-walkable tiles)
    EXPECT_FALSE(map.isReachable(3, 0));
    EXPECT_FALSE(map.isReachable(0, 3));

    // (1, 0) is not walkable
    EXPECT_FALSE(map.isReachable(1, 0));

    // (0, 1) is not walkable
    EXPECT_FALSE(map.isReachable(0, 1));
}

// Performance test: large map
TEST_F(DijkstraMapAPITest, LargeMapPerformance) {
    constexpr int largeSize = 100;
    DijkstraMap map(largeSize, largeSize, DistanceType::Manhattan);
    CoordList goals = {{0, 0}};

    // This should complete quickly
    generateDijkstraMap(map, goals, allWalkable);

    EXPECT_EQ(map.getDistance(0, 0), 0);
    EXPECT_TRUE(map.isReachable(99, 99));
}

// Test path flow: distances should increase monotonically along a path
TEST_F(DijkstraMapAPITest, PathFlowManhattan) {
    DijkstraMap map(mapWidth, mapHeight, DistanceType::Manhattan);
    CoordList goals = {{0, 0}};

    generateDijkstraMap(map, goals, allWalkable);

    // Moving away from goal should increase distance
    EXPECT_LT(map.getDistance(0, 0), map.getDistance(1, 0));
    EXPECT_LT(map.getDistance(1, 0), map.getDistance(2, 0));
    EXPECT_LT(map.getDistance(2, 0), map.getDistance(3, 0));
}

// Test that neighbors have correct relative distances
TEST_F(DijkstraMapAPITest, NeighborDistancesAreConsistent) {
    DijkstraMap map(mapWidth, mapHeight, DistanceType::Manhattan);
    CoordList goals = {{5, 5}};

    generateDijkstraMap(map, goals, allWalkable);

    int center = map.getDistance(5, 5);
    int north = map.getDistance(5, 6);
    int south = map.getDistance(5, 4);
    int east = map.getDistance(6, 5);
    int west = map.getDistance(4, 5);

    // All neighbors should be exactly 1 more than center (which is 0)
    EXPECT_EQ(north, center + 1);
    EXPECT_EQ(south, center + 1);
    EXPECT_EQ(east, center + 1);
    EXPECT_EQ(west, center + 1);
}
