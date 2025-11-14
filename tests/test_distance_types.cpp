#include <gtest/gtest.h>
#include "DijkstraMapLib.hpp"

using namespace DijkstraMapLib;

// Test fixture for distance type comparison tests
class DistanceTypeTest : public ::testing::Test {
protected:
    static constexpr int mapWidth = 20;
    static constexpr int mapHeight = 20;

    static bool allWalkable(int, int) {
        return true;
    }
};

// Test Manhattan distance properties
TEST_F(DistanceTypeTest, ManhattanDistanceProperties) {
    DijkstraMap map(mapWidth, mapHeight, DistanceType::Manhattan);
    CoordList goals = {{10, 10}};

    generateDijkstraMap(map, goals, allWalkable);

    // Manhattan distance: |dx| + |dy|
    // From (10, 10) to (13, 14): |3| + |4| = 7
    EXPECT_EQ(map.getDistance(13, 14), 7);

    // From (10, 10) to (7, 6): |3| + |4| = 7
    EXPECT_EQ(map.getDistance(7, 6), 7);

    // Diagonal tiles are NOT cheaper with Manhattan
    EXPECT_EQ(map.getDistance(11, 11), 2); // |1| + |1| = 2
}

// Test Chebyshev distance properties
TEST_F(DistanceTypeTest, ChebyshevDistanceProperties) {
    DijkstraMap map(mapWidth, mapHeight, DistanceType::Chebyshev);
    CoordList goals = {{10, 10}};

    generateDijkstraMap(map, goals, allWalkable);

    // Chebyshev distance: max(|dx|, |dy|)
    // From (10, 10) to (13, 14): max(3, 4) = 4
    EXPECT_EQ(map.getDistance(13, 14), 4);

    // From (10, 10) to (7, 6): max(3, 4) = 4
    EXPECT_EQ(map.getDistance(7, 6), 4);

    // Diagonal tiles ARE cheaper with Chebyshev
    EXPECT_EQ(map.getDistance(11, 11), 1); // max(1, 1) = 1
}

// Test Euclidean distance properties
TEST_F(DistanceTypeTest, EuclideanDistanceProperties) {
    DijkstraMap map(mapWidth, mapHeight, DistanceType::Euclidean);
    CoordList goals = {{10, 10}};

    generateDijkstraMap(map, goals, allWalkable);

    // With 4-directional movement, Euclidean distance is the sum of step costs
    // From (10, 10) to (13, 14): 3 steps right + 4 steps down = 7 steps
    EXPECT_EQ(map.getDistance(13, 14), 7);

    // From (10, 10) to (11, 10): 1 step = cost 1
    EXPECT_EQ(map.getDistance(11, 10), 1);

    // Can't move diagonally with Euclidean + 4-directional movement
    // From (10, 10) to (11, 11): 1 right + 1 down = 2 steps
    EXPECT_EQ(map.getDistance(11, 11), 2);
}

// Compare all three distance types from same starting point
TEST_F(DistanceTypeTest, CompareAllDistanceTypesAtPoint) {
    const int startX = 10, startY = 10;
    const int targetX = 15, targetY = 14;

    DijkstraMap manhattanMap(mapWidth, mapHeight, DistanceType::Manhattan);
    DijkstraMap chebyshevMap(mapWidth, mapHeight, DistanceType::Chebyshev);
    DijkstraMap euclideanMap(mapWidth, mapHeight, DistanceType::Euclidean);

    CoordList goals = {{startX, startY}};

    generateDijkstraMap(manhattanMap, goals, allWalkable);
    generateDijkstraMap(chebyshevMap, goals, allWalkable);
    generateDijkstraMap(euclideanMap, goals, allWalkable);

    int manhattan = manhattanMap.getDistance(targetX, targetY);
    int chebyshev = chebyshevMap.getDistance(targetX, targetY);
    int euclidean = euclideanMap.getDistance(targetX, targetY);

    // Manhattan: |5| + |4| = 9 steps
    EXPECT_EQ(manhattan, 9);

    // Chebyshev: max(5, 4) = 5 (can move diagonally)
    EXPECT_EQ(chebyshev, 5);

    // Euclidean: 4-directional, so same path as Manhattan = 9 steps
    EXPECT_EQ(euclidean, 9);

    // Chebyshev allows diagonal movement, so it's the shortest
    EXPECT_LE(chebyshev, euclidean);
    EXPECT_LE(chebyshev, manhattan);
}

// Test 4-directional vs 8-directional movement
TEST_F(DistanceTypeTest, MovementDirectionsDifferByType) {
    const int startX = 10, startY = 10;

    // Manhattan and Euclidean use 4 directions
    DijkstraMap manhattanMap(mapWidth, mapHeight, DistanceType::Manhattan);
    CoordList goals = {{startX, startY}};
    generateDijkstraMap(manhattanMap, goals, allWalkable);

    // Chebyshev uses 8 directions
    DijkstraMap chebyshevMap(mapWidth, mapHeight, DistanceType::Chebyshev);
    generateDijkstraMap(chebyshevMap, goals, allWalkable);

    // For purely diagonal movement, Chebyshev is much more efficient
    // From (10, 10) to (15, 15): 5 steps diagonally vs 10 steps with 4-dir
    EXPECT_EQ(chebyshevMap.getDistance(15, 15), 5);  // 8-directional
    EXPECT_EQ(manhattanMap.getDistance(15, 15), 10); // 4-directional
}

// Test distance type changes after construction
TEST_F(DistanceTypeTest, ChangingDistanceTypeAffectsGeneration) {
    DijkstraMap map(mapWidth, mapHeight, DistanceType::Manhattan);
    CoordList goals = {{10, 10}};

    // Generate with Manhattan
    generateDijkstraMap(map, goals, allWalkable);
    int manhattanDist = map.getDistance(15, 15);
    EXPECT_EQ(manhattanDist, 10);

    // Change to Chebyshev and regenerate
    map.setDistanceType(DistanceType::Chebyshev);
    generateDijkstraMap(map, goals, allWalkable);
    int chebyshevDist = map.getDistance(15, 15);
    EXPECT_EQ(chebyshevDist, 5);

    EXPECT_NE(manhattanDist, chebyshevDist);
}

// Test Pythagorean triples with Euclidean distance calculations
TEST_F(DistanceTypeTest, EuclideanPythagoreanTriples) {
    DijkstraMap map(mapWidth, mapHeight, DistanceType::Euclidean);

    // Test the distance calculation function directly (not pathfinding)
    // 3-4-5 triangle: sqrt(9 + 16) = 5
    EXPECT_EQ(map.calculateDistance(0, 0, 3, 4), 5);
    EXPECT_EQ(map.calculateDistance(0, 0, 4, 3), 5);

    // 5-12-13 triangle: sqrt(25 + 144) = 13
    EXPECT_EQ(map.calculateDistance(0, 0, 5, 12), 13);
    EXPECT_EQ(map.calculateDistance(0, 0, 12, 5), 13);

    // Note: actual pathfinding uses 4-directional movement,
    // so path distances will be different from straight-line distances
}

// Test symmetry: distance should be same in all symmetric positions
TEST_F(DistanceTypeTest, ManhattanSymmetry) {
    DijkstraMap map(mapWidth, mapHeight, DistanceType::Manhattan);
    CoordList goals = {{10, 10}};

    generateDijkstraMap(map, goals, allWalkable);

    // All positions at Manhattan distance 5 should have same value
    EXPECT_EQ(map.getDistance(15, 10), 5); // 5 east
    EXPECT_EQ(map.getDistance(5, 10), 5);  // 5 west
    EXPECT_EQ(map.getDistance(10, 15), 5); // 5 south
    EXPECT_EQ(map.getDistance(10, 5), 5);  // 5 north
    EXPECT_EQ(map.getDistance(13, 12), 5); // 3 + 2
    EXPECT_EQ(map.getDistance(12, 13), 5); // 2 + 3
}

TEST_F(DistanceTypeTest, ChebyshevSymmetry) {
    DijkstraMap map(mapWidth, mapHeight, DistanceType::Chebyshev);
    CoordList goals = {{10, 10}};

    generateDijkstraMap(map, goals, allWalkable);

    // All corners at same Chebyshev distance
    EXPECT_EQ(map.getDistance(15, 15), 5);
    EXPECT_EQ(map.getDistance(5, 5), 5);
    EXPECT_EQ(map.getDistance(15, 5), 5);
    EXPECT_EQ(map.getDistance(5, 15), 5);
}

// Test wall interactions with different distance types
TEST_F(DistanceTypeTest, WallInteractionChebyshev) {
    DijkstraMap map(11, 11, DistanceType::Chebyshev);

    // Walkable function: wall at x=5
    auto walkableWithWall = [](int x, int) {
        return x != 5;
    };

    CoordList goals = {{0, 5}};
    generateDijkstraMap(map, goals, walkableWithWall);

    // With Chebyshev (8-dir), we can go around the wall diagonally
    // This might still reach the other side, depending on map size
    // The wall blocks horizontal movement but diagonal might go around

    // Left side should be reachable
    EXPECT_TRUE(map.isReachable(0, 5));
    EXPECT_TRUE(map.isReachable(4, 5));

    // Wall should not be reachable
    EXPECT_FALSE(map.isReachable(5, 5));
}

// Test edge case: distance 0 to goal
TEST_F(DistanceTypeTest, GoalHasZeroDistance) {
    for (auto distType : {DistanceType::Manhattan, DistanceType::Chebyshev, DistanceType::Euclidean}) {
        DijkstraMap map(mapWidth, mapHeight, distType);
        CoordList goals = {{5, 5}};

        generateDijkstraMap(map, goals, allWalkable);

        EXPECT_EQ(map.getDistance(5, 5), 0);
    }
}

// Test multiple goals with different distance types
TEST_F(DistanceTypeTest, MultipleGoalsManhattan) {
    DijkstraMap map(mapWidth, mapHeight, DistanceType::Manhattan);
    CoordList goals = {{5, 5}, {15, 15}};

    generateDijkstraMap(map, goals, allWalkable);

    // (6, 6) is distance 2 from (5,5): |1| + |1| = 2
    int dist66 = map.getDistance(6, 6);
    EXPECT_EQ(dist66, 2);

    // (14, 14) is distance 2 from (15,15): |1| + |1| = 2
    int dist1414 = map.getDistance(14, 14);
    EXPECT_EQ(dist1414, 2);

    // Both are equidistant from their nearest goal
    EXPECT_EQ(dist66, dist1414);

    // Point equidistant from both goals should choose minimum
    int midDist = map.getDistance(10, 10);
    // Distance to (5,5): |5| + |5| = 10
    // Distance to (15,15): |5| + |5| = 10
    EXPECT_EQ(midDist, 10);
}

// Test distance calculation correctness for each type
TEST_F(DistanceTypeTest, DirectDistanceCalculation) {
    // Test the calculateDistance method directly
    DijkstraMap manhattanMap(10, 10, DistanceType::Manhattan);
    EXPECT_EQ(manhattanMap.calculateDistance(0, 0, 3, 4), 7);

    DijkstraMap chebyshevMap(10, 10, DistanceType::Chebyshev);
    EXPECT_EQ(chebyshevMap.calculateDistance(0, 0, 3, 4), 4);

    DijkstraMap euclideanMap(10, 10, DistanceType::Euclidean);
    EXPECT_EQ(euclideanMap.calculateDistance(0, 0, 3, 4), 5);
}

// Test rounding behavior of Euclidean distance
TEST_F(DistanceTypeTest, EuclideanRounding) {
    DijkstraMap map(10, 10, DistanceType::Euclidean);

    // sqrt(2) ≈ 1.414 → rounds to 1
    EXPECT_EQ(map.calculateDistance(0, 0, 1, 1), 1);

    // sqrt(5) ≈ 2.236 → rounds to 2
    EXPECT_EQ(map.calculateDistance(0, 0, 2, 1), 2);
    EXPECT_EQ(map.calculateDistance(0, 0, 1, 2), 2);

    // sqrt(8) ≈ 2.828 → rounds to 3
    EXPECT_EQ(map.calculateDistance(0, 0, 2, 2), 3);
}
