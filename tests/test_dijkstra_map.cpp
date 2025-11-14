#include <gtest/gtest.h>
#include "classes/DijkstraMap/DijkstraMap.hpp"

// Test fixture for DijkstraMap tests
class DijkstraMapTest : public ::testing::Test {
protected:
    static constexpr int testWidth = 10;
    static constexpr int testHeight = 10;
};

// Constructor tests
TEST_F(DijkstraMapTest, ConstructorInitializesCorrectly) {
    DijkstraMap map(testWidth, testHeight);
    auto [width, height] = map.getDimensions();

    EXPECT_EQ(width, testWidth);
    EXPECT_EQ(height, testHeight);
    EXPECT_EQ(map.getDistanceType(), DistanceType::Euclidean);
}

TEST_F(DijkstraMapTest, ConstructorWithDistanceType) {
    DijkstraMap map(testWidth, testHeight, DistanceType::Manhattan);
    EXPECT_EQ(map.getDistanceType(), DistanceType::Manhattan);
}

TEST_F(DijkstraMapTest, AllTilesInitializedAsUnreachable) {
    DijkstraMap map(testWidth, testHeight);

    for (int x = 0; x < testWidth; ++x) {
        for (int y = 0; y < testHeight; ++y) {
            EXPECT_EQ(map.getDistance(x, y), DijkstraMap::UNREACHABLE);
            EXPECT_FALSE(map.isReachable(x, y));
        }
    }
}

// Bounds checking tests
TEST_F(DijkstraMapTest, IsWithinBoundsValidCoordinates) {
    DijkstraMap map(testWidth, testHeight);

    EXPECT_TRUE(map.isWithinBounds(0, 0));
    EXPECT_TRUE(map.isWithinBounds(testWidth - 1, testHeight - 1));
    EXPECT_TRUE(map.isWithinBounds(5, 5));
}

TEST_F(DijkstraMapTest, IsWithinBoundsInvalidCoordinates) {
    DijkstraMap map(testWidth, testHeight);

    EXPECT_FALSE(map.isWithinBounds(-1, 0));
    EXPECT_FALSE(map.isWithinBounds(0, -1));
    EXPECT_FALSE(map.isWithinBounds(testWidth, 0));
    EXPECT_FALSE(map.isWithinBounds(0, testHeight));
    EXPECT_FALSE(map.isWithinBounds(-1, -1));
    EXPECT_FALSE(map.isWithinBounds(testWidth, testHeight));
}

// Distance get/set tests
TEST_F(DijkstraMapTest, SetAndGetDistance) {
    DijkstraMap map(testWidth, testHeight);

    map.setDistance(5, 5, 42);
    EXPECT_EQ(map.getDistance(5, 5), 42);
    EXPECT_TRUE(map.isReachable(5, 5));
}

TEST_F(DijkstraMapTest, SetDistanceOutOfBoundsIsIgnored) {
    DijkstraMap map(testWidth, testHeight);

    map.setDistance(-1, 0, 10);
    map.setDistance(0, -1, 10);
    map.setDistance(testWidth, 0, 10);
    map.setDistance(0, testHeight, 10);

    // No crash expected, operations should be safely ignored
    SUCCEED();
}

TEST_F(DijkstraMapTest, GetDistanceOutOfBoundsReturnsUnreachable) {
    DijkstraMap map(testWidth, testHeight);

    EXPECT_EQ(map.getDistance(-1, 0), DijkstraMap::UNREACHABLE);
    EXPECT_EQ(map.getDistance(0, -1), DijkstraMap::UNREACHABLE);
    EXPECT_EQ(map.getDistance(testWidth, 0), DijkstraMap::UNREACHABLE);
    EXPECT_EQ(map.getDistance(0, testHeight), DijkstraMap::UNREACHABLE);
}

// Clear tests
TEST_F(DijkstraMapTest, ClearResetsAllDistances) {
    DijkstraMap map(testWidth, testHeight);

    // Set some distances
    map.setDistance(0, 0, 0);
    map.setDistance(5, 5, 10);
    map.setDistance(9, 9, 20);

    // Clear
    map.clear();

    // Verify all distances are reset
    for (int x = 0; x < testWidth; ++x) {
        for (int y = 0; y < testHeight; ++y) {
            EXPECT_EQ(map.getDistance(x, y), DijkstraMap::UNREACHABLE);
            EXPECT_FALSE(map.isReachable(x, y));
        }
    }
}

// Distance type tests
TEST_F(DijkstraMapTest, SetAndGetDistanceType) {
    DijkstraMap map(testWidth, testHeight);

    map.setDistanceType(DistanceType::Manhattan);
    EXPECT_EQ(map.getDistanceType(), DistanceType::Manhattan);

    map.setDistanceType(DistanceType::Chebyshev);
    EXPECT_EQ(map.getDistanceType(), DistanceType::Chebyshev);

    map.setDistanceType(DistanceType::Euclidean);
    EXPECT_EQ(map.getDistanceType(), DistanceType::Euclidean);
}

// Distance calculation tests
TEST_F(DijkstraMapTest, CalculateManhattanDistance) {
    DijkstraMap map(testWidth, testHeight, DistanceType::Manhattan);

    EXPECT_EQ(map.calculateDistance(0, 0, 1, 0), 1);  // Horizontal
    EXPECT_EQ(map.calculateDistance(0, 0, 0, 1), 1);  // Vertical
    EXPECT_EQ(map.calculateDistance(0, 0, 1, 1), 2);  // Diagonal
    EXPECT_EQ(map.calculateDistance(0, 0, 3, 4), 7);  // |3| + |4| = 7
    EXPECT_EQ(map.calculateDistance(5, 5, 2, 3), 5);  // |5-2| + |5-3| = 5
}

TEST_F(DijkstraMapTest, CalculateChebyshevDistance) {
    DijkstraMap map(testWidth, testHeight, DistanceType::Chebyshev);

    EXPECT_EQ(map.calculateDistance(0, 0, 1, 0), 1);  // Horizontal
    EXPECT_EQ(map.calculateDistance(0, 0, 0, 1), 1);  // Vertical
    EXPECT_EQ(map.calculateDistance(0, 0, 1, 1), 1);  // Diagonal (Chebyshev!)
    EXPECT_EQ(map.calculateDistance(0, 0, 3, 4), 4);  // max(3, 4) = 4
    EXPECT_EQ(map.calculateDistance(5, 5, 2, 3), 3);  // max(3, 2) = 3
}

TEST_F(DijkstraMapTest, CalculateEuclideanDistance) {
    DijkstraMap map(testWidth, testHeight, DistanceType::Euclidean);

    EXPECT_EQ(map.calculateDistance(0, 0, 1, 0), 1);   // sqrt(1) = 1
    EXPECT_EQ(map.calculateDistance(0, 0, 0, 1), 1);   // sqrt(1) = 1
    EXPECT_EQ(map.calculateDistance(0, 0, 1, 1), 1);   // sqrt(2) ≈ 1.414 → 1
    EXPECT_EQ(map.calculateDistance(0, 0, 3, 4), 5);   // sqrt(9 + 16) = 5
    EXPECT_EQ(map.calculateDistance(0, 0, 5, 12), 13); // sqrt(25 + 144) = 13
}

// Reachability tests
TEST_F(DijkstraMapTest, IsReachableForSetDistances) {
    DijkstraMap map(testWidth, testHeight);

    EXPECT_FALSE(map.isReachable(5, 5));

    map.setDistance(5, 5, 0);
    EXPECT_TRUE(map.isReachable(5, 5));

    map.setDistance(5, 5, 100);
    EXPECT_TRUE(map.isReachable(5, 5));
}

TEST_F(DijkstraMapTest, IsReachableForOutOfBounds) {
    DijkstraMap map(testWidth, testHeight);

    EXPECT_FALSE(map.isReachable(-1, 0));
    EXPECT_FALSE(map.isReachable(testWidth, 0));
}

// Dimensions test
TEST_F(DijkstraMapTest, GetDimensionsReturnsCorrectValues) {
    DijkstraMap map(15, 25);
    auto [width, height] = map.getDimensions();

    EXPECT_EQ(width, 15);
    EXPECT_EQ(height, 25);
}

// Edge cases
TEST_F(DijkstraMapTest, SingleTileMap) {
    DijkstraMap map(1, 1);
    auto [width, height] = map.getDimensions();

    EXPECT_EQ(width, 1);
    EXPECT_EQ(height, 1);

    map.setDistance(0, 0, 0);
    EXPECT_EQ(map.getDistance(0, 0), 0);
    EXPECT_TRUE(map.isReachable(0, 0));
}

TEST_F(DijkstraMapTest, LargeMap) {
    DijkstraMap map(1000, 1000);
    auto [width, height] = map.getDimensions();

    EXPECT_EQ(width, 1000);
    EXPECT_EQ(height, 1000);

    // Should handle large maps without issues
    map.setDistance(999, 999, 42);
    EXPECT_EQ(map.getDistance(999, 999), 42);
}
