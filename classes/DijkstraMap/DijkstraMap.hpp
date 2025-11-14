#pragma once
#include <vector>
#include <tuple>
#include <limits>
#include <cmath>

/**
 * @brief Distance calculation methods for Dijkstra maps
 */
enum class DistanceType 
{
    Manhattan,    // Sum of absolute differences (|dx| + |dy|)
    Chebyshev,    // Maximum of absolute differences (max(|dx|, |dy|))  
    Euclidean     // Square root of sum of squares (sqrt(dx² + dy²))
};

/**
 * @brief Represents a Dijkstra map for pathfinding and connectivity analysis
 * 
 * A Dijkstra map stores distance values from goal tiles to all other reachable tiles.
 * Unreachable tiles retain their initial infinite distance value.
 * Supports multiple distance calculation methods.
 */
class DijkstraMap
{
public:
    // Use a large value to represent infinite/unreachable distance
    static constexpr int UNREACHABLE = std::numeric_limits<int>::max();
    
private:
    int width;
    int height;
    std::vector<std::vector<int>> distances;
    DistanceType distanceType;
    
public:
    /**
     * @brief Constructor - initializes all distances to UNREACHABLE
     * @param w Width of the map
     * @param h Height of the map  
     * @param distType Distance calculation method (default: Manhattan)
     */
    DijkstraMap(int widthParam, int heightParam, DistanceType distTypeParam = DistanceType::Euclidean ) 
                : width(widthParam), height(heightParam), distanceType(distTypeParam)
    {
        distances.resize(width);
        for (int x = 0; x < width; ++x) 
        {
            distances[x].resize(height, UNREACHABLE);
        }
    }
    
    /**
     * @brief Get the distance value at a specific coordinate
     * @param x X coordinate
     * @param y Y coordinate
     * @return Distance value, or UNREACHABLE if out of bounds
     */
    int getDistance(int x, int y) const
    {
        if (x < 0 || x >= width || y < 0 || y >= height) 
        {
            return UNREACHABLE;
        }
        return distances[x][y];
    }
    
    /**
     * @brief Set the distance value at a specific coordinate
     * @param x X coordinate
     * @param y Y coordinate
     * @param distance Distance value to set
     */
    void setDistance(int x, int y, int distance)
    {
        if (x >= 0 && x < width && y >= 0 && y < height) 
        {
            distances[x][y] = distance;
        }
    }
    
    /**
     * @brief Check if coordinates are within map bounds
     * @param x X coordinate
     * @param y Y coordinate
     * @return True if within bounds
     */
    bool isWithinBounds(int x, int y) const
    {
        return x >= 0 && x < width && y >= 0 && y < height;
    }
    
    /**
     * @brief Check if a tile is reachable (distance is not UNREACHABLE)
     * @param x X coordinate
     * @param y Y coordinate
     * @return True if reachable
     */
    bool isReachable(int x, int y) const
    {
        return getDistance(x, y) != UNREACHABLE;
    }
    
    /**
     * @brief Get map dimensions
     * @return Tuple of (width, height)
     */
    std::tuple<int, int> getDimensions() const
    {
        return std::make_tuple(width, height);
    }
    
    /**
     * @brief Get the current distance calculation type
     * @return The distance type being used
     */
    DistanceType getDistanceType() const
    {
        return distanceType;
    }
    
    /**
     * @brief Set the distance calculation type
     * @param distType New distance calculation method
     */
    void setDistanceType(DistanceType distType)
    {
        distanceType = distType;
    }
    
    /**
     * @brief Calculate distance between two points using current distance type
     * @param x1 First point X coordinate
     * @param y1 First point Y coordinate  
     * @param x2 Second point X coordinate
     * @param y2 Second point Y coordinate
     * @return Distance value
     */
    int calculateDistance(int x1, int y1, int x2, int y2) const
    {
        int dx = x2 - x1;
        int dy = y2 - y1;
        
        switch (distanceType) 
        {
            case DistanceType::Manhattan:
                return calculateManhattanDistance(dx, dy);
                
            case DistanceType::Chebyshev:
                return calculateChebyshevDistance(dx, dy);
                
            case DistanceType::Euclidean:
                return calculateEuclideanDistance(dx, dy);
                
            default:
                return calculateEuclideanDistance(dx, dy);
        }
    }
    
    /**
     * @brief Clear the map - reset all distances to UNREACHABLE
     */
    void clear()
    {
        for (int x = 0; x < width; ++x) 
        {
            for (int y = 0; y < height; ++y) 
            {
                distances[x][y] = UNREACHABLE;
            }
        }
    }

private:
    /**
     * @brief Calculate Manhattan distance: |dx| + |dy|
     * @param dx X difference
     * @param dy Y difference
     * @return Manhattan distance
     */
    int calculateManhattanDistance(int dx, int dy) const
    {
        return std::abs(dx) + std::abs(dy);
    }
    
    /**
     * @brief Calculate Chebyshev distance: max(|dx|, |dy|)
     * @param dx X difference
     * @param dy Y difference
     * @return Chebyshev distance
     */
    int calculateChebyshevDistance(int dx, int dy) const
    {
        return std::max(std::abs(dx), std::abs(dy));
    }
    
    /**
     * @brief Calculate Euclidean distance: sqrt(dx² + dy²)
     * @param dx X difference
     * @param dy Y difference
     * @return Euclidean distance (rounded to nearest integer)
     */
    int calculateEuclideanDistance(int dx, int dy) const
    {
        double distance = std::sqrt(dx * dx + dy * dy);
        return static_cast<int>(std::round(distance));
    }
};