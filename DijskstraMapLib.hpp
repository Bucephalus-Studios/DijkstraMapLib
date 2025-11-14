#pragma once
#include <vector>
#include <queue>
#include <tuple>
#include <functional>
#include "classes/DijkstraMap/DijkstraMap.hpp"

namespace DijkstraMapLib
{
    
    /**
     * @brief Generate a Dijkstra map using flood-fill from goal positions
     * 
     * @param dijkstraMap The map to populate with distances
     * @param goals Vector of goal positions (distance 0)
     * @param isWalkable Function to determine if a tile is walkable: bool(int x, int y)
     */
    template<typename WalkableFunc>
    void generateDijkstraMap(DijkstraMap& dijkstraMap, 
                           const std::vector<std::tuple<int, int>>& goals,
                           WalkableFunc isWalkable)
    {
        // Clear the map first
        dijkstraMap.clear();
        
        // Priority queue for processing tiles: (distance, x, y)
        std::priority_queue<std::tuple<int, int, int>, 
                           std::vector<std::tuple<int, int, int>>,
                           std::greater<std::tuple<int, int, int>>> queue;
        
        // Set all goal positions to distance 0 and add to queue
        for (const auto& goal : goals) 
        {
            int x = std::get<0>(goal);
            int y = std::get<1>(goal);
            if (dijkstraMap.isWithinBounds(x, y) && isWalkable(x, y)) 
            {
                dijkstraMap.setDistance(x, y, 0);
                queue.push(std::make_tuple(0, x, y));
            }
        }
        
        // Determine movement directions based on distance type
        std::vector<std::tuple<int, int>> directions;
        if (dijkstraMap.getDistanceType() == DistanceType::Chebyshev) 
        {
            // 8-directional movement for Chebyshev distance (includes diagonals)
            directions = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}, 
                         {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
        } 
        else 
        {
            // 4-directional movement for Manhattan and Euclidean
            directions = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}};
        }
        
        // Dijkstra flood-fill algorithm
        while (!queue.empty()) 
        {
            auto current = queue.top();
            queue.pop();
            
            int currentDist = std::get<0>(current);
            int currentX = std::get<1>(current);
            int currentY = std::get<2>(current);
            
            // Skip if we've already found a better path to this tile
            if (currentDist > dijkstraMap.getDistance(currentX, currentY)) 
            {
                continue;
            }
            
            // Check all neighbors based on distance type
            for (const auto& dir : directions) 
            {
                int newX = currentX + std::get<0>(dir);
                int newY = currentY + std::get<1>(dir);
                
                // Skip if out of bounds or not walkable
                if (!dijkstraMap.isWithinBounds(newX, newY) || !isWalkable(newX, newY)) 
                {
                    continue;
                }
                
                // Calculate movement cost based on distance type
                int movementCost = dijkstraMap.calculateDistance(currentX, currentY, newX, newY);
                int newDistance = currentDist + movementCost;
                
                // If we found a shorter path to this neighbor
                if (newDistance < dijkstraMap.getDistance(newX, newY)) 
                {
                    dijkstraMap.setDistance(newX, newY, newDistance);
                    queue.push(std::make_tuple(newDistance, newX, newY));
                }
            }
        }
    }
    
    /**
     * @brief Find all unreachable tiles in a map
     * 
     * @param dijkstraMap The Dijkstra map to analyze
     * @param isWalkable Function to determine if a tile should be walkable: bool(int x, int y)
     * @return Vector of unreachable tile coordinates
     */
    template<typename WalkableFunc>
    std::vector<std::tuple<int, int>> findUnreachableTiles(const DijkstraMap& dijkstraMap,
                                                         WalkableFunc isWalkable)
    {
        std::vector<std::tuple<int, int>> unreachableTiles;
        
        auto [width, height] = dijkstraMap.getDimensions();
        
        for (int x = 0; x < width; ++x) 
        {
            for (int y = 0; y < height; ++y) 
            {
                // If the tile should be walkable but is unreachable
                if (isWalkable(x, y) && !dijkstraMap.isReachable(x, y)) 
                {
                    unreachableTiles.push_back(std::make_tuple(x, y));
                }
            }
        }
        
        return unreachableTiles;
    }
    
    /**
     * @brief Generate Dijkstra map from a single goal position
     * 
     * @param dijkstraMap The map to populate
     * @param goalX Goal X coordinate
     * @param goalY Goal Y coordinate
     * @param isWalkable Function to determine walkable tiles
     */
    template<typename WalkableFunc>
    void generateDijkstraMapFromSingleGoal(DijkstraMap& dijkstraMap,
                                         int goalX, int goalY,
                                         WalkableFunc isWalkable)
    {
        std::vector<std::tuple<int, int>> goals = {std::make_tuple(goalX, goalY)};
        generateDijkstraMap(dijkstraMap, goals, isWalkable);
    }
}