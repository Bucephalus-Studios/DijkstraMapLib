#pragma once
#include <functional>
#include <queue>
#include <tuple>
#include <vector>
#include "classes/DijkstraMap/DijkstraMap.hpp"

namespace DijkstraMapLib
{
    // Type alias for coordinates
    using Coord = std::tuple<int, int>;
    using CoordList = std::vector<Coord>;

    // Type alias for priority queue entries: (distance, x, y)
    using QueueEntry = std::tuple<int, int, int>;

    namespace detail
    {
        /**
         * @brief Get movement directions based on distance type
         * @param distType The distance type determining movement pattern
         * @return Vector of direction offsets (dx, dy)
         */
        inline const CoordList& getDirections(DistanceType distType)
        {
            static const CoordList fourDirectional = {
                {0, 1}, {0, -1}, {1, 0}, {-1, 0}
            };

            static const CoordList eightDirectional = {
                {0, 1}, {0, -1}, {1, 0}, {-1, 0},
                {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
            };

            return (distType == DistanceType::Chebyshev) ? eightDirectional : fourDirectional;
        }

        /**
         * @brief Initialize queue with goal positions
         * @param dijkstraMap The map being populated
         * @param goals Goal positions to start from
         * @param isWalkable Function to check walkability
         * @param queue Priority queue to populate
         */
        template<typename WalkableFunc>
        void initializeGoals(DijkstraMap& dijkstraMap,
                           const CoordList& goals,
                           WalkableFunc isWalkable,
                           std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>>& queue)
        {
            for (const auto& [goalX, goalY] : goals) {
                if (!dijkstraMap.isWithinBounds(goalX, goalY) || !isWalkable(goalX, goalY)) {
                    continue;
                }

                dijkstraMap.setDistance(goalX, goalY, 0);
                queue.push({0, goalX, goalY});
            }
        }

        /**
         * @brief Process a single neighbor during flood-fill
         * @param dijkstraMap The map being populated
         * @param currentDist Distance to current position
         * @param currentX Current X coordinate
         * @param currentY Current Y coordinate
         * @param dx Direction X offset
         * @param dy Direction Y offset
         * @param isWalkable Function to check walkability
         * @param queue Priority queue for processing
         * @return True if neighbor was updated
         */
        template<typename WalkableFunc>
        bool processNeighbor(DijkstraMap& dijkstraMap,
                           int currentDist,
                           int currentX,
                           int currentY,
                           int dx,
                           int dy,
                           WalkableFunc isWalkable,
                           std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>>& queue)
        {
            const int neighborX = currentX + dx;
            const int neighborY = currentY + dy;

            if (!dijkstraMap.isWithinBounds(neighborX, neighborY) || !isWalkable(neighborX, neighborY)) {
                return false;
            }

            const int movementCost = dijkstraMap.calculateDistance(currentX, currentY, neighborX, neighborY);
            const int newDistance = currentDist + movementCost;

            if (newDistance >= dijkstraMap.getDistance(neighborX, neighborY)) {
                return false;
            }

            dijkstraMap.setDistance(neighborX, neighborY, newDistance);
            queue.push({newDistance, neighborX, neighborY});
            return true;
        }
    } // namespace detail
    
    /**
     * @brief Generate a Dijkstra map using flood-fill from goal positions
     *
     * @param dijkstraMap The map to populate with distances
     * @param goals Vector of goal positions (distance 0)
     * @param isWalkable Function to determine if a tile is walkable: bool(int x, int y)
     */
    template<typename WalkableFunc>
    void generateDijkstraMap(DijkstraMap& dijkstraMap,
                           const CoordList& goals,
                           WalkableFunc isWalkable)
    {
        dijkstraMap.clear();

        // Priority queue for processing tiles: (distance, x, y)
        std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> queue;

        // Initialize goals
        detail::initializeGoals(dijkstraMap, goals, isWalkable, queue);

        // Get movement directions based on distance type
        const auto& directions = detail::getDirections(dijkstraMap.getDistanceType());

        // Dijkstra flood-fill algorithm
        while (!queue.empty()) {
            const auto [currentDist, currentX, currentY] = queue.top();
            queue.pop();

            // Skip if we've already found a better path to this tile
            if (currentDist > dijkstraMap.getDistance(currentX, currentY)) {
                continue;
            }

            // Process all neighbors
            for (const auto& [dx, dy] : directions) {
                detail::processNeighbor(dijkstraMap, currentDist, currentX, currentY,
                                      dx, dy, isWalkable, queue);
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
    CoordList findUnreachableTiles(const DijkstraMap& dijkstraMap, WalkableFunc isWalkable)
    {
        CoordList unreachableTiles;
        const auto [width, height] = dijkstraMap.getDimensions();

        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                if (isWalkable(x, y) && !dijkstraMap.isReachable(x, y)) {
                    unreachableTiles.emplace_back(x, y);
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
                                         int goalX,
                                         int goalY,
                                         WalkableFunc isWalkable)
    {
        generateDijkstraMap(dijkstraMap, {{goalX, goalY}}, isWalkable);
    }
}