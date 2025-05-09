#include "dijkstra_planner/dijkstra.hpp"
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace dijkstra_planner {

    // Constructor to initialize grid dimensions
    DijkstraPlanner::DijkstraPlanner(int width, int height) : width_(width), height_(height)
    {
        // Initialize the grid with invalid nodes
        grid_ = std::vector<std::vector<Node>>(width, std::vector<Node>(height));

        // Define possible directions to move on the grid (4-connected)
        directions_ = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};

        // Initialize start_ and goal_ to default values (can be set later)
        start_.pose.position.x = 0;
        start_.pose.position.y = 0;
        goal_.pose.position.x = 0;
        goal_.pose.position.y = 0;
    }

    // Method to set start and goal positions
    void DijkstraPlanner::setStartAndGoal(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal)
    {
        start_ = start;
        goal_ = goal;
    }

    // Check if the position is within the grid bounds
    bool DijkstraPlanner::isValid(int x, int y)
    {
        return (x >= 0 && x < width_ && y >= 0 && y < height_);
    }

    // Reconstruct the path from the goal node to the start node
    std::vector<geometry_msgs::Pose> DijkstraPlanner::reconstructPath(Node* goalNode)
    {
        std::vector<geometry_msgs::Pose> path;
        Node* currentNode = goalNode;
        while (currentNode != nullptr)
        {
            geometry_msgs::Pose pose;
            pose.position.x = currentNode->x;
            pose.position.y = currentNode->y;
            path.push_back(pose);
            currentNode = currentNode->parent;
        }

        // Reverse the path so it goes from start to goal
        std::reverse(path.begin(), path.end());
        return path;
    }

    // Calculate Euclidean distance between two points
    float DijkstraPlanner::calculateHeuristic(int x1, int y1, int x2, int y2)
    {
        return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    }

    // Calculate the path using Dijkstra's algorithm
    std::vector<geometry_msgs::Pose> DijkstraPlanner::calculateDijkstraPath(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal)
    {
        // Clear previous grid data
        for (int i = 0; i < width_; ++i)
        {
            for (int j = 0; j < height_; ++j)
            {
                grid_[i][j] = Node(i, j);
            }
        }

        int startX = static_cast<int>(start.position.x);
        int startY = static_cast<int>(start.position.y);
        int goalX = static_cast<int>(goal.position.x);
        int goalY = static_cast<int>(goal.position.y);

        // Priority queue for Dijkstra's algorithm (min-heap)
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;

        // Start node
        Node* startNode = &grid_[startX][startY];
        startNode->cost = 0;
        openSet.push(*startNode);

        // Dijkstra's algorithm
        while (!openSet.empty())
        {
            Node currentNode = openSet.top();
            openSet.pop();

            // If we reached the goal, reconstruct the path
            if (currentNode.x == goalX && currentNode.y == goalY)
            {
                return reconstructPath(&grid_[goalX][goalY]);
            }

            // Explore neighbors
            for (const auto& direction : directions_)
            {
                int newX = currentNode.x + direction.first;
                int newY = currentNode.y + direction.second;

                if (isValid(newX, newY))
                {
                    Node* neighbor = &grid_[newX][newY];
                    float newCost = currentNode.cost + calculateHeuristic(currentNode.x, currentNode.y, newX, newY);

                    if (newCost < neighbor->cost)
                    {
                        neighbor->cost = newCost;
                        neighbor->parent = &grid_[currentNode.x][currentNode.y];
                        openSet.push(*neighbor);
                    }
                }
            }
        }

        return {};  // Return empty path if no path found
    }

    // Try computing the path with logging for debugging
    bool DijkstraPlanner::tryComputePath()
    {
        // Log start and goal positions
        ROS_INFO("Start position: (%f, %f)", start_.pose.position.x, start_.pose.position.y);
        ROS_INFO("Goal position: (%f, %f)", goal_.pose.position.x, goal_.pose.position.y);

        // Log if the function was entered
        ROS_INFO("Entered tryComputePath().");

        // Calculate the path using Dijkstra's algorithm
        std::vector<geometry_msgs::Pose> path = calculateDijkstraPath(start_.pose, goal_.pose);

        // Before returning the path, confirm success
        ROS_INFO("Path computation complete. Path length: %lu", path.size());

        if (path.empty())
        {
            ROS_WARN("No valid path found.");
            return false;  // No path found
        }

        // Optionally publish or process the path
        // For example: path_publisher_.publish(path);

        return true;  // Path found
    }

    // Create the planner instance
    DijkstraPlanner* createPlanner(int width, int height)
    {
        return new DijkstraPlanner(width, height);
    }

} // namespace dijkstra_planner

