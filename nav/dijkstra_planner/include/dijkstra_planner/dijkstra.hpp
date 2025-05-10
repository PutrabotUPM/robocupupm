#ifndef DIJKSTRA_HPP
#define DIJKSTRA_HPP

#include <vector>
#include <queue>
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

namespace dijkstra_planner {

    // Define the structure for a node
    struct Node {
        int x;  // x-coordinate
        int y;  // y-coordinate
        float cost;  // Cost to reach this node from the start node
        Node* parent;  // Pointer to the parent node, used for path reconstruction

        Node(int x = 0, int y = 0, float cost = std::numeric_limits<float>::infinity(), Node* parent = nullptr)
            : x(x), y(y), cost(cost), parent(parent) {}

        // Comparator for priority queue (min-heap)
        bool operator>(const Node& other) const {
            return cost > other.cost;
        }
    };

    // The Dijkstra planner class
    class DijkstraPlanner {
    public:
        // Constructor, initialize the grid size
        DijkstraPlanner(int width, int height);

        // Method to calculate the path using Dijkstra's algorithm
        std::vector<geometry_msgs::Pose> calculateDijkstraPath(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal);

        // Method to try computing the path, with logging
        bool tryComputePath();

        // Declare start_ and goal_ as member variables
        geometry_msgs::PoseStamped start_;  // Start position
        geometry_msgs::PoseStamped goal_;   // Goal position

        // Method to set start and goal positions
        void setStartAndGoal(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal);

    private:
        // Grid dimensions
        int width_;
        int height_;

        // Direction vectors for moving in a 4-connected grid
        std::vector<std::pair<int, int>> directions_;

        // Method to check if a position is within the grid boundaries
        bool isValid(int x, int y);

        // Method to reconstruct the path from goal to start
        std::vector<geometry_msgs::Pose> reconstructPath(Node* goalNode);

        // Method to calculate the Euclidean distance between two positions
        float calculateHeuristic(int x1, int y1, int x2, int y2);

        // Grid (represented as a vector of vectors of nodes)
        std::vector<std::vector<Node>> grid_;
    };

    // Function to create and initialize a DijkstraPlanner instance
    DijkstraPlanner* createPlanner(int width, int height);

} // namespace dijkstra_planner

#endif // DIJKSTRA_HPP

