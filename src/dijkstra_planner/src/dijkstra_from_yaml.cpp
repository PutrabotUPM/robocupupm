// dijkstra_from_yaml.cpp
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <vector>
#include <queue>
#include <string>
#include <limits>
#include <algorithm>
#include <yaml-cpp/yaml.h>

struct Node {
    std::string id;
    float cost = std::numeric_limits<float>::infinity();
    Node* parent = nullptr;
    std::vector<std::pair<Node*, float>> neighbors;

    Node(const std::string& id_) : id(id_) {}
};

// Priority queue comparator
struct CompareNode {
    bool operator()(Node* a, Node* b) {
        return a->cost > b->cost;
    }
};

std::vector<Node*> dijkstra(Node* start, Node* goal) {
    start->cost = 0;
    std::priority_queue<Node*, std::vector<Node*>, CompareNode> openSet;
    openSet.push(start);

    while (!openSet.empty()) {
        Node* current = openSet.top();
        openSet.pop();

        if (current == goal) {
            std::vector<Node*> path;
            for (Node* n = goal; n != nullptr; n = n->parent) {
                path.push_back(n);
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (const auto& [neighbor, edge_cost] : current->neighbors) {
            float newCost = current->cost + edge_cost;
            if (newCost < neighbor->cost) {
                neighbor->cost = newCost;
                neighbor->parent = current;
                openSet.push(neighbor);
            }
        }
    }
    return {};  // No path found
}

std::unordered_map<std::string, Node*> loadGraphFromYAML(const std::string& filename) {
    std::unordered_map<std::string, Node*> graph;
    YAML::Node config = YAML::LoadFile(filename);

    // Create nodes
    for (const auto& node : config["nodes"]) {
        std::string id = node["id"].as<std::string>();
        graph[id] = new Node(id);
    }

    // Add neighbors
    for (const auto& node : config["nodes"]) {
        std::string id = node["id"].as<std::string>();
        Node* current = graph[id];
        for (const auto& neighbor : node["neighbors"]) {
            std::string nid = neighbor["id"].as<std::string>();
            float cost = neighbor["cost"].as<float>();
            current->neighbors.push_back({graph[nid], cost});
        }
    }

    return graph;
}

int main() {
    auto graph = loadGraphFromYAML("graph.yaml");
    Node* start = graph["A"];
    Node* goal  = graph["D"];

    auto path = dijkstra(start, goal);
    if (path.empty()) {
        std::cout << "No path found.\n";
    } else {
        std::cout << "Path: ";
        for (const auto& node : path) {
            std::cout << node->id << " ";
        }
        std::cout << "\n";
    }

    // Cleanup
    for (auto& [id, node] : graph) {
        delete node;
    }
    return 0;
}
