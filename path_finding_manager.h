#ifndef PATHFINDING_MANAGER_H
#define PATHFINDING_MANAGER_H

#include <vector>
#include <unordered_map>
#include <SFML/Graphics.hpp>
#include "Node.h"
#include "Edge.h"
#include "Graph.h"

class PathFindingManager {
public:
    PathFindingManager(WindowManager* win) : window_manager(win), src(nullptr), dest(nullptr) {}

    void dijkstra(Graph& graph);
    void render(sf::RenderWindow& window);
    void set_final_path(std::unordered_map<Node*, Node*>& prev);

    void set_source(Node* node) { src = node; }
    void set_destination(Node* node) { dest = node; }

private:
    WindowManager* window_manager;
    Node* src;
    Node* dest;
    std::vector<sf::VertexArray> path;
    std::vector<sf::VertexArray> visited_edges;
};

#endif
