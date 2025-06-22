#ifndef PATH_FINDING_MANAGER_H
#define PATH_FINDING_MANAGER_H

#include <vector>
#include <unordered_map>
#include <queue>
#include <limits>
#include <SFML/Graphics.hpp>
#include "node.h"
#include "edge.h"
#include "graph.h"
#include "window_manager.h"

class PathFindingManager {
public:
    PathFindingManager(WindowManager* wm) : window_manager(wm), src(nullptr), dest(nullptr) {}

    void set_source(Node* node) { src = node; }
    void set_destination(Node* node) { dest = node; }

    void dijkstra(Graph& graph) {
        visited_edges.clear();
        path.clear();

        std::priority_queue<std::pair<float, Node*>, std::vector<std::pair<float, Node*>>, std::greater<>> pq;
        std::unordered_map<Node*, float> dist;
        std::unordered_map<Node*, Node*> prev;

        for (auto& pair : graph.nodes) {
            dist[pair.second] = std::numeric_limits<float>::infinity();
            prev[pair.second] = nullptr;
        }

        dist[src] = 0.0f;
        pq.push({0.0f, src});

        while (!pq.empty()) {
            auto [cost, u] = pq.top();
            pq.pop();

            for (Edge* edge : u->edges) {
                Node* v = edge->dest;
                float alt = dist[u] + edge->length;
                if (alt < dist[v]) {
                    dist[v] = alt;
                    prev[v] = u;
                    pq.push({alt, v});

                    sf::VertexArray line(sf::Lines, 2);
                    line[0].position = u->coord;
                    line[1].position = v->coord;
                    line[0].color = sf::Color::Yellow;
                    line[1].color = sf::Color::Yellow;
                    visited_edges.push_back(line);
                }
            }
        }

        set_final_path(prev);
    }

    void set_final_path(const std::unordered_map<Node*, Node*>& prev) {
        Node* current = dest;
        while (current && prev.at(current)) {
            Node* from = prev.at(current);
            sf::VertexArray line(sf::Lines, 2);
            line[0].position = from->coord;
            line[1].position = current->coord;
            line[0].color = sf::Color::Magenta;
            line[1].color = sf::Color::Magenta;
            path.push_back(line);
            current = from;
        }
    }

    void render(sf::RenderWindow& window) {
        for (auto& e : visited_edges)
            window.draw(e);
        for (auto& p : path)
            window.draw(p);
    }

private:
    WindowManager* window_manager;
    Node* src;
    Node* dest;
    std::vector<sf::VertexArray> path;
    std::vector<sf::VertexArray> visited_edges;
};

#endif
