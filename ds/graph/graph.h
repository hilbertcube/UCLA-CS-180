#pragma once

#include "../../ds/headers.hpp"
#include "union_find.h"

// Forward declarations for friend functions
template <typename T, typename W> class Graph;
template <typename T, typename W> struct MSTEdge;
template <typename T, typename W> std::vector<MSTEdge<T, W>> prim_mst(const Graph<T, W>& graph);
template <typename T, typename W> std::vector<MSTEdge<T, W>> kruskal_mst(const Graph<T, W>& graph);

template <typename T, typename W = int>
class Graph {
private:
    static_assert(std::is_arithmetic<W>::value, "Weight must be numeric");
    
    struct Edge {
        T dest;
        W weight;
        
        Edge(const T& d, W w) : dest(d), weight(w) {}
    };
    
    // Adjacency list representation: vertex -> list of edges
    std::unordered_map<T, std::vector<Edge>> adj_list;
    size_t edge_count;
    
    // Friend function declarations for MST algorithms
    friend std::vector<MSTEdge<T, W>> prim_mst<>(const Graph<T, W>& graph);
    friend std::vector<MSTEdge<T, W>> kruskal_mst<>(const Graph<T, W>& graph);
    
public:
    Graph() : edge_count(0) {}
    
    // Add a vertex to the graph
    void add_vertex(const T& vertex) {
        if (!adj_list.count(vertex)) {
            adj_list[vertex] = std::vector<Edge>();
        }
    }
    
    // Add a directed edge from source to destination
    void add_directed_edge(const T& src, const T& des, W weight = W(1)) {
        add_vertex(src);
        add_vertex(des);
        
        adj_list[src].push_back(Edge(des, weight));
        edge_count++;
    }
    
    // Add an undirected edge between two vertices
    void add_undirected_edge(const T& v1, const T& v2, W weight = W(1)) {
        add_vertex(v1);
        add_vertex(v2);
        
        adj_list[v1].push_back(Edge(v2, weight));
        adj_list[v2].push_back(Edge(v1, weight));
        edge_count += 2;
    }
    
    // Remove a vertex and all associated edges
    void remove_vertex(const T& vertex) {
        if (!adj_list.count(vertex)) {
            return;
        }
        
        // Remove all edges pointing to this vertex
        for (auto& pair : adj_list) {
            auto& edges = pair.second;
            auto new_end = std::remove_if(edges.begin(), edges.end(),
                [&vertex](const Edge& e) { return e.dest == vertex; });
            
            edge_count -= std::distance(new_end, edges.end());
            edges.erase(new_end, edges.end());
        }
        
        // Remove all edges from this vertex
        edge_count -= adj_list[vertex].size();
        
        // Remove the vertex itself
        adj_list.erase(vertex);
    }
    
    // Remove an edge from source to destination
    void remove_edge(const T& src, const T& dest) {
        if (!adj_list.count(src)) {
            return;
        }
        
        auto& edges = adj_list[src];
        auto new_end = std::remove_if(edges.begin(), edges.end(),
            [&dest](const Edge& e) { return e.dest == dest; });
        
        edge_count -= std::distance(new_end, edges.end());
        edges.erase(new_end, edges.end());
    }

    std::vector<T> neighbors_of(const T& start) const {
        if (!adj_list.count(start)) return {};

        std::vector<T> result;

        for(const auto& edge : adj_list.at(start)) {
            result.push_back(edge.dest);
        }
        return result;
    }
    
    // Breadth-First Search traversal starting from a given vertex
    std::vector<T> bfs(const T& start) const {
        if (!adj_list.count(start)) return {};

        std::vector<T> result;
        std::unordered_set<T> visited;
        std::queue<T> q;
        
        q.push(start);
        visited.insert(start);
        
        while (!q.empty()) {
            T current = q.front();
            q.pop();
            result.push_back(current);
            
            for (const auto& edge : adj_list.at(current)) {
                if (!visited.count(edge.dest)) {
                    visited.insert(edge.dest);
                    q.push(edge.dest);
                }
            }
            
        }
        return result;
    }
    
    // Iterative Depth-First Search traversal
    std::vector<T> dfs_iterative(const T& start) const {
        if (!adj_list.count(start)) return {};

        std::vector<T> result;
        std::unordered_set<T> visited;
        std::stack<T> s;
        
        s.push(start);
        
        while (!s.empty()) {
            T current = s.top();
            s.pop();
            
            // if node not visited,
            if (!visited.count(current)) {
                visited.insert(current);    // mark this node as visited
                result.push_back(current);  // push to output
                
                // Push in reverse order to maintain left-to-right traversal
                const auto& edges = adj_list.at(current);
                for (auto it = edges.rbegin(); it != edges.rend(); ++it) {
                    if (!visited.count(it->dest)) {
                        s.push(it->dest);
                    }
                }
            }
        }
        return result;
    }

    void dfs_recursive(const T& vertex, 
                             std::unordered_set<T>& visited, 
                             std::vector<T>& result) const {
        visited.insert(vertex);
        result.push_back(vertex);
        
        if (adj_list.count(vertex)) {
            for (const auto& edge : adj_list.at(vertex)) {
                if (!visited.count(edge.dest)) {
                    dfs_recursive(edge.dest, visited, result);
                }
            }
        }
    }

    // Recursive Depth-First Search traversal
    std::vector<T> dfs_recursive(const T& start) const {
        if (!adj_list.count(start)) return {};

        std::vector<T> result;
        std::unordered_set<T> visited;
        dfs_recursive(start, visited, result);
        
        return result;
    }
    
    // Topological sort using Kahn's algorithm (BFS-based)
    std::vector<T> topological_sort() const {
        std::vector<T> result;
        std::unordered_map<T, int> in_degree; // {vertex, in-degree}
        
        // Initialize in-degrees
        // For each vertex (V iterations)
        for (const auto& pair : adj_list) {
            // every vertex starts with in-degree = 0
            if (!in_degree.count(pair.first)) {
                in_degree[pair.first] = 0;
            }
            
            // For each edge of the vertex's adjacency list
            // Total across ALL vertices = E iterations
            for (const auto& edge : pair.second) {
                in_degree[edge.dest]++;
            }
        }
        
        // Queue all vertices with in-degree 0
        std::queue<T> q;
        for (const auto& pair : in_degree) {
            if (pair.second == 0) {
                q.push(pair.first);
            }
        }
        
        // Process vertices
        while (!q.empty()) {
            T current = q.front();
            q.pop();
            result.push_back(current);
            
            // For each neighbor of that vertex
            for (const auto& edge : adj_list.at(current)) {
                in_degree[edge.dest]--;
                if (in_degree[edge.dest] == 0) {
                    q.push(edge.dest);
                }
            }
        }
        
        // If result doesn't contain all vertices, graph has a cycle
        if (result.size() != adj_list.size()) {
            return std::vector<T>(); // Return empty vector for cyclic graphs
        }
        
        return result;
    }
    
    // Get shortest path from source to destination using Dijkstra
    std::vector<T> shortest_path_Dijkstra(const T& source, const T& destination) const {
        if (!adj_list.count(source) || !adj_list.count(destination)) {
            return {}; // Return empty if vertices don't exist
        }

        std::unordered_map<T, W> distances;
        std::unordered_map<T, T> predecessors;
        std::unordered_set<T> visited;
        
        // Initialize distances
        for (const auto& pair : adj_list) {
            distances[pair.first] = std::numeric_limits<W>::max();
        }
        
        distances[source] = W(0);
        
        // Priority queue: pair<distance, vertex>
        // std::greater<std::pair<W, T>> creates a min-heap instead of the default max heap
        // std::vector<std::pair<W, T>>: container type. std::vector is the default and most
        // choice for priority queue
        // The min is which ever pair has the smallest distance
        // if distance are tied, smallest vertex identifier first
        std::priority_queue<std::pair<W, T>,
                           std::vector<std::pair<W, T>>, 
                           std::greater<std::pair<W, T>>> pq;
        
        pq.push({W(0), source});
        
        while (!pq.empty()) {
            auto current_pair = pq.top();
            pq.pop();
            
            W current_dist = current_pair.first;
            T current_vertex = current_pair.second;
            
            if (visited.count(current_vertex)) {
                continue;
            }
            
            visited.insert(current_vertex);
            
            // If we reached the destination, we can stop
            if (current_vertex == destination) {
                break;
            }
            
            // Check all neighbors
            for (const auto& edge : adj_list.at(current_vertex)) {
                if (!visited.count(edge.dest)) {
                    W new_distance = current_dist + edge.weight;
                    
                    if (new_distance < distances[edge.dest]) {
                        distances[edge.dest] = new_distance;
                        predecessors[edge.dest] = current_vertex;
                        pq.push({new_distance, edge.dest});
                    }
                }
            }
        }
        
        // Reconstruct path from destination to source
        std::vector<T> path;
        T current = destination;
        
        // If no path exists
        if (distances[destination] == std::numeric_limits<W>::max()) {
            return path; // Return empty vector
        }
        
        // Build path backwards
        while (current != source) {
            path.push_back(current);
            if (!predecessors.count(current)) {
                return std::vector<T>(); // No path found
            }
            current = predecessors[current];
        }
        path.push_back(source);
        
        // Reverse to get path from source to destination
        std::reverse(path.begin(), path.end());
        
        return path;
    }

    // Get shortest paths from source to all other vertices using Dijkstra
    std::unordered_map<T, std::pair<W, std::vector<T>>> shortest_paths_from_source(const T& source) const {
        if (!adj_list.count(source)) {
            return {}; // Return empty if source doesn't exist
        }

        std::unordered_map<T, W> distances;
        std::unordered_map<T, T> predecessors;
        std::unordered_set<T> visited;
        
        // Initialize distances
        for (const auto& pair : adj_list) {
            distances[pair.first] = std::numeric_limits<W>::max();
        }
        
        distances[source] = W(0);
        
        // Priority queue: pair<distance, vertex>
        std::priority_queue<std::pair<W, T>,
                        std::vector<std::pair<W, T>>, 
                        std::greater<std::pair<W, T>>> pq;
        
        pq.push({W(0), source});
        
        while (!pq.empty()) {
            auto current_pair = pq.top();
            pq.pop();
            
            W current_dist = current_pair.first;
            T current_vertex = current_pair.second;
            
            if (visited.count(current_vertex)) {
                continue;
            }
            
            visited.insert(current_vertex);
            
            // Check all neighbors (don't stop at any specific destination)
            for (const auto& edge : adj_list.at(current_vertex)) {
                if (!visited.count(edge.dest)) {
                    W new_distance = current_dist + edge.weight;
                    
                    if (new_distance < distances[edge.dest]) {
                        distances[edge.dest] = new_distance;
                        predecessors[edge.dest] = current_vertex;
                        pq.push({new_distance, edge.dest});
                    }
                }
            }
        }
        
        // Build result: ( destination -> {distance, path} )
        std::unordered_map<T, std::pair<W, std::vector<T>>> result;
        
        for (const auto& pair : distances) {
            T destination = pair.first;
            W distance = pair.second;
            
            if (destination == source) {
                result[destination] = {W(0), {source}};
                continue;
            }
            
            // If unreachable
            if (distance == std::numeric_limits<W>::max()) {
                result[destination] = {distance, {}};
                continue;
            }
            
            // Reconstruct path
            std::vector<T> path;
            T current = destination;
            
            while (current != source) {
                path.push_back(current);
                if (!predecessors.count(current)) {
                    path.clear(); // No path found
                    break;
                }
                current = predecessors[current];
            }
            
            if (!path.empty()) {
                path.push_back(source);
                std::reverse(path.begin(), path.end());
            }
            
            result[destination] = {distance, path};
        }
        
        return result;
    }
    
    size_t num_vertices() const { return adj_list.size(); }
    size_t num_edges() const { return edge_count; }
    bool is_empty() const { return adj_list.empty(); }
    
    // Display the graph (for debugging)
    void print_graph() const {
        for (const auto& pair : adj_list) {
            std::cout << pair.first << " -> ";
            for (const auto& edge : pair.second) {
                std::cout << "(" << edge.dest << ", w:" << edge.weight << ") ";
            }
            std::cout << std::endl;
        }
    }
};