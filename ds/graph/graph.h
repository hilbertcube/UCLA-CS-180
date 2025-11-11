#pragma once

#include "../../include/headers.hpp"
#include "union_find.h"

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
    
    // Helper function for cycle detection using DFS
    bool has_cycle_directed_helper(const T& vertex,
                                   std::unordered_set<T>& visited,
                                   std::unordered_set<T>& rec_stack) const {
        visited.insert(vertex);
        rec_stack.insert(vertex);
        
        if (adj_list.count(vertex)) {
            for (const auto& edge : adj_list.at(vertex)) {
                if (!visited.count(edge.dest)) {
                    if (has_cycle_directed_helper(edge.dest, visited, rec_stack)) {
                        return true;
                    }
                } else if (rec_stack.count(edge.dest)) {
                    return true;
                }
            }
        }
        
        rec_stack.erase(vertex);
        return false;
    }
    
    // Helper function for cycle detection in undirected graph
    bool has_cycle_undirected_helper(const T& vertex,
                                    const T& parent,
                                    std::unordered_set<T>& visited) const {
        visited.insert(vertex);
        
        if (adj_list.count(vertex)) {
            for (const auto& edge : adj_list.at(vertex)) {
                if (!visited.count(edge.dest)) {
                    if (has_cycle_undirected_helper(edge.dest, vertex, visited)) {
                        return true;
                    }
                } else if (edge.dest != parent) {
                    return true;
                }
            }
        }
        
        return false;
    }

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
    
    // Check if graph has a cycle
    bool has_cycle() const {
        if (adj_list.empty()) {
            return false;
        }
        
        std::unordered_set<T> visited;
        std::unordered_set<T> rec_stack;
        
        // Try to detect cycle starting from each unvisited vertex
        for (const auto& pair : adj_list) {
            if (!visited.count(pair.first)) {
                if (has_cycle_directed_helper(pair.first, visited, rec_stack)) {
                    return true;
                }
            }
        }
        
        return false;
    }
    
    // MST Edge structure for returning results
    struct MSTEdge {
        T u, v;
        W weight;
        
        MSTEdge(const T& u, const T& v, W w) : u(u), v(v), weight(w) {}
        
        bool operator<(const MSTEdge& other) const {
            return weight < other.weight;
        }
    };
    
private:
    // Get all edges in the graph for Kruskal's algorithm
    std::vector<MSTEdge> get_all_edges() const {
        std::vector<MSTEdge> edges;
        std::set<std::pair<T, T>> added_edges; // To avoid duplicate edges in undirected graph
        
        for (const auto& vertex_pair : adj_list) {
            const T& u = vertex_pair.first;
            for (const auto& edge : vertex_pair.second) {
                const T& v = edge.dest;
                W weight = edge.weight;
                
                // For undirected graphs, avoid adding the same edge twice
                // We'll add the edge only if we haven't seen the reverse edge
                std::pair<T, T> edge_pair = (u < v) ? std::make_pair(u, v) : std::make_pair(v, u);
                
                if (added_edges.find(edge_pair) == added_edges.end()) {
                    edges.emplace_back(u, v, weight);
                    added_edges.insert(edge_pair);
                }
            }
        }
        
        return edges;
    }

public:
    
    // Prim's algorithm for Minimum Spanning Tree
    std::vector<MSTEdge> prim_mst() const {
        if (adj_list.empty()) {
            return {};
        }
        
        std::vector<MSTEdge> mst;
        std::unordered_set<T> in_mst;
        
        // Start with the first vertex
        T start_vertex = adj_list.begin()->first;
        in_mst.insert(start_vertex);
        
        // Priority queue to store edges: {weight, {u, v}}
        std::priority_queue<std::pair<W, std::pair<T, T>>, 
                           std::vector<std::pair<W, std::pair<T, T>>>, 
                           std::greater<std::pair<W, std::pair<T, T>>>> pq;
        
        // Add all edges from start vertex to priority queue
        for (const auto& edge : adj_list.at(start_vertex)) {
            pq.push({edge.weight, {start_vertex, edge.dest}});
        }
        
        while (!pq.empty() && mst.size() < adj_list.size() - 1) {
            auto current = pq.top();
            pq.pop();
            
            W weight = current.first;
            T u = current.second.first;
            T v = current.second.second;
            
            // If both vertices are already in MST, skip this edge
            if (in_mst.count(u) && in_mst.count(v)) {
                continue;
            }
            
            // Add edge to MST
            mst.emplace_back(u, v, weight);
            
            // Add the new vertex to MST
            T new_vertex = in_mst.count(u) ? v : u;
            in_mst.insert(new_vertex);
            
            // Add all edges from new vertex to priority queue
            if (adj_list.count(new_vertex)) {
                for (const auto& edge : adj_list.at(new_vertex)) {
                    if (!in_mst.count(edge.dest)) {
                        pq.push({edge.weight, {new_vertex, edge.dest}});
                    }
                }
            }
        }
        
        return mst;
    }
    
    // Kruskal's algorithm for Minimum Spanning Tree
    std::vector<MSTEdge> kruskal_mst() const {
        if (adj_list.empty()) {
            return {};
        }
        
        std::vector<MSTEdge> mst;
        std::vector<MSTEdge> edges = get_all_edges();
        
        // Sort edges by weight
        std::sort(edges.begin(), edges.end());
        
        // Initialize Union-Find structure
        UnionFind<T> uf;
        for (const auto& vertex_pair : adj_list) {
            uf.make_set(vertex_pair.first);
        }
        
        // Process edges in order of increasing weight
        for (const auto& edge : edges) {
            if (uf.union_sets(edge.u, edge.v)) {
                mst.push_back(edge);
                
                // If we have n-1 edges, we're done (n = number of vertices)
                if (mst.size() == adj_list.size() - 1) {
                    break;
                }
            }
        }
        
        return mst;
    }
    
    // Helper function to print MST results
    void print_mst(const std::vector<MSTEdge>& mst, const std::string& algorithm_name) const {
        std::cout << "\n=== " << algorithm_name << " MST ===" << std::endl;
        
        if (mst.empty()) {
            std::cout << "No MST found (graph might be empty or disconnected)" << std::endl;
            return;
        }
        
        W total_weight = W(0);
        std::cout << "Edges in MST:" << std::endl;
        
        for (const auto& edge : mst) {
            std::cout << edge.u << " -- " << edge.v << " (weight: " << edge.weight << ")" << std::endl;
            total_weight += edge.weight;
        }
        
        std::cout << "Total weight: " << total_weight << std::endl;
        std::cout << "Number of edges: " << mst.size() << std::endl;
    }
    
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