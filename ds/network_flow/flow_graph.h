#pragma once
#include "../graph/graph.h"

// Forward declarations
template <typename T, typename W>
class FlowGraph;

template <typename T, typename W>
std::pair<W, std::vector<T>> find_max_bottleneck_path(FlowGraph<T, W>& fg, const T& s, const T& t);

// Flow Graph class that wraps Graph<T, W> to handle flow networks
template <typename T, typename W = int>
class FlowGraph {
private:
    Graph<T, W> graph;
    std::unordered_map<T, std::unordered_map<T, W>> capacity;
    std::unordered_map<T, std::unordered_map<T, W>> flow;
    
    friend std::pair<W, std::vector<T>> find_max_bottleneck_path<T, W>(FlowGraph<T, W>& fg, const T& s, const T& t);
public:
    FlowGraph() {}
    
    // Add a flow edge with capacity
    void add_edge(const T& from, const T& to, W cap) {
        graph.add_directed_edge(from, to, cap);
        capacity[from][to] = cap;
        capacity[to][from] = 0; // Reverse edge with 0 initial capacity
        flow[from][to] = 0;
        flow[to][from] = 0;
    }
    
    // Get residual capacity of an edge
    W get_residual_capacity(const T& from, const T& to) const {
        auto it1 = capacity.find(from);
        if (it1 == capacity.end()) return 0;
        
        auto it2 = it1->second.find(to);
        if (it2 == it1->second.end()) return 0;
        
        W cap = it2->second;
        W used = 0;
        
        auto flow_it1 = flow.find(from);
        if (flow_it1 != flow.end()) {
            auto flow_it2 = flow_it1->second.find(to);
            if (flow_it2 != flow_it1->second.end()) {
                used = flow_it2->second;
            }
        }
        
        return cap - used;
    }
    
    // DFS to find augmenting path and return bottleneck flow (Ford-Fulkerson)
    W dfs(const T& u, const T& sink, W min_flow, 
          std::unordered_set<T>& visited, 
          std::unordered_map<T, T>& parent) {
        
        if (u == sink) {
            return min_flow;
        }
        
        visited.insert(u);
        
        // Get neighbors
        std::vector<T> neighbors = graph.neighbors_of(u);
        
        // Also check reverse edges
        for (const auto& pair : capacity) {
            for (const auto& edge : pair.second) {
                if (edge.first == u && get_residual_capacity(pair.first, u) > 0) {
                    bool already_neighbor = std::find(neighbors.begin(), neighbors.end(), pair.first) != neighbors.end();
                    if (!already_neighbor) {
                        neighbors.push_back(pair.first);
                    }
                }
            }
        }
        
        for (const T& v : neighbors) {
            W residual = get_residual_capacity(u, v);
            
            if (visited.find(v) == visited.end() && residual > 0) {
                parent[v] = u;
                W flow = dfs(v, sink, std::min(min_flow, residual), visited, parent);
                
                if (flow > 0) {
                    return flow;
                }
            }
        }
        
        return 0;
    }
    
    // BFS to find augmenting path and return bottleneck flow (Edmonds-Karp)
    W bfs(const T& source, const T& sink, std::unordered_map<T, T>& parent) {
        parent.clear();
        std::unordered_set<T> visited;
        std::queue<std::pair<T, W>> q;
        
        q.push({source, std::numeric_limits<W>::max()});
        visited.insert(source);
        parent[source] = source;
        
        while (!q.empty()) {
            T u = q.front().first;
            W min_flow = q.front().second;
            q.pop();
            
            // Get neighbors
            std::vector<T> neighbors = graph.neighbors_of(u);
            
            // Also check reverse edges
            for (const auto& pair : capacity) {
                for (const auto& edge : pair.second) {
                    if (edge.first == u && get_residual_capacity(pair.first, u) > 0) {
                        bool already_neighbor = std::find(neighbors.begin(), neighbors.end(), pair.first) != neighbors.end();
                        if (!already_neighbor) {
                            neighbors.push_back(pair.first);
                        }
                    }
                }
            }
            
            for (const T& v : neighbors) {
                W residual = get_residual_capacity(u, v);
                
                if (visited.find(v) == visited.end() && residual > 0) {
                    visited.insert(v);
                    parent[v] = u;
                    W new_flow = std::min(min_flow, residual);
                    
                    if (v == sink) {
                        return new_flow;
                    }
                    
                    q.push({v, new_flow});
                }
            }
        }
        
        return 0;
    }
    
    // Compute maximum flow using Ford-Fulkerson algorithm (DFS-based)
    W ford_fulkerson(const T& source, const T& sink) {
        std::unordered_map<T, T> parent;
        W total_flow = 0;
        
        while (true) {
            std::unordered_set<T> visited;
            parent.clear();
            parent[source] = source;
            
            W path_flow = dfs(source, sink, std::numeric_limits<W>::max(), visited, parent);
            
            if (path_flow == 0) {
                break;
            }
            
            total_flow += path_flow;
            
            // Update flows along the path
            T current = sink;
            while (current != source) {
                T prev = parent[current];
                flow[prev][current] += path_flow;
                flow[current][prev] -= path_flow;
                current = prev;
            }
        }
        
        return total_flow;
    }
    
    // Compute maximum flow using Edmonds-Karp algorithm (BFS-based)
    W edmonds_karp(const T& source, const T& sink) {
        std::unordered_map<T, T> parent;
        W total_flow = 0;
        
        while (true) {
            W path_flow = bfs(source, sink, parent);
            
            if (path_flow == 0) {
                break;
            }
            
            total_flow += path_flow;
            
            // Update flows along the path
            T current = sink;
            while (current != source) {
                T prev = parent[current];
                flow[prev][current] += path_flow;
                flow[current][prev] -= path_flow;
                current = prev;
            }
        }
        
        return total_flow;
    }
    
    // Alias for backwards compatibility
    W max_flow(const T& source, const T& sink) {
        return edmonds_karp(source, sink);
    }
    
    void print_graph() const {
        graph.print_graph();
    }
};
