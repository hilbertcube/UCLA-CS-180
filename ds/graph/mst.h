#pragma once

#include "../../ds/headers.hpp"
#include "union_find.h"

// Forward declaration
template <typename T, typename W> class Graph;

// MST Edge structure
template <typename T, typename W = int>
struct MSTEdge {
    T u, v;
    W weight;
    
    MSTEdge(const T& u, const T& v, W w) : u(u), v(v), weight(w) {}
    
    bool operator<(const MSTEdge& other) const {
        return weight < other.weight;
    }
};

// Prim's algorithm for Minimum Spanning Tree
template <typename T, typename W = int>
std::vector<MSTEdge<T, W>> prim_mst(const Graph<T, W>& graph) {
    if (graph.adj_list.empty()) {
        return {};
    }
    
    std::vector<MSTEdge<T, W>> mst;
    std::unordered_set<T> in_mst;
    
    // Start with the first vertex
    T start_vertex = graph.adj_list.begin()->first;
    in_mst.insert(start_vertex);
    
    // Priority queue to store edges: {weight, {u, v}}
    std::priority_queue<std::pair<W, std::pair<T, T>>, 
                       std::vector<std::pair<W, std::pair<T, T>>>, 
                       std::greater<std::pair<W, std::pair<T, T>>>> pq;
    
    // Add all edges from start vertex to priority queue
    for (const auto& edge : graph.adj_list.at(start_vertex)) {
        pq.push({edge.weight, {start_vertex, edge.dest}});
    }
    
    while (!pq.empty() && mst.size() < graph.adj_list.size() - 1) {
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
        if (graph.adj_list.count(new_vertex)) {
            for (const auto& edge : graph.adj_list.at(new_vertex)) {
                if (!in_mst.count(edge.dest)) {
                    pq.push({edge.weight, {new_vertex, edge.dest}});
                }
            }
        }
    }
    
    return mst;
}

// Kruskal's algorithm for Minimum Spanning Tree
template <typename T, typename W = int>
std::vector<MSTEdge<T, W>> kruskal_mst(const Graph<T, W>& graph) {
    if (graph.adj_list.empty()) {
        return {};
    }
    
    std::vector<MSTEdge<T, W>> mst;
    std::vector<MSTEdge<T, W>> edges;
    std::set<std::pair<T, T>> added_edges; // To avoid duplicate edges in undirected graph
    
    // Get all edges from the graph
    for (const auto& vertex_pair : graph.adj_list) {
        const T& u = vertex_pair.first;
        for (const auto& edge : vertex_pair.second) {
            const T& v = edge.dest;
            W weight = edge.weight;
            
            // For undirected graphs, avoid adding the same edge twice
            std::pair<T, T> edge_pair = (u < v) ? std::make_pair(u, v) : std::make_pair(v, u);
            
            if (added_edges.find(edge_pair) == added_edges.end()) {
                edges.emplace_back(u, v, weight);
                added_edges.insert(edge_pair);
            }
        }
    }
    
    // Sort edges by weight
    std::sort(edges.begin(), edges.end());
    
    // Initialize Union-Find structure
    UnionFind<T> uf;
    for (const auto& vertex_pair : graph.adj_list) {
        uf.make_set(vertex_pair.first);
    }
    
    // Process edges in order of increasing weight
    for (const auto& edge : edges) {
        if (uf.union_sets(edge.u, edge.v)) {
            mst.push_back(edge);
            
            // If we have n-1 edges, we're done (n = number of vertices)
            if (mst.size() == graph.adj_list.size() - 1) {
                break;
            }
        }
    }
    
    return mst;
}

// Helper function to print MST results
template <typename T, typename W>
void print_mst(const std::vector<MSTEdge<T, W>>& mst, const std::string& algorithm_name) {
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
