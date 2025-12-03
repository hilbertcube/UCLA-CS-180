#pragma once

#include "../../ds/headers.hpp"

// Union-Find (Disjoint Set Union) data structure for Kruskal's algorithm
template<typename T>
class UnionFind {
private:
    std::unordered_map<T, T> parent;
    std::unordered_map<T, int> rank;
    
public:
    void make_set(const T& x) {
        if (parent.find(x) == parent.end()) {
            parent[x] = x;
            rank[x] = 0;
        }
    }
    
    T find(const T& x) {
        if (parent[x] != x) {
            parent[x] = find(parent[x]); // Path compression
        }
        return parent[x];
    }
    
    bool union_sets(const T& x, const T& y) {
        T root_x = find(x);
        T root_y = find(y);
        
        if (root_x == root_y) {
            return false; // Already in the same set (would create cycle)
        }
        
        // Union by rank
        if (rank[root_x] < rank[root_y]) {
            parent[root_x] = root_y;
        } else if (rank[root_x] > rank[root_y]) {
            parent[root_y] = root_x;
        } else {
            parent[root_y] = root_x;
            rank[root_x]++;
        }
        
        return true;
    }
};