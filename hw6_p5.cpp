#include "ds/headers.hpp"
#include "ds/network_flow/flow_graph.h"

template <typename T, typename W>
pair<W, vector<T>> find_max_bottleneck_path(FlowGraph<T, W>& fg, const T& s, const T& t) {
    // Initialization
    unordered_map<T, W> width;
    unordered_map<T, T> parent;
    
    // Max-Priority Queue: stores {current_width, vertex}
    priority_queue<pair<W, T>> pq;

    // Set width to -infinity (0 for positive capacities) initially
    // Set source width to +infinity
    width[s] = numeric_limits<W>::max();
    parent[s] = s;
    pq.push({width[s], s});

    while (!pq.empty()) {
        W current_width = pq.top().first;
        T u = pq.top().second;
        pq.pop();

        // Optimization: If we extracted the target, we are done
        if (u == t) break;

        // Lazy deletion check
        if (current_width < width[u]) continue;

        // DIRECT ACCESS to private member 'graph'
        vector<T> neighbors = fg.graph.neighbors_of(u);

        for (const T& v : neighbors) {
            if (fg.capacity.find(u) == fg.capacity.end() || 
                fg.capacity[u].find(v) == fg.capacity[u].end()) continue;

            W edge_cap = fg.capacity[u][v];
            if (edge_cap == 0) continue; 

            // Calculate candidate bottleneck
            W cand = min(current_width, edge_cap);

            // Relaxation
            if (cand > width[v]) {
                width[v] = cand;
                parent[v] = u;
                pq.push({width[v], v});
            }
        }
    }

    // Path Reconstruction
    vector<T> path;
    
    if (width.find(t) == width.end() || width[t] == 0) {
        return {0, path}; // Target not reachable
    }

    T curr = t;
    path.push_back(curr);
    while (curr != s) {
        curr = parent[curr];
        path.push_back(curr);
    }
    reverse(path.begin(), path.end());

    return {width[t], path};
}

int main() {
    // 1. Create the FlowGraph instance
    // Using std::string for vertex labels, int for weights
    FlowGraph<std::string, int> network;

    std::cout << "Building graph..." << std::endl;

    // 2. Build the graph
    // Source 'S', Sink 'T'
    
    // Path 1: "Short but Narrow" (Bottleneck = 10)
    network.add_edge("S", "A1", 10);
    network.add_edge("A1", "T", 10);

    // Path 2: "Wide start, Narrow end" (Bottleneck = 5)
    network.add_edge("S", "B1", 20);
    network.add_edge("B1", "T", 5);

    // Path 3: "Long but Wide" (Bottleneck = 15) -> This should be the winner
    network.add_edge("S", "C1", 15);
    network.add_edge("C1", "C2", 15);
    network.add_edge("C2", "T", 15);

    // 3. Run the algorithm
    std::string source = "S";
    std::string sink = "T";

    std::cout << "Finding max bottleneck path from " << source << " to " << sink << "..." << std::endl;

    // Call the friend function
    std::pair<int, std::vector<std::string>> result = find_max_bottleneck_path(network, source, sink);
    
    int max_width = result.first;
    std::vector<std::string> path = result.second;

    // 4. Output results
    if (path.empty()) {
        std::cout << "No path exists between " << source << " and " << sink << "." << std::endl;
    } else {
        std::cout << "------------------------------------------------" << std::endl;
        std::cout << "Max Bottleneck Width: " << max_width << std::endl;
        std::cout << "Path Sequence: ";
        
        for (size_t i = 0; i < path.size(); ++i) {
            std::cout << path[i];
            if (i != path.size() - 1) std::cout << " -> ";
        }
        std::cout << std::endl;
        std::cout << "------------------------------------------------" << std::endl;

        // Validation logic
        if (max_width == 15 && path.size() == 4) {
            std::cout << "TEST PASSED: Correctly identified the widest path." << std::endl;
        } else {
            std::cout << "TEST FAILED: Expected width 15, got " << max_width << std::endl;
        }
    }

    return 0;
}

