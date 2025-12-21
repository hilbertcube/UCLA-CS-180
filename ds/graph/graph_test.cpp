#include "graph.h"

int main() {
    Graph<std::string, int> graph;

    // Add more vertices for comprehensive testing
    graph.add_vertex("A");
    graph.add_vertex("B");
    graph.add_vertex("C");
    graph.add_vertex("D");
    graph.add_vertex("E");
    graph.add_vertex("F");
    graph.add_vertex("G");
    graph.add_vertex("H");

    // Add edges with varied weights to demonstrate Dijkstra's algorithm
    // Create a more complex graph with different path costs
    graph.add_directed_edge("A", "B", 4);  // A -> B with weight 4
    graph.add_directed_edge("A", "C", 2);  // A -> C with weight 2
    graph.add_directed_edge("B", "C", 1);  // B -> C with weight 1
    graph.add_directed_edge("B", "D", 5);  // B -> D with weight 5
    graph.add_directed_edge("B", "E", 10); // B -> E with weight 10
    graph.add_directed_edge("C", "D", 8);  // C -> D with weight 8
    graph.add_directed_edge("C", "E", 3);  // C -> E with weight 3
    graph.add_directed_edge("D", "F", 2);  // D -> F with weight 2
    graph.add_directed_edge("E", "F", 4);  // E -> F with weight 4
    graph.add_directed_edge("E", "G", 2);  // E -> G with weight 2
    graph.add_directed_edge("F", "H", 3);  // F -> H with weight 3
    graph.add_directed_edge("G", "H", 1);  // G -> H with weight 1
    
    // Add some bidirectional edges to make it more interesting
    graph.add_directed_edge("C", "A", 2);  // Reverse edge C -> A
    graph.add_directed_edge("D", "B", 5);  // Reverse edge D -> B
    graph.add_directed_edge("F", "D", 2);  // Reverse edge F -> D

    std::cout << "=== GRAPH STRUCTURE ===" << std::endl;
    graph.print_graph();

    std::cout << "\nNeighbor of B: " << graph.neighbors_of("B") << std::endl;

    std::cout << "\n=== GRAPH TRAVERSALS ===" << std::endl;
    std::cout << "BFS from A: " << graph.bfs("A") << std::endl;
    std::cout << "DFS from A: " << graph.dfs_iterative("A") << std::endl;
    std::cout << "Topological Sort: " << graph.topological_sort() << std::endl;

    std::cout << "\n=== DIJKSTRA'S SHORTEST PATH DEMONSTRATIONS ===" << std::endl;
    
    // Test multiple shortest path scenarios
    auto print_path = [](const std::vector<std::string>& path, const std::string& from, const std::string& to) {
        if (path.empty()) {
            std::cout << "No path from " << from << " to " << to << std::endl;
        } else {
            std::cout << "Shortest path from " << from << " to " << to << ": ";
            for (size_t i = 0; i < path.size(); ++i) {
                std::cout << path[i];
                if (i < path.size() - 1) std::cout << " -> ";
            }
            std::cout << std::endl;
        }
    };

    // Demonstrate various shortest paths
    print_path(graph.shortest_path_Dijkstra("A", "H"), "A", "H");
    print_path(graph.shortest_path_Dijkstra("A", "F"), "A", "F");
    print_path(graph.shortest_path_Dijkstra("A", "G"), "A", "G");
    print_path(graph.shortest_path_Dijkstra("B", "H"), "B", "H");
    print_path(graph.shortest_path_Dijkstra("C", "H"), "C", "H");
    print_path(graph.shortest_path_Dijkstra("A", "E"), "A", "E");
    
    // Test paths that don't exist or are unreachable
    print_path(graph.shortest_path_Dijkstra("H", "A"), "H", "A");
    print_path(graph.shortest_path_Dijkstra("G", "A"), "G", "A");

    std::cout << "\n=== TESTING AFTER VERTEX REMOVAL ===" << std::endl;
    graph.remove_vertex("B");
    std::cout << "After removing vertex B:" << std::endl;
    graph.print_graph();
    
    // Test paths after vertex removal
    std::cout << "\nPaths after removing B:" << std::endl;
    print_path(graph.shortest_path_Dijkstra("A", "H"), "A", "H");
    print_path(graph.shortest_path_Dijkstra("A", "F"), "A", "F");

    // Run: g++ test.cpp -o bin/test.exe; ./bin/test.exe
    return 0;
}