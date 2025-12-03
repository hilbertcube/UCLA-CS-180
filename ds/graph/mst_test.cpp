#include "graph.h"
#include "mst.h"

int main() {
    std::cout << "=== MINIMUM SPANNING TREE ALGORITHMS DEMONSTRATION ===" << std::endl;
    
    // Test Case 1: Simple connected graph
    std::cout << "\n--- Test Case 1: Simple Connected Graph ---" << std::endl;
    Graph<char, int> graph1;
    
    // Create a simple weighted undirected graph
    graph1.add_undirected_edge('A', 'B', 4);
    graph1.add_undirected_edge('A', 'C', 2);
    graph1.add_undirected_edge('B', 'C', 1);
    graph1.add_undirected_edge('B', 'D', 5);
    graph1.add_undirected_edge('C', 'D', 8);
    graph1.add_undirected_edge('C', 'E', 10);
    graph1.add_undirected_edge('D', 'E', 2);
    graph1.add_undirected_edge('D', 'F', 6);
    graph1.add_undirected_edge('E', 'F', 3);
    
    std::cout << "Original graph:" << std::endl;
    graph1.print_graph();
    
    auto prim_mst1 = prim_mst(graph1);
    auto kruskal_mst1 = kruskal_mst(graph1);
    
    print_mst(prim_mst1, "PRIM'S");
    print_mst(kruskal_mst1, "KRUSKAL'S");
    
    // Test Case 2: Larger graph with string vertices
    std::cout << "\n--- Test Case 2: Larger String Vertex Graph ---" << std::endl;
    Graph<std::string, double> graph2;
    
    // Create a more complex graph
    graph2.add_undirected_edge("New York", "Boston", 215.0);
    graph2.add_undirected_edge("New York", "Philadelphia", 95.0);
    graph2.add_undirected_edge("New York", "Washington", 230.0);
    graph2.add_undirected_edge("Boston", "Philadelphia", 300.0);
    graph2.add_undirected_edge("Philadelphia", "Washington", 140.0);
    graph2.add_undirected_edge("Washington", "Atlanta", 640.0);
    graph2.add_undirected_edge("Philadelphia", "Atlanta", 780.0);
    graph2.add_undirected_edge("Boston", "Chicago", 980.0);
    graph2.add_undirected_edge("New York", "Chicago", 790.0);
    graph2.add_undirected_edge("Chicago", "Atlanta", 720.0);
    graph2.add_undirected_edge("Chicago", "Denver", 1000.0);
    graph2.add_undirected_edge("Atlanta", "Denver", 1400.0);
    graph2.add_undirected_edge("Denver", "Los Angeles", 1000.0);
    graph2.add_undirected_edge("Chicago", "Los Angeles", 2100.0);
    graph2.add_undirected_edge("Atlanta", "Los Angeles", 2200.0);
    
    std::cout << "City distance graph:" << std::endl;
    graph2.print_graph();
    
    auto prim_mst2 = prim_mst(graph2);
    auto kruskal_mst2 = kruskal_mst(graph2);
    
    print_mst(prim_mst2, "PRIM'S");
    print_mst(kruskal_mst2, "KRUSKAL'S");
    
    // Test Case 3: Triangle graph (minimum case)
    std::cout << "\n--- Test Case 3: Triangle Graph ---" << std::endl;
    Graph<int, int> graph3;
    
    graph3.add_undirected_edge(1, 2, 3);
    graph3.add_undirected_edge(2, 3, 1);
    graph3.add_undirected_edge(1, 3, 2);
    
    std::cout << "Triangle graph:" << std::endl;
    graph3.print_graph();
    
    auto prim_mst3 = prim_mst(graph3);
    auto kruskal_mst3 = kruskal_mst(graph3);
    
    print_mst(prim_mst3, "PRIM'S");
    print_mst(kruskal_mst3, "KRUSKAL'S");
    
    // Test Case 4: Single vertex (edge case)
    std::cout << "\n--- Test Case 4: Single Vertex ---" << std::endl;
    Graph<char, int> graph4;
    graph4.add_vertex('A');
    
    std::cout << "Single vertex graph:" << std::endl;
    graph4.print_graph();
    
    auto prim_mst4 = prim_mst(graph4);
    auto kruskal_mst4 = kruskal_mst(graph4);
    
    print_mst(prim_mst4, "PRIM'S");
    print_mst(kruskal_mst4, "KRUSKAL'S");
    
    // Test Case 5: Comparison with identical weights
    std::cout << "\n--- Test Case 5: Graph with Identical Weights ---" << std::endl;
    Graph<char, int> graph5;
    
    graph5.add_undirected_edge('A', 'B', 1);
    graph5.add_undirected_edge('B', 'C', 1);
    graph5.add_undirected_edge('C', 'D', 1);
    graph5.add_undirected_edge('A', 'D', 1);
    graph5.add_undirected_edge('A', 'C', 1);
    graph5.add_undirected_edge('B', 'D', 1);
    
    std::cout << "Graph with identical weights:" << std::endl;
    graph5.print_graph();
    
    auto prim_mst5 = prim_mst(graph5);
    auto kruskal_mst5 = kruskal_mst(graph5);
    
    print_mst(prim_mst5, "PRIM'S");
    print_mst(kruskal_mst5, "KRUSKAL'S");
    
    std::cout << "\n=== ALGORITHM COMPARISON ===" << std::endl;
    std::cout << "Both Prim's and Kruskal's algorithms find minimum spanning trees." << std::endl;
    std::cout << "- Prim's: Grows the MST one vertex at a time (greedy vertex selection)" << std::endl;
    std::cout << "- Kruskal's: Processes edges in order of weight (greedy edge selection)" << std::endl;
    std::cout << "- Both have O(E log V) time complexity with efficient implementations" << std::endl;
    std::cout << "- The resulting MST may differ in structure but will have the same total weight" << std::endl;
    
    // Compile and run: g++ mst_test.cpp -o bin/mst_test.exe; ./bin/mst_test.exe
    return 0;
}