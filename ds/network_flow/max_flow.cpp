#include "../headers.hpp"
#include "../graph/graph.h"
#include "flow_graph.h"

int main() {
    // Create a flow graph with integer vertices
    FlowGraph<int, int> fg;

    // Add edges with capacities (same as before)
    fg.add_edge(0, 1, 16);
    fg.add_edge(0, 2, 13);
    fg.add_edge(1, 2, 10);
    fg.add_edge(2, 1, 4);
    fg.add_edge(1, 3, 12);
    fg.add_edge(2, 4, 14);
    fg.add_edge(3, 2, 9);
    fg.add_edge(4, 3, 7);
    fg.add_edge(3, 5, 20);
    fg.add_edge(4, 5, 4);

    int source = 0;
    int sink = 5;

    std::cout << "Flow Network:" << std::endl;
    fg.print_graph();
    std::cout << std::endl;

    int result = fg.max_flow(source, sink);
    std::cout << "Maximum flow from " << source << " to " << sink << ": " << result << std::endl;

    return 0;
}

// Compile and run: g++ max_flow.cpp -o bin/max_flow.exe; ./bin/max_flow.exe
