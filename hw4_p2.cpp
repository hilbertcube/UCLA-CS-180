#include "ds/headers.hpp"

struct Edge {
    int to;
    function<double(double)> f; // time-dependent travel function
};

struct Node {
    int vertex;
    double time;
    bool operator>(const Node& other) const {
        return time > other.time;
    }
};

vector<double> time_dependent_dijkstra(vector<vector<Edge>>& graph, int source, int target
) {
    int n = graph.size();
    const double INF = std::numeric_limits<double>::max();

    vector<double> A(n, INF);         // Earliest arrival times
    vector<bool> Finalized(n, false); // Boolean array, not a set

    A[source] = 0.0;

    priority_queue<Node, vector<Node>, greater<Node>> PQ;
    PQ.push({source, 0.0});

    while (!PQ.empty()) {
        Node curr = PQ.top(); PQ.pop();
        int u = curr.vertex;

        if (Finalized[u]) continue; // already finalized
        Finalized[u] = true;

        if (u == target) break;

        for (auto& e : graph[u]) {
            double arrival = e.f(A[u]);
            if (arrival < A[e.to]) {
                A[e.to] = arrival;
                PQ.push({e.to, arrival});
            }
        }
    }

    return A;
}

int main() {
    int n = 4;
    vector<vector<Edge>> G(n);

    G[0].push_back({1, [](double t){ return t + 5; }});
    G[0].push_back({2, [](double t){ return t + 10; }});
    G[1].push_back({2, [](double t){ return t + 2; }});
    G[1].push_back({3, [](double t){ return t + 6; }});
    G[2].push_back({3, [](double t){ return t + 3; }});

    int s = 0, t = 3;

    vector<double> A = time_dependent_dijkstra(G, s, t);

    cout << fixed << setprecision(2);
    cout << "Earliest arrival times:\n";
    for (int i = 0; i < n; ++i) {
        cout << "Node " << i << ": ";
        if (A[i] == 1e18)
            cout << "unreachable\n";
        else
            cout << A[i] << "\n";
    }

    cout << "\nEarliest arrival at target " << t << " = " << A[t] << "\n";
    // Run: g++ p1.cpp -o bin/p1.exe; ./bin/p1.exe
    return 0;
}