#include "include/headers.hpp"

// P_i = {s_i, f_i}
vector<int> status_check(vector<int>& start_times, vector<int>& finish_times) {
    if(start_times.empty() || finish_times.empty()) return {};
    int n = start_times.size();

    // initialize intervals, O(n)
    vector<pair<int, int>> P;
    for(int i = 0; i < n; ++i)
        P.push_back({start_times[i], finish_times[i]});

    // sort the intervals by the second element, O(nlog n)
    sort(P.begin(), P.end(), [](auto &start_times, auto &finish_times) {
        return start_times.second < finish_times.second;
    });

    vector<int> T;
    int earliest_finish = -1; // (assuming all positive start time, else just use INT_MIN)
    for(auto& interval : P) {
        if(interval.first > earliest_finish) {
            earliest_finish = interval.second;
            T.push_back(earliest_finish);
        }
    }
    return T;
}

int main() {
    vector<int> start_times = {1, 2, 3, 7};
    vector<int> finish_times = {4, 6, 5, 9};
    cout << status_check(start_times, finish_times) << endl;
    
    return 0;
}

// .\scripts\run.ps1 -p hw4_p1