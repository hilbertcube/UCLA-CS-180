#include "ds/headers.hpp"

int longest_alternating_subsequence_size(const vector<int>& arr)
{
    int n = arr.size();
    if (n == 0) return 0;

    // DP[i][0] and DP[i][1] as before
    vector<vector<int>> DP(n, vector<int>(2, 1));

    int res = 1;

    for (int i = 1; i < n; i++) {
        for (int j = 0; j < i; j++) {

            if (arr[j] < arr[i] && DP[i][0] < DP[j][1] + 1)
                DP[i][0] = DP[j][1] + 1;

            if (arr[j] > arr[i] && DP[i][1] < DP[j][0] + 1)
                DP[i][1] = DP[j][0] + 1;
        }

        res = max(res, max(DP[i][0], DP[i][1]));
        cout << DP << endl;
    }
    
    return res;
}

vector<int> longest_alternating_subsequence(const vector<int>& arr)
{
    int n = arr.size();
    if (n == 0) return {};

    // DP[i][0] longest alt subsequence ending at i where arr[i] > previous
    // DP[i][1] longest alt subsequence ending at i where arr[i] < previous
    // Create a vector with n elements
    // Each element is a vector of size 2, both entries initialized to 1
    vector<vector<int>> DP(n, vector<int>(2, 1));

    // Parent indices for reconstruction
    vector<vector<int>> parent(n, vector<int>(2, -1));

    int best_len = 1;
    int best_i = 0;
    int best_state = 0;

    for (int i = 1; i < n; i++) {
        for (int j = 0; j < i; j++) {

            if (arr[j] < arr[i] && DP[i][0] < DP[j][1] + 1) {
                DP[i][0] = DP[j][1] + 1;
                parent[i][0] = j;
            }

            if (arr[j] > arr[i] && DP[i][1] < DP[j][0] + 1) {
                DP[i][1] = DP[j][0] + 1;
                parent[i][1] = j;
            }
        }

        for (int s = 0; s < 2; s++) {
            if (DP[i][s] > best_len) {
                best_len = DP[i][s];
                best_i = i;
                best_state = s;
            }
        }
    }

    // Reconstruct sequence
    vector<int> output;
    int idx = best_i;
    int state = best_state;

    while (idx != -1) {
        output.push_back(arr[idx]);
        int next = parent[idx][state];
        state = 1 - state;
        idx = next;
    }

    reverse(output.begin(), output.end());
    return output;
}


vector<int> longest_alternating_subsequence_greedy(const vector<int>& arr) {
    if (arr.empty()) return {};

    vector<int> output;
    output.push_back(arr[0]);

    int direction = 0;  // 0 unknown, +1 increasing, -1 decreasing

    // start with second element
    for (int i = 1; i < arr.size(); i++) {
        int x = arr[i];

        // if only 1 element so far: Add x if different, set direction based on comparison
        if (output.size() == 1) {
            if (x != output.back()) {
                direction = (x > output.back()) ? 1 : -1;
                output.push_back(x);
            }
        } else {
            int last = output.back();

            if (direction == 1 && x < last) {
                direction = -1;
                output.push_back(x);
            } else if (direction == -1 && x > last) {
                direction = 1;
                output.push_back(x);
            } else {
                output.back() = x;
            }
        }
    }

    return output;
}

int main() {
    vector<int> input_1 = {10, 22, 9, 33, 49, 50, 31, 60};
    cout << longest_alternating_subsequence_greedy(input_1) << endl;

    vector<int> input_2 = {10, 9, 1, 2, 1, 2, 17};
    cout << longest_alternating_subsequence_greedy(input_2) << endl;

    // output: [10, 22, 9, 33, 31, 60]

    return 0;
}

// .\scripts\run.ps1 -p hw6_p6