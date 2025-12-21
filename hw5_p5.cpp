#include "ds/headers.hpp"

/*
Find an algorithm to solve the following problem:
- You are given a rod of length n inches, and an array of the prices of all rod lengths. Determine the maximum
value obtainable by cutting up the rod and selling the pieces.
- For example, if the length of the rod is 8 and the values of different pieces are given as follows, then the
maximum obtainable value is 22 (by cutting into pieces of lengths 2 and 6).
*/

int rod_cutting(vector<int>& prices) {
    int n = prices.size();

    vector<int> dp(n + 1, 0);
    
    // Find maximum value for all 
    // rod of length i.
    for (int i=1; i<=n; i++) {
        for (int j=1; j<=i; j++) {
            // stores only the best revenue for length i
            // price of the length at jth position + the max price of the remaining of the rod
            int candidate = prices[j-1] + dp[i-j];

            // d[i] keeps updating as we iterate through j
            // eventually, we will reach the true max for dp[i]
            dp[i] = max(dp[i], candidate);
        }
    }
    return dp[n];
}

pair<int, vector<int>> rod_cutting_with_cuts(vector<int>& prices) {
    int n = prices.size();

    vector<int> dp(n + 1, 0);
    vector<int> cut(n + 1, 0);

    for (int i = 1; i <= n; i++) {
        for (int j = 1; j <= i; j++) {
            int candidate = prices[j - 1] + dp[i - j];
            if (candidate > dp[i]) {
                dp[i] = candidate;
                cut[i] = j;     // remember the first cut
            }
        }
    }

    // reconstruct cuts
    vector<int> cuts;
    int length = n;
    while (length > 0) {
        cuts.push_back(cut[length]);
        length -= cut[length];
    }

    return {dp[n], cuts};
}


int main() {
    vector<int> prices = {1, 5, 8, 9, 10, 17, 17, 20};
    cout << rod_cutting_with_cuts(prices) << endl;
    return 0;
}


// .\scripts\run.ps1 -p hw5_p5