#include "ds/headers.hpp"


int optimalGame(const vector<int>& v) {
    int n = v.size();
    vector<vector<int>> dp(n, vector<int>(n, 0));

    for(int i = 0; i < n; i++)
        dp[i][i] = v[i];

    for(int i = 0; i + 1 < n; i++)
        dp[i][i+1] = max(v[i], v[i+1]);

    for(int len = 3; len <= n; len++) { // O(n)
        for(int i = 0; i < n - len + 1; i++) { // O(n)
            int j = i + len - 1;

            int leftA  = (i+2 <= j)   ? dp[i+2][j]   : 0;
            int leftB  = (i+1 <= j-1) ? dp[i+1][j-1] : 0;
            int takeLeft = v[i] + min(leftA, leftB);

            int rightA = (i <= j-2)   ? dp[i][j-2]   : 0;
            int rightB = (i+1 <= j-1) ? dp[i+1][j-1] : 0;
            int takeRight = v[j] + min(rightA, rightB);

            dp[i][j] = max(takeLeft, takeRight);
        }
    }

    return dp[0][n-1];
}

int main() {
    vector<int> test1 = {5, 3, 7, 10};
    vector<int> test2 = {8, 15, 3, 7};

    cout << "Example 1 result: " << optimalGame(test1) << endl;
    cout << "Example 2 result: " << optimalGame(test2) << endl;

    return 0;
}

// .\scripts\run.ps1 -p hw5_p6