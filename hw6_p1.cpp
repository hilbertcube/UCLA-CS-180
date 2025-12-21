#include "ds/headers.hpp"

// Structure to hold precinct data
struct Precinct {
    int id;
    int votesA;
    int votesB;
};

// Function to run the DP algorithm for a specific party's votes
// Corresponds to Step 3 (Main loop) in the image solution
bool canPartyGerrymander(int n, int m, const vector<int>& votes, const string& partyName) {
    int k = n / 2; // We need to choose exactly n/2 precincts
    int totalVotes = accumulate(votes.begin(), votes.end(), 0);
    
    // Threshold calculation: T = (n * m) / 4
    // A party needs strictly > T votes to win a district.
    double T = (double)(n * m) / 4.0;

    // Step 3b: Early exit check
    // If the party doesn't have enough total votes to win two districts theoretically
    // i.e., TotalP <= 2 * T, then it's impossible.
    if (totalVotes <= (n * m) / 2) {
        cout << "Party " << partyName << " does not have enough total votes to dominate both districts." << endl;
        return false;
    }

    // Step 3c: Initialize DP table
    // dp[j][s] will be true if we can choose exactly 'j' precincts that sum to 's' votes
    // Max possible sum for a district is (n/2) * m
    int max_district_votes = k * m;
    
    // We use a 2D vector here. 
    // Dimension 1: Number of precincts picked (0 to k)
    // Dimension 2: Sum of votes (0 to max_district_votes)
    vector<vector<bool>> dp(k + 1, vector<bool>(max_district_votes + 1, false));

    // Step 3d: Base case
    // We can choose 0 precincts with a sum of 0.
    dp[0][0] = true;

    // Step 3e: DP Transitions
    // Iterate through every precinct's vote count
    for (int vote_count : votes) {
        // Iterate backwards through the count of precincts (k down to 1)
        // We go backwards to avoid using the same precinct twice for the same count level 
        // (standard knapsack space optimization)
        for (int j = k; j >= 1; --j) {
            // Iterate backwards through possible sums
            for (int s = max_district_votes; s >= vote_count; --s) {
                // If we could form sum (s - vote_count) using (j - 1) precincts,
                // then we can form sum 's' using 'j' precincts by including the current one.
                if (dp[j - 1][s - vote_count]) {
                    dp[j][s] = true;
                }
            }
        }
    }

    // Step 3f: Check the table for a valid configuration
    // We look at the row dp[k] (meaning we selected exactly n/2 precincts)
    for (int s = 0; s <= max_district_votes; ++s) {
        if (dp[k][s]) {
            // Check if this sum 's' allows the party to win District 1
            // AND if the remaining votes (totalVotes - s) allow them to win District 2.
            // Condition: T < s < TotalP - T
            if (s > T && s < (totalVotes - T)) {
                cout << "Found a valid partition for Party " << partyName << "!" << endl;
                cout << "  - District 1 votes: " << s << " (Threshold > " << T << ")" << endl;
                cout << "  - District 2 votes: " << (totalVotes - s) << " (Threshold > " << T << ")" << endl;
                return true; 
            }
        }
    }

    return false;
}

// Main logic to solve the problem
void solveGerrymandering(int n, int m, vector<Precinct>& precincts) {
    cout << "Analyzing for n=" << n << ", m=" << m << "..." << endl;
    
    // 1. Extract votes for Party A
    vector<int> votesA;
    for (const auto& p : precincts) votesA.push_back(p.votesA);

    // Run DP check for Party A
    if (canPartyGerrymander(n, m, votesA, "A")) {
        cout << "Result: Susceptible (favors Party A)" << endl;
        return;
    }

    // 2. Extract votes for Party B
    vector<int> votesB;
    for (const auto& p : precincts) votesB.push_back(p.votesB);

    // Run DP check for Party B
    if (canPartyGerrymander(n, m, votesB, "B")) {
        cout << "Result: Susceptible (favors Party B)" << endl;
        return;
    }

    // Step 4: If neither works
    cout << "Result: Not Susceptible" << endl;
}

int main() {
    // Example from the image
    int n = 4;   // Number of precincts
    int m = 100; // Voters per precinct (55+45 = 100)

    // Data from the table in the image
    vector<Precinct> precincts = {
        {1, 55, 45},
        {2, 43, 57},
        {3, 60, 40},
        {4, 47, 53}
    };

    solveGerrymandering(n, m, precincts);

    return 0;
}

// .\scripts\run.ps1 -p hw6_p1