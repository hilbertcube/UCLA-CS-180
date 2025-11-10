#include "include/headers.hpp"

// Input: [7, 1, 4, 10, 15]
// Output: [4, 13]

vector<int> stations_positions(vector<int>& houses) {
    if (houses.empty()) return {};

    // Step 1: Sort the positions
    sort(houses.begin(), houses.end());

    vector<int> towers;
    int i = 0;
    int n = houses.size();

    while(i < n) {
        // Step 1: start from the leftmost uncovered house
        int p = houses[i];

         // Step 2: place tower as far right as possible (4 miles east of the first uncovered house)
        int tower_pos = p + 4;
        towers.push_back(tower_pos);

        // Step 3: skip all houses covered by this tower (within 4 miles)
        while (i < n && houses[i] <= tower_pos + 4)
            i++;
    }

    return towers;
}

int main() {
    vector<int> input = {7, 1, 4, 10, 15};
    cout << stations_positions(input) << endl;

    // Run: g++ house_problem.cpp -o bin/house_problem.exe; ./bin/house_problem.exe
    return 0;
}