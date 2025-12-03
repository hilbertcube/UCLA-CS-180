#include "ds/headers.hpp"

vector<vector<int>> distinct_pair(vector<int>& arr, int k) {
    int n = arr.size();
    vector<vector<int>> output;

    int i = 0;
    int j = 1;

    while(i < n && j < n) {
        int diff = arr[j] - arr[i];

        // if diff < k, it means the next value can potentially be k
        if(diff < k) {
            j++;
        } else if(diff > k) {
            i++;
            if(j <= i) j = i + 1;
        } else {
            // avoid duplicates
            if(i == 0 || arr[i] != arr[i-1])
                output.push_back({arr[i], arr[j]});
            i++;
            if(j <= i) j = i + 1;
        }
    }
    return output;
}

int main() {
    vector<int> test_1 = {1, 2, 2, 2, 4, 5, 5, 5};
    int k = 3;
    cout << distinct_pair(test_1, k) << endl;

    vector<int> test_2 = {1, 1, 2, 2, 3, 4, 5, 6, 8, 9};
    cout << distinct_pair(test_2, 3) << endl;
    return 0;
}


// .\scripts\run.ps1 -p hw5_p1