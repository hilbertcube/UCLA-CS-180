#include "ds/headers.hpp"

bool stairway_search(vector<vector<int>> mat, int target) {
    int n = mat.size();
    int i = 0;
    int j = n - 1;

    while(i < n && j >= 0) {
        if(mat[i][j] == target) return true;
        else if(mat[i][j] > target) j--;
        else i++;
    }
    return false;
}

int main() {
    vector<vector<int>> mat = 
    {{3, 30, 38},
     {20, 52, 54},
     {35, 60, 69}};
     
    print_bool(stairway_search(mat, 35));
    return 0;
}