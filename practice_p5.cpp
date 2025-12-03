#include "ds/headers.hpp"

vector<int> coins = {1, 2, 5, 10, 20, 50, 100, 500, 1000};

int make_change(int n) {
    int output = 0;
    int i = coins.size() - 1;
    
    while(i >= 0 && n > 0) {
        if(n >= coins[i]) {
            int coins_used = n / coins[i];
            output += coins_used;
            n %= coins[i];
        }
        i--;
    }
    return output;
}

int main() {
    cout << make_change(70) << endl;
    cout << make_change(121) << endl;
    cout << make_change(321) << endl;
    return 0;
}

// .\scripts\run.ps1 -p practice_p5