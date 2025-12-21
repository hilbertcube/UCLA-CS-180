#include "ds/headers.hpp"

vector<vector<int>> floodFill(vector<vector<int>>& image, int sr, int sc, int color) {
    int origColor = image[sr][sc];
    if (origColor == color) return image; // No change needed

    int rows = image.size();
    int cols = image[0].size();
    queue<pair<int, int>> q;
    q.push({sr, sc});

    // 4-directional movement: down, up, right, left
    int directions[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

    while (!q.empty()) {
        auto [r, c] = q.front(); q.pop(); // structure binding
        /* same as
        auto cell = q.front();
        int r = cell.first;
        int c = cell.second;
        */

        // Recolor the current pixel
        if (image[r][c] == origColor) {
            image[r][c] = color;

            // Check all 4 neighbors
            for (auto& dir : directions) {
                int nr = r + dir[0];
                int nc = c + dir[1];

                // Within bounds and still the original color
                if (nr >= 0 && nr < rows && nc >= 0 && nc < cols && image[nr][nc] == origColor) {
                    q.push({nr, nc});
                }
            }
        }
    }
    return image;
}

int main() {
    vector<vector<int>> image = 
    {{1, 1, 1, 0},
     {0, 1, 1, 1},
     {1, 0, 1, 1}};

    cout << floodFill(image, 1, 2, 2);
    return 0;
}