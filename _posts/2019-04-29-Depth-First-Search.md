---
layout:     post
title:      "Depth-First Search"
date:       2019-4-29
author:     Tong
catalog: true
tags:
    - Algorithm
---

### Summary

1. What is a connection?

2. Who connects whom?

3. From where to start? (given coordinates? one by one? border?)

4. When stop exploring?

5. Maybe set to another value (tag) for the moment? Modify all of them at last according to the tag?


### [Leetcode 130. Surrounded Regions](https://leetcode.com/problems/surrounded-regions/)

#### Question

Given a 2D board containing `'X'` and `'O'` (__the letter O__), capture all regions surrounded by `'X'`.

A region is captured by flipping all `'O'`s into `'X'`s in that surrounded region.

```
X X X X
X O O X
X X O X
X O X X
```

After running your function, the board should be:

```
X X X X
X X X X
X X X X
X O X X
```

__Explanation__<br>
Surrounded regions shouldn’t be on the border, which means that any `'O'` on the border of the board are not flipped to `'X'`. Any `'O'` that is not on the border and it is not connected to an `'O'` on the border will be flipped to `'X'`. Two cells are connected if they are adjacent cells connected horizontally or vertically.

#### Train of Thought
There are two ways.
- Find the cells which need to be flipped.
- Find the cells which don't need to be flipped.

##### 1. Find the cells which need to be flipped.
The cells must meet the following conditions __at the same time__.
- They are originally `'O'`.
- They are __not__ on the border.
- They are __not__ connected to those `'O'` on the border.

##### 2. Find the cells which don't need to be flipped.
The cells only need to meet __one of__ the following conditions.
- They are originally not `'O'`.
- They are on the border.
- They are connected to those `'O'` on the border.

__Important:__ We need to save the information about the cells which need / don't need to be flipped.

##### Discussion
If we use the first way,
1. we need to traverse all neighbors of one cell until we encounter the border. Unless we save all the paths we have been through, we can update only one cell.
2. Apart from this, we also need to somehow save the visited cells, otherwise, it is easy encounter some infinite loops/overflow trouble.

If we use the second way,
1. we can start from the cells on the border. If it does not need to be flipped, that means, its `'O'` neighbors do not need to be flipped, either, so we can continue to find all the cells which do not need to be flipped.
2. While finding them, we need to tag them with something to show they are different (e.g., set them to another value).
3. After this finding process, we can traverse the whole board again. Those remaining `'O'`s are surrounded and need to be flipped. And don't forget to flip tagged cells to `'O'` back.

Based on the above discussion, we decide to use the second way.

#### Solution
```cpp
#include <iostream>

#include <vector>

using namespace std;

static int desyncio = []() {
	std::ios::sync_with_stdio(false);
	cin.tie(nullptr);
	cout.tie(nullptr);
	return 0;
}();

class Solution {
public:
	// convert the unsurrounded cells to 'U'

	void updateBoard(vector<vector<char>>& board, int r, int c)
	{
		if (r < 0 || c < 0 || r >= board.size() || c >= board[0].size() || board[r][c] != 'O')
			return;

		board[r][c] = 'U';

		// continue to explore its neighbors

		updateBoard(board, r - 1, c);
		updateBoard(board, r + 1, c);
		updateBoard(board, r, c - 1);
		updateBoard(board, r, c + 1);
	}
	void solve(vector<vector<char>> & board) {
		int row = board.size();
		if (row == 0)
			return;
		int col = board[0].size();
		if (col == 0)
			return;

		// Start from the 'O's on the first/last row, first/last column

		// Flip the unsurrounded 'O' to 'U'

		for (int i = 0; i < col; ++i)
		{
			updateBoard(board, 0, i);
			if (row > 1)
				updateBoard(board, row - 1, i);
		}
		if (row > 2)
			for (int i = 1; i < row - 1; ++i)
			{
				updateBoard(board, i, 0);
				if (col > 1)
					updateBoard(board, i, col - 1);
			}

		// Flip the remaining 'O' to 'X', and 'U' to 'O'

		for (int i = 0; i < row; ++i)
			for (int j = 0; j < col; ++j)
				board[i][j] = board[i][j] == 'U' ? 'O' : 'X';
	}
};
int main()
{
	int row, col;
	cin >> row >> col;
	vector<vector<char>> board(row, vector<char>(col));
	char tmp;
	for (int i = 0; i < row; ++i)
	{
		for (int j = 0; j < col; ++j)
		{
			cin >> tmp;
			board[i][j] = tmp;
		}
	}
	Solution solution;
	solution.solve(board);
	cout << endl << endl << endl;
	for (int i = 0; i < row; ++i)
	{
		for (int j = 0; j < col - 1; ++j)
			cout << board[i][j] << " ";
		cout << board[i][col-1] << endl;
	}
}
```


### [Leetcode 200. Number of Islands](https://leetcode.com/problems/number-of-islands/)

#### Question

Given a 2d grid map of `'1'`s (land) and `'0'`s (water), count the number of islands. An island is surrounded by water and is formed by connecting adjacent lands horizontally or vertically. You may assume all four edges of the grid are all surrounded by water.

__Example 1:__
```
Input:
11110
11010
11000
00000

Output: 1
```

__Example 2:__
```
Input:
11000
11000
00100
00011

Output: 3
```

#### Train of Thought

The same idea as before. However, this time, instead of starting from borders, we can traverse as usual. Just __REMEMBER__: tag the visited area.

#### Solution
```cpp
#include <iostream>

#include <vector>

using namespace std;

class Solution {
public:
	void explore(vector<vector<char>>& grid, int i, int j)
	{
		int row = grid.size();
		int col = grid[0].size();
		if (i < 0 || i >= row || j < 0 || j >= col || grid[i][j] != '1')
			return;

		grid[i][j] = '2';
		explore(grid, i - 1, j);
		explore(grid, i + 1, j);
		explore(grid, i, j - 1);
		explore(grid, i, j + 1);
	}

	int numIslands(vector<vector<char>> & grid) {
		int row = grid.size();
		if (row == 0)
			return 0;
		int col = grid[0].size();
		if (col == 0)
			return 0;

		int cnt = 0;
		for (int i = 0; i < row; ++i)
			for (int j = 0; j < col; ++j)
				if (grid[i][j] == '1')
				{
					++cnt;				  // if not explored, we must find a new island

					explore(grid, i, j);  // set all explored (connected) cells to '2'

				}

		return cnt;
	}
};

int main()
{
	int row, col;
	cin >> row >> col;
	vector<vector<char>> grid(row, vector<char>(col));
	char tmp;
	for (int i = 0; i < row; ++i)
	{
		for (int j = 0; j < col; ++j)
		{
			cin >> tmp;
			grid[i][j] = tmp;
		}
	}
	Solution solution;
	cout << endl << endl << endl;
	cout << solution.numIslands(grid) << endl;
}
```

### [Leetcode 695. Max Area of Island](https://leetcode.com/problems/max-area-of-island/)

#### Question

Given a non-empty 2D array grid of `0`'s and `1`'s, an island is a group of `1`'s (representing land) connected 4-directionally (horizontal or vertical.) You may assume all four edges of the grid are surrounded by water.

Find the maximum area of an island in the given 2D array. (If there is no island, the maximum area is 0.)

__Example 1:__
```
[[0,0,1,0,0,0,0,1,0,0,0,0,0],
 [0,0,0,0,0,0,0,1,1,1,0,0,0],
 [0,1,1,0,1,0,0,0,0,0,0,0,0],
 [0,1,0,0,1,1,0,0,1,0,1,0,0],
 [0,1,0,0,1,1,0,0,1,1,1,0,0],
 [0,0,0,0,0,0,0,0,0,0,1,0,0],
 [0,0,0,0,0,0,0,1,1,1,0,0,0],
 [0,0,0,0,0,0,0,1,1,0,0,0,0]]
```

Given the above grid, return `6`. Note the answer is not `11`, because the island must be connected 4-directionally.

__Example 2:__
```
[[0,0,0,0,0,0,0,0]]
```
Given the above grid, return `0`.

__Note:__ The length of each dimension in the given grid does not exceed 50.
#### Train of Thought

The same idea as before. Just __REMEMBER__: tag the visited area.

#### Solution
```cpp
#include <iostream>

#include <vector>

#include <algorithm>

using namespace std;

class Solution {
public:
	void searchArea(int& curArea, vector<vector<int>>& grid, int i, int j)
	{
		int row = grid.size();
		int col = grid[0].size();

		if (i < 0 || i >= row || j < 0 || j >= col || grid[i][j] != 1)
			return;

		++curArea;
		grid[i][j] = 2;
		searchArea(curArea, grid, i - 1, j);
		searchArea(curArea, grid, i + 1, j);
		searchArea(curArea, grid, i, j - 1);
		searchArea(curArea, grid, i, j + 1);
	}
	int maxAreaOfIsland(vector<vector<int>> & grid) {
		int row = grid.size();
		if (row == 0)
			return 0;
		int col = grid[0].size();
		if (col == 0)
			return 0;

		int maxArea = 0;
		int curArea = 0;
		for (int i = 0; i < row; ++i)
			for (int j = 0; j < col; ++j)
			{
				curArea = 0;
				searchArea(curArea, grid, i, j);
				maxArea = max(maxArea, curArea);
			}

		return maxArea;
	}
};

int main()
{
	int row, col;
	cin >> row >> col;
	vector<vector<int>> grid(row, vector<int>(col));
	int tmp;
	for (int i = 0; i < row; ++i)
	{
		for (int j = 0; j < col; ++j)
		{
			cin >> tmp;
			grid[i][j] = tmp;
		}
	}
	Solution solution;
	cout << endl << endl << endl;
	cout << solution.maxAreaOfIsland(grid) << endl;
}
```

### [Leetcode 463. Island Perimeter](https://leetcode.com/problems/island-perimeter/)

#### Question

You are given a map in form of a two-dimensional integer grid where `1` represents land and `0` represents water.

Grid cells are connected horizontally/vertically (not diagonally). The grid is completely surrounded by water, and there is exactly one island (i.e., one or more connected land cells).

The island doesn't have "lakes" (water inside that isn't connected to the water around the island). One cell is a square with side length 1. The grid is rectangular, width and height don't exceed 100. Determine the perimeter of the island.

__Example 1:__
```
Input:
[[0,1,0,0],
 [1,1,1,0],
 [0,1,0,0],
 [1,1,0,0]]

Output: 16
```

#### Train of Thought

This question is a little different from previous ones. We are only concerned about the perimeter of the island. At the same time, we need to be careful about _how many one cell contributes to the perimeter_. There are five situations depending on `1`'s neighbors:
1. 0 neighbors --> That means that this cell is exactly our whole island. Perimeter is then 4.
2. 1 neighbor --> The cell contributes 3.  
3. 2 neighbors --> The cell contributes 2.
4. 3 neighbors --> The cell contributes 1.
5. 4 neighbors --> The cell contributes 0.

#### Solution
```cpp
#include <iostream>

#include <vector>

#include <algorithm>

using namespace std;

class Solution {
public:
	void searchNeighbors(int& perimeter, vector<vector<int>>& grid, int i, int j)
	{
		int row = grid.size();
		int col = grid[0].size();

		if (i < 0 || i >= row || j < 0 || j >= col || grid[i][j] != 1)
			return;

		--perimeter;
	}

	int islandPerimeter(vector<vector<int>> & grid) {
		int row = grid.size();
		if (row == 0)
			return 0;
		int col = grid[0].size();
		if (col == 0)
			return 0;

		int perimeter = 0;
		for (int i = 0; i < row; ++i)
			for (int j = 0; j < col; ++j)
				if (grid[i][j] == 1)
				{
					perimeter += 4;
					searchNeighbors(perimeter, grid, i - 1, j);
					searchNeighbors(perimeter, grid, i + 1, j);
					searchNeighbors(perimeter, grid, i, j - 1);
					searchNeighbors(perimeter, grid, i, j + 1);
				}
		return perimeter;
	}
};

int main()
{
	int row, col;
	cin >> row >> col;
	vector<vector<int>> grid(row, vector<int>(col));
	int tmp;
	for (int i = 0; i < row; ++i)
	{
		for (int j = 0; j < col; ++j)
		{
			cin >> tmp;
			grid[i][j] = tmp;
		}
	}
	Solution solution;
	cout << endl << endl << endl;
	cout << solution.islandPerimeter(grid) << endl;
}
```

### [Leetcode 733. Flood Fill](https://leetcode.com/problems/flood-fill/)

#### Question

An `image` is represented by a 2-D array of integers, each integer representing the pixel value of the image (from 0 to 65535).

Given a coordinate `(sr, sc)` representing the starting pixel (row and column) of the flood fill, and a pixel value `newColor`, "flood fill" the image.

To perform a "flood fill", consider the starting pixel, plus any pixels connected 4-directionally to the starting pixel of the same color as the starting pixel, plus any pixels connected 4-directionally to those pixels (also with the same color as the starting pixel), and so on. Replace the color of all of the aforementioned pixels with the newColor.

At the end, return the modified image.

__Example 1:__
```
Input:
image = [[1,1,1],[1,1,0],[1,0,1]]
sr = 1, sc = 1, newColor = 2

Output: [[2,2,2],[2,2,0],[2,0,1]]

Explanation:
From the center of the image (with position (sr, sc) = (1, 1)), all pixels connected
by a path of the same color as the starting pixel are colored with the new color.
Note the bottom corner is not colored 2, because it is not 4-directionally connected
to the starting pixel.
```

__Note:__ <br>
The length of `image` and `image[0]` will be in the range `[1, 50]`. <br>
The given starting pixel will satisfy `0 <= sr < image.length` and `0 <= sc < image[0].length`. <br>
The value of each color in `image[i][j]` and `newColor` will be an integer in `[0, 65535]`.<br>

#### Train of Thought

__ATTENTION:__ Think about the __critical__ situations. In this question, what if the `newColor` is the same as the color of the starting pixel? A less careful implementation will lead to a heap/stack overflow.

#### Solution
```cpp
#include <iostream>

#include <vector>

#include <algorithm>

using namespace std;

class Solution {
public:
	void modifyImage(const int& startColor, vector<vector<int>>& image, int sr, int sc, int newColor)
	{
		int row = image.size();
		int col = image[0].size();

		if (sr < 0 || sr >= row || sc < 0 || sc >= col || image[sr][sc] != startColor || image[sr][sc] == newColor)
			return;

		image[sr][sc] = newColor;
		modifyImage(startColor, image, sr - 1, sc, newColor);
		modifyImage(startColor, image, sr + 1, sc, newColor);
		modifyImage(startColor, image, sr, sc - 1, newColor);
		modifyImage(startColor, image, sr, sc + 1, newColor);
	}

	vector<vector<int>> floodFill(vector<vector<int>> & image, int sr, int sc, int newColor) {
		int row = image.size();
		if (row == 0)
			return image;
		int col = image[0].size();
		if (col == 0)
			return image;

		const int startColor = image[sr][sc];
		modifyImage(startColor, image, sr, sc, newColor);

		return image;
	}
};

int main()
{
	int row, col;
	cin >> row >> col;
	vector<vector<int>> image(row, vector<int>(col));
	int tmp;
	for (int i = 0; i < row; ++i)
	{
		for (int j = 0; j < col; ++j)
		{
			cin >> tmp;
			image[i][j] = tmp;
		}
	}
	int sr, sc, newColor;
	cin >> sr >> sc >> newColor;
	Solution solution;
	image = solution.floodFill(image, sr, sc, newColor);
	cout << endl << endl << endl;
	for (int i = 0; i < row; ++i)
	{
		for (int j = 0; j < col-1; ++j)
			cout << image[i][j] << " ";
		cout << image[i][col-1] << endl;
	}
}
```

### [Leetcode 1034. Coloring A Border](https://leetcode.com/problems/coloring-a-border/)

#### Question

Given a 2-dimensional `grid` of integers, each value in the grid represents the color of the grid square at that location.

Two squares belong to the same _connected component_ if and only if they have the same color and are next to each other in any of the 4 directions.

The _border_ of a connected component is all the squares in the connected component that are either 4-directionally adjacent to a square not in the component, or on the boundary of the grid (the first or last row or column).

Given a square at location `(r0, c0)` in the grid and a `color`, color the border of the connected component of that square with the given `color`, and return the final `grid`.

__Example 1:__
```
Input: grid = [[1,1],[1,2]], r0 = 0, c0 = 0, color = 3
Output: [[3, 3], [3, 2]]
```

__Example 2:__
```
Input: grid = [[1,2,2],[2,3,2]], r0 = 0, c0 = 1, color = 3
Output: [[1, 3, 3], [2, 3, 3]]
```

__Example 3:__
```
Input: grid = [[1,1,1],[1,1,1],[1,1,1]], r0 = 1, c0 = 1, color = 2
Output: [[2, 2, 2], [2, 1, 2], [2, 2, 2]]
```

__Note:__ <br>
1. `1 <= grid.length <= 50`
2. `1 <= grid[0].length <= 50`
3. `1 <= grid[i][j] <= 1000`
4. `0 <= r0 < grid.length`
5. `0 <= c0 < grid[0].length`
6. `1 <= color <= 1000`

#### Train of Thought

__ATTENTION:__ Tagging is sometimes a good solution to play around the critical cases (infinite loops). Tag border and non-border with different values.

#### Solution
```cpp
#include <iostream>

#include <vector>

#include <algorithm>

using namespace std;

static const auto _ = []() {
	ios::sync_with_stdio(false);
	cin.tie(nullptr);
	cout.tie(nullptr);
	return nullptr;
}();

class Solution {
public:
	void colorGrid(int startColor, vector<vector<int>>& grid, int r, int c, int color)
	{
		int row = grid.size();
		int col = grid[0].size();

		if (r < 0 || r >= row || c < 0 || c >= col || grid[r][c] != startColor)
			return;

		if (r == 0 || r == row - 1 || c == 0 || c == col - 1 || (grid[r][c - 1] != startColor && grid[r][c - 1] < 1001) || (grid[r][c + 1] != startColor && grid[r][c + 1] < 1001) || (grid[r - 1][c] != startColor && grid[r - 1][c] < 1001) || (grid[r + 1][c] != startColor && grid[r + 1][c] < 1001))
			grid[r][c] = 1001;
		else
			grid[r][c] = 1002;
		colorGrid(startColor, grid, r - 1, c, color);
		colorGrid(startColor, grid, r + 1, c, color);
		colorGrid(startColor, grid, r, c - 1, color);
		colorGrid(startColor, grid, r, c + 1, color);
	}

	vector<vector<int>> colorBorder(vector<vector<int>> & grid, int r0, int c0, int color) {
		int row = grid.size();
		if (row == 0)
			return grid;
		int col = grid[0].size();
		if (col == 0)
			return grid;
		int startColor = grid[r0][c0];
		if (startColor == color)
			return grid;

		colorGrid(startColor, grid, r0, c0, color);

		for (int i = 0; i < row; ++i)
			for (int j = 0; j < col; ++j)
			{
				if (grid[i][j] == 1001)
					grid[i][j] = color;
				else if (grid[i][j] == 1002)
					grid[i][j] = startColor;
			}

		return grid;
	}
};

int main()
{
	int row, col;
	cin >> row >> col;
	vector<vector<int>> image(row, vector<int>(col));
	int tmp;
	for (int i = 0; i < row; ++i)
	{
		for (int j = 0; j < col; ++j)
		{
			cin >> tmp;
			image[i][j] = tmp;
		}
	}
	int sr, sc, newColor;
	cin >> sr >> sc >> newColor;
	Solution solution;
	image = solution.colorBorder(image, sr, sc, newColor);
	cout << endl << endl << endl;
	for (int i = 0; i < row; ++i)
	{
		for (int j = 0; j < col - 1; ++j)
			cout << image[i][j] << " ";
		cout << image[i][col - 1] << endl;
	}
}
```


### [大家来扫雷](https://www.nowcoder.com/questionTerminal/2104cdc2aa464befac72868421066fcb)

#### Question

小M最近爱上了扫雷游戏，就是在一个`n*m`的区域中，有地雷，每一个方格上都有一个数字s，表示在这个方格周围有s颗雷，现在给你一张表明地雷的图，并且指定一个位置点开，请输出点开后的数字情况，若点开的地方的数字为`0`，则向该方格周围扩展，直到遇到数字或者地图边界为止，若点开的地方为地雷,那么直接输出`"GG"`。

周围指的是上，左上，左，左下，下，右下，右，右上八个方向。

__输入描述：__
```
第一行有两个数字n和m(2<=n,m<=1000)，表示地图的大小，
第二行有两个整数x和y(1<=x<=n,1<=y<=m)，表示点击第x行y列的方格，
接下来的是一个n行m列的一个矩阵，表示地图，其中.表示空地，*表示地雷。
```

__输出描述：__
```
如果点开的地方为地雷直接输出"GG"。
否则输出点击指定位置后的地图，"."表示未点开的空地，"*"表示地雷，数字表示在该方格周围的地雷数目。
```

__示例：__
<pre>
<b>输入：</b>
3 4
1 1
....
..* .
....

<b>输出：</b>
01..
01*.
01..
</pre>

#### Train of Thought
没什么特别的，就像之前的几题一样。

__注意事项__:
1. 如何节约内存? 使用 __global__ 变量。
2. 注意如何读取输入数据。
3. 注意如何输出数据。

#### Solution
```cpp
#include <iostream>

#include <vector>

using namespace std;

static const auto _ = []() {
	ios::sync_with_stdio(false);
	cin.tie(nullptr);
	cout.tie(nullptr);
	return nullptr;
}();

vector<vector<char>> grid;

class Solution {
public:
	char countMines(int x, int y) {
		int row = grid.size();
		int col = grid[0].size();
		int num = 0;
		vector<pair<int, int>> neighbors{ {x - 1, y - 1}, {x - 1, y}, {x - 1, y + 1},
		{x, y - 1}, {x, y + 1}, {x + 1, y - 1}, {x + 1, y}, {x + 1, y + 1} }; //eight neighors

		for (auto& n : neighbors)
			if (n.first >= 0 && n.first < row && n.second >= 0 && n.second < col && grid[n.first][n.second] == '*' )
				++num;

		return '0' + num;
	}
	void explore(int x, int y)
	{
		int row = grid.size();
		int col = grid[0].size();
		if (x < 0 || x >= row || y < 0 || y >= col || grid[x][y] != '.')
			return;
		char num = countMines(x, y);
		if (num != '0')
			grid[x][y] = num;
		else
		{
			grid[x][y] = '0';
			explore(x - 1, y - 1);
			explore(x - 1, y);
			explore(x - 1, y + 1);
			explore(x, y - 1);
			explore(x, y + 1);
			explore(x + 1, y - 1);
			explore(x + 1, y);
			explore(x + 1, y + 1);
		}
	}

	bool exploreMines(int x, int y)
	{
		int row = grid.size();
		if (row == 0)
			return true;
		int col = grid[0].size();
		if (col == 0)
			return true;
		if (x < 0 || x >= row || y < 0 || y >= col)
			return true;
		if (grid[x][y] == '*')
			return false;

		explore(x, y);
		return true;
	}
};

int main()
{
	int n, m; // n - row, m - column
	cin >> n >> m;
	int x, y; // start point grid[x-1][y-1]
	cin >> x >> y;
	char tmp;
	for (int i = 0; i < n; ++i)
	{
		vector<char> vTmp;
		for (int j = 0; j < m; ++j)
		{
			cin >> tmp;
			vTmp.push_back(tmp);
		}
		grid.push_back(vTmp);
	}
	Solution solution;
	if (solution.exploreMines(x - 1, y - 1))
		for (int i = 0; i < n; ++i)
		{
			for (int j = 0; j < m; ++j)
				cout << grid[i][j];
			cout << endl;
		}
	else
		cout << "GG" << endl;
}
```

### [207\. Course Schedule](https://leetcode.com/problems/course-schedule/)

Difficulty: **Medium**


There are a total of _n_ courses you have to take, labeled from `0` to `n-1`.

Some courses may have prerequisites, for example to take course 0 you have to first take course 1, which is expressed as a pair: `[0,1]`

Given the total number of courses and a list of prerequisite **pairs**, is it possible for you to finish all courses?

**Example 1:**

```
Input: 2, [[1,0]]
Output: true
Explanation: There are a total of 2 courses to take.
             To take course 1 you should have finished course 0\. So it is possible.
```

**Example 2:**

```
Input: 2, [[1,0],[0,1]]
Output: false
Explanation: There are a total of 2 courses to take.
             To take course 1 you should have finished course 0, and to take course 0 you should
             also have finished course 1\. So it is impossible.
```

**Note:**

1.  The input prerequisites is a graph represented by **a list of edges**, not adjacency matrices. Read more about .
2.  You may assume that there are no duplicate edges in the input prerequisites.

#### Train of Thought

We cannot finish all courses, only when there is cycle in the graph. For example, to learn course 1, we need to learn 0. To learn course 0, we need to learn course 1.

To find if there is a cycle, we can do a depth first search. To speed up the computation, we can store those checked nodes which do not have cycles. To mark the visited one, we need to use another number from those which are already checked.

#### Solution

Language: **C++**

```c++
class Solution
{
public:
    bool canFinishDFS(vector<vector<int>>& graph, vector<int>& visited, int course)
    {
        if (visited[course] == 1) return true;
        if (visited[course] == -1) return false;
        visited[course] = -1;
        for (int& neighbor : graph[course])
            if (!canFinishDFS(graph, visited, neighbor)) return false;
        visited[course] = 1;
        return true;
    }
    
    bool canFinish(int numCourses, vector<vector<int>>& prerequisites)
    {
        vector<vector<int>> graph(numCourses, vector<int>());
        vector<int> visited(numCourses); // -1: currently visited; 0: never visisted; 1: no cycle.
        for (vector<int>& pre : prerequisites)
            graph[pre[0]].push_back(pre[1]);
        
        for (int i = 0; i < numCourses; ++i)
            if (!canFinishDFS(graph, visited, i)) return false;
        return true;
    }
};
```

### [210\. Course Schedule II](https://leetcode.com/problems/course-schedule-ii/)

Difficulty: **Medium**


There are a total of _n_ courses you have to take, labeled from `0` to `n-1`.

Some courses may have prerequisites, for example to take course 0 you have to first take course 1, which is expressed as a pair: `[0,1]`

Given the total number of courses and a list of prerequisite **pairs**, return the ordering of courses you should take to finish all courses.

There may be multiple correct orders, you just need to return one of them. If it is impossible to finish all courses, return an empty array.

**Example 1:**

```
Input: 2, [[1,0]]
Output: [0,1]
Explanation: There are a total of 2 courses to take. To take course 1 you should have finished   
             course 0\. So the correct course order is [0,1].
```

**Example 2:**

```
Input: 4, [[1,0],[2,0],[3,1],[3,2]]
Output: [0,1,2,3] or [0,2,1,3]
Explanation: There are a total of 4 courses to take. To take course 3 you should have finished both     
             courses 1 and 2\. Both courses 1 and 2 should be taken after you finished course 0\.
             So one correct course order is [0,1,2,3]. Another correct ordering is [0,2,1,3] .
```

**Note:**

1.  The input prerequisites is a graph represented by **a list of edges**, not adjacency matrices. Read more about .
2.  You may assume that there are no duplicate edges in the input prerequisites.


#### Solution

Language: **C++**

```c++
#include <iostream>

#include <vector>

#include <queue>

using std::cout;
using std::endl;
using std::vector;
using std::queue;

// <<Algorithm>> P580 Depth-first search vertex ordering in a digraph
// modification: instead of stack, we use queue
class Solution {
public:
	bool canFinishDFS(vector<vector<int>>& graph, vector<int>& visited, int i)
	{
		if (visited[i] == 1) return true;
		if (visited[i] == -1) return false;
		visited[i] = -1;
		for (int& neighbor : graph[i])
			if (!canFinishDFS(graph, visited, neighbor)) return false;
		visited[i] = 1;
		return true;
	}

	void findOrderDFS(vector<vector<int>>& graph, vector<int>& visited, queue<int>& q, int i)
	{
		visited[i] = 2;
		for (int& neighbor : graph[i])
			if (visited[neighbor] != 2) findOrderDFS(graph, visited, q, neighbor);
		q.push(i);
	}

	vector<int> findOrder(int numCourses, vector<vector<int>>& prerequisites)
	{
		vector<vector<int>> graph(numCourses, vector<int>());
		vector<int> visited(numCourses); // -1: currently visited; 0: never visisted; 1: no cycle; 2: visited for finding order
		for (vector<int>& pre : prerequisites)
			graph[pre[0]].push_back(pre[1]); // To learn i, we must learn graph[i]

		for (int i = 0; i < numCourses; ++i)
			if (!canFinishDFS(graph, visited, i)) return {};

		queue<int> q;
		for (int i = 0; i < numCourses; ++i)
			if (visited[i] != 2) findOrderDFS(graph, visited, q, i);

		vector<int> res;
		while (!q.empty())
		{
			res.push_back(q.front());
			q.pop();
		}
		return res;
	}
};

std::ostream& operator<<(std::ostream& stream, const vector<int>& nums)
{
	for (auto& n : nums)
		stream << n << " ";
	cout << endl;
	return stream;
}

int main()
{
	Solution solution;
	vector<vector<int>> prerequisites{ {1, 0} };
	cout << solution.findOrder(2, prerequisites);
}

```


### [417\. Pacific Atlantic Water Flow](https://leetcode.com/problems/pacific-atlantic-water-flow/)

Difficulty: **Medium**


Given an `m x n` matrix of non-negative integers representing the height of each unit cell in a continent, the "Pacific ocean" touches the left and top edges of the matrix and the "Atlantic ocean" touches the right and bottom edges.

Water can only flow in four directions (up, down, left, or right) from a cell to another one with height equal or lower.

Find the list of grid coordinates where water can flow to both the Pacific and Atlantic ocean.

**Note:**

1.  The order of returned grid coordinates does not matter.
2.  Both _m_ and _n_ are less than 150.

**Example:**

```
Given the following 5x5 matrix:

  Pacific ~   ~   ~   ~   ~
       ~  1   2   2   3  (5) *
       ~  3   2   3  (4) (4) *
       ~  2   4  (5)  3   1  *
       ~ (6) (7)  1   4   5  *
       ~ (5)  1   1   2   4  *
          *   *   *   *   * Atlantic

Return:

[[0, 4], [1, 3], [1, 4], [2, 2], [3, 0], [3, 1], [4, 0]] (positions with parentheses in above matrix).
```

#### Train of Thought

遍历出那些能流进pacific的cells，遍历出那些能流进atlantic的cells，其中相交的部分便是我们要求的。

#### Solution

Language: **C++**

```c++
#include <iostream>

#include <queue>

#include <vector>

using std::cout;
using std::endl;
using std::vector;
using std::queue;

class Solution1
{
public:
	void DFS(vector<vector<int>>& matrix, vector<vector<int>>& visited, int height, int r, int c)
	{
		int row = matrix.size();
		int col = matrix[0].size();
		if (r < 0 || r >= row || c < 0 || c >= col || visited[r][c] == 1 || matrix[r][c] < height) return;
		visited[r][c] = 1;
		DFS(matrix, visited, matrix[r][c], r - 1, c);
		DFS(matrix, visited, matrix[r][c], r + 1, c);
		DFS(matrix, visited, matrix[r][c], r, c - 1);
		DFS(matrix, visited, matrix[r][c], r, c + 1);
	}

	vector<vector<int>> pacificAtlantic(vector<vector<int>>& matrix)
	{
		int row = matrix.size();
		if (row == 0) return {};
		int col = matrix[0].size();
		if (col == 0) return {};
		vector<vector<int>> visited_p(row, vector<int>(col, 0));
		vector<vector<int>> visited_a(row, vector<int>(col, 0));
		for (int i = 0; i < col; ++i)
		{
			DFS(matrix, visited_p, INT_MIN, 0, i);
			DFS(matrix, visited_a, INT_MIN, row - 1, i);
		}
		for (int i = 0; i < row; ++i)
		{
			DFS(matrix, visited_p, INT_MIN, i, 0);
			DFS(matrix, visited_a, INT_MIN, i, col - 1);
		}
		vector<vector<int>> coordinates;
		for (int r = 0; r < row; ++r)
			for (int c = 0; c < col; ++c)
				if (visited_p[r][c] == 1 && visited_a[r][c] == 1)
					coordinates.push_back({ r, c });
		return coordinates;
	}
};

class Solution2 {
public:
	void BFS(vector<vector<int>>& matrix, vector<vector<int>>& visited, queue<vector<int>>& q)
	{
		int row = matrix.size();
		int col = matrix[0].size();
		while (!q.empty())
		{
			int breadth = q.size();
			for (int i = 0; i < breadth; ++i)
			{
				vector<int> cur = q.front();
				q.pop();
				vector<vector<int>> neighbors{ {cur[0] - 1, cur[1]}, {cur[0] + 1, cur[1]}, {cur[0], cur[1] - 1}, {cur[0], cur[1] + 1} };
				for (vector<int>& n : neighbors)
					if (n[0] >= 0 && n[0] < row && n[1] >= 0 && n[1] < col && visited[n[0]][n[1]] == 0 && matrix[n[0]][n[1]] >= matrix[cur[0]][cur[1]])
					{
						q.push(n);
						visited[n[0]][n[1]] = 1;
					}
			}
		}
	}

	vector<vector<int>> pacificAtlantic(vector<vector<int>>& matrix) {
		int row = matrix.size();
		if (row == 0) return {};
		int col = matrix[0].size();
		if (col == 0) return {};
		vector<vector<int>> visited_p(row, vector<int>(col, 0));
		vector<vector<int>> visited_a(row, vector<int>(col, 0));
		queue<vector<int>> q_p;
		queue<vector<int>> q_a;
		for (int i = 0; i < col; ++i)
		{
			q_p.push({ 0, i });
			visited_p[0][i] = 1;
			q_a.push({ row - 1, i });
			visited_a[row - 1][i] = 1;
		}
		for (int i = 0; i < row; ++i)
		{
			q_p.push({ i, 0 });
			visited_p[i][0] = 1;
			q_a.push({ i, col - 1 });
			visited_a[i][col - 1] = 1;
		}
		BFS(matrix, visited_p, q_p);
		BFS(matrix, visited_a, q_a);
		vector<vector<int>> coordinates;
		for (int r = 0; r < row; ++r)
			for (int c = 0; c < col; ++c)
				if (visited_p[r][c] == 1 && visited_a[r][c] == 1)
					coordinates.push_back({ r, c });
		return coordinates;
	}
};

int main()
{
	vector<vector<int>> matrix{ {1,2,2,3,5},{3,2,3,4,4},{2,4,5,3,1},{6,7,1,4,5},{5,1,1,2,4} };
	Solution1 solution;
	vector<vector<int>> res = solution.pacificAtlantic(matrix);
	for (const auto& r : res)
		cout << r[0] << " " << r[1] << endl;
}

```


### [332\. Reconstruct Itinerary](https://leetcode.com/problems/reconstruct-itinerary/)

Difficulty: **Medium**


Given a list of airline tickets represented by pairs of departure and arrival airports `[from, to]`, reconstruct the itinerary in order. All of the tickets belong to a man who departs from `JFK`. Thus, the itinerary must begin with `JFK`.

**Note:**

1.  If there are multiple valid itineraries, you should return the itinerary that has the smallest lexical order when read as a single string. For example, the itinerary `["JFK", "LGA"]` has a smaller lexical order than `["JFK", "LGB"]`.
2.  All airports are represented by three capital letters (IATA code).
3.  You may assume all tickets form at least one valid itinerary.

**Example 1:**

```
Input: [["MUC", "LHR"], ["JFK", "MUC"], ["SFO", "SJC"], ["LHR", "SFO"]]
Output: ["JFK", "MUC", "LHR", "SFO", "SJC"]
```

**Example 2:**

```
Input: [["JFK","SFO"],["JFK","ATL"],["SFO","ATL"],["ATL","JFK"],["ATL","SFO"]]
Output: ["JFK","ATL","JFK","SFO","ATL","SFO"]
Explanation: Another possible reconstruction is ["JFK","SFO","ATL","JFK","ATL","SFO"].
             But it is larger in lexical order.
```


#### Solution

Language: **C++**

```c++
#include <algorithm>

#include <iostream>

#include <unordered_map>

#include <vector>

#include <deque>

using namespace std;

class Solution
{
private:
	// src -> {dest1, dest2, ..., destn}
	unordered_map<string, deque<string>> trips_;
	// ans (reversed)
	vector<string> route_;

	void visit(const string& src)
	{
		auto& dests = trips_[src];
		while (!dests.empty())
		{
			// get the smallest dest
			const string dest = dests.front();
			// remove the ticket
			dests.pop_front();
			// visit dest
			visit(dest);
		}
		// add current node to the route
		route_.push_back(src);
	}

public:
	vector<string> findItinerary(vector<vector<string>>& tickets)
	{
		route_.clear();
		trips_.clear();

		for (const auto& t : tickets)
			trips_[t[0]].push_back(t[1]);

		for (auto& t : trips_)
		{
			auto& dests = t.second;
			std::sort(dests.begin(), dests.end());
		}

		const string start = "JFK";

		visit(start);

		return vector<string>(route_.crbegin(), route_.crend());
	}
};

int main()
{
	vector<vector<string>> tickets{ {"MUC", "LHR"}, {"JFK", "MUC"}, {"SFO", "SJC"}, {"LHR", "SFO"} };
	Solution solution;
	vector<string> route = solution.findItinerary(tickets);
	for (auto& s : route)
		cout << s << " ";
	cout << endl;
}

```
