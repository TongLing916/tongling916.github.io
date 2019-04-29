---
layout:     post
title:      "Union Find"
date:       2019-3-27
author:     Tong
catalog: true
tags:
    - Union Find
    - Depth-first Search
    - Breadth-first Search
---

### Summary

1.

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
```C++
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
