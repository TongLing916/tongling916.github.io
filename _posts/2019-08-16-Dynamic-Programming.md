---
layout:     post
title:      "Dynamic Programming"
date:       2019-8-16
author:     Tong
catalog: true
tags:
    - Algorithm
---

### Summary

1.


### [5\. Longest Palindromic Substring](https://leetcode.com/problems/longest-palindromic-substring/)

Difficulty: **Medium**


Given a string **s**, find the longest palindromic substring in **s**. You may assume that the maximum length of **s** is 1000.

**Example 1:**

```
Input: "babad"
Output: "bab"
Note: "aba" is also a valid answer.
```

**Example 2:**

```
Input: "cbbd"
Output: "bb"
```


#### Train of Thought

首先明确一点，一个具有相同字母的`string`的单词一定是回文单词。根据这个信息，我们可以每次都先向右扩展成具有相同字母的单词。接着，左右两边各自尝试着同时扩展（通过比较字母是否相同）。得出来的结果就是最佳单词。

一个小细节：每次向右扩展相同字母后，我们下一轮可以从下一个不同的字母开始，因为我们只需要对每个不同的字母进行遍历。

#### Solution

Language: **C++**

```c++
#include <iostream>
#include <string>

using namespace std;

class Solution {
public:
	string longestPalindrome(string s) {
		int n = s.size();
		int max_len = 0;
		int start = 0;
		for (int i = 0; i < n;)
		{
			int left = i, right = i;
			while (right + 1 < n && s[right + 1] == s[left]) ++right;

			i = right + 1;

			while (left >= 1 && right < n - 1 && s[left - 1] == s[right + 1])
			{
				--left;
				++right;
			}

			if (right - left + 1 > max_len)
			{
				max_len = right - left + 1;
				start = left;
			}

		}
		return s.substr(start, max_len);
	}
};

int main()
{
	Solution solution;
	cout << solution.longestPalindrome("ananas") << endl;
}

```

### [62\. Unique Paths](https://leetcode.com/problems/unique-paths/)

Difficulty: **Medium**


A robot is located at the top-left corner of a _m_ x _n_ grid (marked 'Start' in the diagram below).

The robot can only move either down or right at any point in time. The robot is trying to reach the bottom-right corner of the grid (marked 'Finish' in the diagram below).

How many possible unique paths are there?

![](https://assets.leetcode.com/uploads/2018/10/22/robot_maze.png)  
<small style="display: inline;">Above is a 7 x 3 grid. How many possible unique paths are there?</small>

**Note:** _m_ and _n_ will be at most 100.

**Example 1:**

```
Input: m = 3, n = 2
Output: 3
Explanation:
From the top-left corner, there are a total of 3 ways to reach the bottom-right corner:
1\. Right -> Right -> Down
2\. Right -> Down -> Right
3\. Down -> Right -> Right
```

**Example 2:**

```
Input: m = 7, n = 3
Output: 28
```


#### Train of Thought

要到达(i, j)的位置，我们上一步只能是(i - 1, j)或者(i, j - 1)。因此，如果我们知道上一步的次数，把它们加起来就是到达当前位置的次数。

除此之外，对于第一行或者第一列，path只有1种。

#### Solution

Language: **C++**

```c++
#include <iostream>
#include <vector>

using namespace std;

class Solution {
public:
	int uniquePaths(int m, int n) {
		vector<vector<int>> dp(n, vector<int>(m, 1)); // n: row, m: col
		for (int i = 1; i < n; ++i)
			for (int j = 1; j < m; ++j)
				dp[i][j] = dp[i - 1][j] + dp[i][j - 1];
		return dp[n - 1][m - 1];
	}
};

int main()
{
	Solution solution;
	cout << solution.uniquePaths(7, 3) << endl;
}

```


### [63\. Unique Paths II](https://leetcode.com/problems/unique-paths-ii/)

Difficulty: **Medium**


A robot is located at the top-left corner of a _m_ x _n_ grid (marked 'Start' in the diagram below).

The robot can only move either down or right at any point in time. The robot is trying to reach the bottom-right corner of the grid (marked 'Finish' in the diagram below).

Now consider if some obstacles are added to the grids. How many unique paths would there be?

![](https://assets.leetcode.com/uploads/2018/10/22/robot_maze.png)

An obstacle and empty space is marked as `1` and `0` respectively in the grid.

**Note:** _m_ and _n_ will be at most 100.

**Example 1:**

```
Input:
[
  [0,0,0],
  [0,1,0],
  [0,0,0]
]
Output: 2
Explanation:
There is one obstacle in the middle of the 3x3 grid above.
There are two ways to reach the bottom-right corner:
1\. Right -> Right -> Down -> Down
2\. Down -> Down -> Right -> Right
```

#### Train of Thought

正确初始化第一行和第一列的次数十分重要。如果第一行的某个位置有个障碍物，那么这个位置以及它的右边都无法被到达（即次数为0）。同理，第一列也是如此。然后向前面一样，叠加得到最后结果。

#### Solution

Language: **C++**

```c++
#include <iostream>
#include <vector>

using namespace std;

class Solution {
public:
	int uniquePathsWithObstacles(vector<vector<int>>& obstacleGrid) {
		int row = obstacleGrid.size();
		if (row == 0) return 0;
		int col = obstacleGrid[0].size();
		if (col == 0) return 0;

		vector<vector<long long>> dp(row, vector<long long>(col, 0));
		for (int i = 0; i < row; ++i)
		{
			if (obstacleGrid[i][0] == 0) dp[i][0] = 1;
			else break;
		}
		for (int i = 0; i < col; ++i)
		{
			if (obstacleGrid[0][i] == 0) dp[0][i] = 1;
			else break;
		}

		for (int r = 1; r < row; ++r)
			for (int c = 1; c < col; ++c)
			{
				if (obstacleGrid[r][c] == 1) dp[r][c] = 0;
				else dp[r][c] = dp[r - 1][c] + dp[r][c - 1];
			}
		return dp[row - 1][col - 1];
	}
};

int main()
{
	Solution solution;
	vector<vector<int>> obstacleGrid({
		{0, 0, 0},
		{0, 1, 0},
		{0, 0, 0}
		});
	cout << solution.uniquePathsWithObstacles(obstacleGrid) << endl;
}

```


### [64\. Minimum Path Sum](https://leetcode.com/problems/minimum-path-sum/)

Difficulty: **Medium**


Given a _m_ x _n_ grid filled with non-negative numbers, find a path from top left to bottom right which _minimizes_ the sum of all numbers along its path.

**Note:** You can only move either down or right at any point in time.

**Example:**

```
Input:
[
  [1,3,1],
  [1,5,1],
  [4,2,1]
]
Output: 7
Explanation: Because the path 1→3→1→1→1 minimizes the sum.
```

#### Train of Thought


#### Solution

Language: **C++**

```c++
#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

class Solution {
public:
	int minPathSum(vector<vector<int>>& grid) {
		typedef long long ll;
		int row = grid.size();
		if (row == 0) return 0;
		int col = grid[0].size();
		if (col == 0) return 0;
		vector<vector<ll>> dp(row, vector<ll>(col, 0));
		dp[0][0] = grid[0][0];
		for (int i = 1; i < row; ++i)
			dp[i][0] = dp[i - 1][0] + grid[i][0];
		for (int i = 1; i < col; ++i)
			dp[0][i] = dp[0][i - 1] + grid[0][i];
		for (int r = 1; r < row; ++r)
			for (int c = 1; c < col; ++c)
				dp[r][c] = min(dp[r - 1][c], dp[r][c - 1]) + grid[r][c];
		return dp[row - 1][col - 1];
	}
};

int main()
{
	Solution solution;
	vector<vector<int>> grid{
		{1, 3, 1},
		{1, 5, 1},
		{4, 2, 1}
	};
	cout << solution.minPathSum(grid) << endl;
}

```

### [91\. Decode Ways](https://leetcode.com/problems/decode-ways/)

Difficulty: **Medium**


A message containing letters from `A-Z` is being encoded to numbers using the following mapping:

```
'A' -> 1
'B' -> 2
...
'Z' -> 26
```

Given a **non-empty** string containing only digits, determine the total number of ways to decode it.

**Example 1:**

```
Input: "12"
Output: 2
Explanation: It could be decoded as "AB" (1 2) or "L" (12).
```

**Example 2:**

```
Input: "226"
Output: 3
Explanation: It could be decoded as "BZ" (2 26), "VF" (22 6), or "BBF" (2 2 6).
```


#### Train of Thought

记住'0'是无法被decode的。


#### Solution

Language: **C++**

```c++
#include <iostream>
#include <string>

using namespace std;

class Solution {
public:
	int numDecodings(string s) {
		int n = s.size();
		if (n == 0 || s[0] == '0') return 0;
		if (n == 1) return 1;
		int pre2 = 1, pre1 = 1;
		int cur;
		for (int i = 1; i < n; ++i) {
			cur = 0;
			int first = (s[i] - '0');
			int second = stoi(s.substr(i - 1, 2));
			if (1 <= first && first <= 9) cur += pre1;
			if (10 <= second && second <= 26) cur += pre2;
			pre2 = pre1;
			pre1 = cur;
		}
		return cur;
	}
};

int main()
{
	Solution solution;
	cout << solution.numDecodings("101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010") << endl;
}

```


### [91\. Decode Ways](https://leetcode.com/problems/decode-ways/)

Difficulty: **Medium**


A message containing letters from `A-Z` is being encoded to numbers using the following mapping:

```
'A' -> 1
'B' -> 2
...
'Z' -> 26
```

Given a **non-empty** string containing only digits, determine the total number of ways to decode it.

**Example 1:**

```
Input: "12"
Output: 2
Explanation: It could be decoded as "AB" (1 2) or "L" (12).
```

**Example 2:**

```
Input: "226"
Output: 3
Explanation: It could be decoded as "BZ" (2 26), "VF" (22 6), or "BBF" (2 2 6).
```


#### Train of Thought

#### Solution

Language: **C++**

```c++
class Solution {
public:
    int numDecodings(string s) {
        
    }
};
```



### [95. Unique Binary Search Trees II](https://leetcode.com/problems/unique-binary-search-trees-ii/)

#### Question

Given an integer `n`, generate all structurally unique __BST__'s (binary search trees) that store values `1 ... n`.

__Example:__
```
Input: 3
Output:
[
  [1,null,3,2],
  [3,2,null,1],
  [3,1,null,null,2],
  [2,1,3],
  [1,null,2,null,3]
]
Explanation:
The above output corresponds to the 5 unique BST's shown below:

   1         3     3      2      1
    \       /     /      / \      \
     3     2     1      1   3      2
    /     /       \                 \
   2     1         2                 3
```

#### Train of Thought

For every number `i`, we can generate all possible left trees between `1` and `i - 1`, and all possible right trees between `i + 1` and `n`.

#### Solution
```cpp
#include <iostream>

#include <vector>

using std::vector;

struct TreeNode
{
	int val;
	TreeNode* left;
	TreeNode* right;
	TreeNode(int x) : val(x), left(NULL), right(NULL) {}
};

class Solution {
public:
	vector<TreeNode*> generateTrees(int n)
	{
		if (n <= 0)
		{
			vector<TreeNode*> trees;
			return trees;
		}
		return generateTrees(1, n);
	}

	vector<TreeNode*> generateTrees(int lo, int hi)
	{
		vector<TreeNode*> trees;
		if (lo > hi)
		{
			trees.push_back(nullptr);  // important step
		}

		else if (lo == hi)
		{
			TreeNode* tree = new TreeNode(lo);
			trees.push_back(tree);
		}
		else
		{
			for (int i = lo; i <= hi; ++i)
			{
				vector<TreeNode*> l_nodes = generateTrees(lo, i - 1);
				vector<TreeNode*> r_nodes = generateTrees(i + 1, hi);
				for (auto l_node : l_nodes)
				{
					for (auto r_node : r_nodes)
					{
						TreeNode* cur = new TreeNode(i);
						cur->left = l_node;
						cur->right = r_node;
						trees.push_back(cur);
					}
				}
			}
		}

		return trees;
	}
};

int main()
{
	Solution solution;
	vector<TreeNode*> trees = solution.generateTrees(0);
}
```


### [96\. Unique Binary Search Trees](https://leetcode.com/problems/unique-binary-search-trees/)

Difficulty: **Medium**


Given _n_, how many structurally unique **BST's** (binary search trees) that store values 1 ... _n_?

**Example:**

```
Input: 3
Output: 5
Explanation:
Given n = 3, there are a total of 5 unique BST's:

   1         3     3      2      1
    \       /     /      / \      \
     3     2     1      1   3      2
    /     /       \                 \
   2     1         2                 3
```


#### Solution

Language: **C++**

```c++
class Solution {
public:
    int numTrees(int n) {
       long dp[n+1] = {0};
        dp[0] = 1;
        dp[1] =1;

        for(int i = 2; i<n+1; i++) {
            long temp = 0;
            for (int j = 0; j < i ; j++) {
                temp += dp[j]* dp[i-1-j];
            }
            dp[i]=temp;
        }

        return dp[n];
    }
};
```


### [120\. Triangle](https://leetcode.com/problems/triangle/)

Difficulty: **Medium**


Given a triangle, find the minimum path sum from top to bottom. Each step you may move to adjacent numbers on the row below.

For example, given the following triangle

```
[
     [2],
    [3,4],
   [6,5,7],
  [4,1,8,3]
]
```

The minimum path sum from top to bottom is `11` (i.e., **2** + **3** + **5** + **1** = 11).

**Note:**

Bonus point if you are able to do this using only _O_(_n_) extra space, where _n_ is the total number of rows in the triangle.


#### Solution

Language: **C++**

```c++
#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

class Solution {
public:
	int minimumTotal(vector<vector<int>>& triangle) {
		vector<int> res(triangle.back().size(), 0);
		res[0] = triangle[0][0];
		int n = triangle.size();
		for (int i = 1; i < n; ++i)
		{
			res[i] = res[i - 1] + triangle[i][i];
			for (int j = i - 1; j > 0; --j)
				res[j] = min(res[j - 1] + triangle[i][j], res[j] + triangle[i][j]);
			res[0] = res[0] + triangle[i][0];
		}
		return * min_element(res.begin(), res.end());
	}
};

int main()
{
	vector<vector<int>> triangle{ {2}, {3,4}, {6,5,7}, {4,1,8,3} };
	Solution solution;
	cout << solution.minimumTotal(triangle) << endl;
}

```


### [139\. Word Break](https://leetcode.com/problems/word-break/)

Difficulty: **Medium**


Given a **non-empty** string _s_ and a dictionary _wordDict_ containing a list of **non-empty** words, determine if _s_ can be segmented into a space-separated sequence of one or more dictionary words.

**Note:**

*   The same word in the dictionary may be reused multiple times in the segmentation.
*   You may assume the dictionary does not contain duplicate words.

**Example 1:**

```
Input: s = "leetcode", wordDict = ["leet", "code"]
Output: true
Explanation: Return true because "leetcode" can be segmented as "leet code".
```

**Example 2:**

```
Input: s = "applepenapple", wordDict = ["apple", "pen"]
Output: true
Explanation: Return true because "applepenapple" can be segmented as "apple pen apple".
             Note that you are allowed to reuse a dictionary word.
```

**Example 3:**

```
Input: s = "catsandog", wordDict = ["cats", "dog", "sand", "and", "cat"]
Output: false
```

#### Train of Thought

要采用递归的思想，我们可以把原string切割两段，前面的一段递归，后面的一段查找字典。

在查找过程中，我们应记录一些中间结果，方便后续查找。


#### Solution

Language: **C++**

```c++
#include <iostream>
#include <string>
#include <vector>
#include <unordered_set>
#include <unordered_map>

using namespace std;

// https://www.youtube.com/watch?v=ptlwluzeC1I
class Solution {
private:
	unordered_map<string, bool> mem_;
public:
	bool wordBreak(string s, vector<string>& wordDict)
	{
		// create a hashset of words for fast query
		unordered_set<string> dict(wordDict.cbegin(), wordDict.cend());
		// query the answer of the original string
		return wordBreak(s, dict);
	}

	bool wordBreak(const string& s, const unordered_set<string>& dict)
	{
		// whole string is a word, memorize and return
		if (dict.count(s)) return mem_[s] = true;
		// in memory, directly return
		if (mem_.count(s)) return mem_[s];

		// Try every break point.
		for (int i = 1; i < s.length(); ++i)
		{
			const string left = s.substr(0, i);
			const string right = s.substr(i);
			// find the solution for s
			if (dict.count(right) && wordBreak(left, dict)) return mem_[s] = true;
		}
		// no solution for s, memorize and return
		return mem_[s] = false;
	}
};

int main()
{
	Solution solution;
	vector<string> wordDict{"leet", "code"};
	cout << solution.wordBreak("leetcode", wordDict) << endl;
}

```


### [152\. Maximum Product Subarray](https://leetcode.com/problems/maximum-product-subarray/)

Difficulty: **Medium**


Given an integer array `nums`, find the contiguous subarray within an array (containing at least one number) which has the largest product.

**Example 1:**

```
Input: [2,3,-2,4]
Output: 6
Explanation: [2,3] has the largest product 6.
```

**Example 2:**

```
Input: [-2,0,-1]
Output: 0
Explanation: The result cannot be 2, because [-2,-1] is not a subarray.
```


#### Solution

Language: **C++**

```c++
#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

class Solution {
public:
	int maxProduct(vector<int>& nums) {
		if (nums.size() == 0) return 0;
		int res = nums[0];
		// max and min product so far
		int max_product = nums[0];
		int min_product = nums[0];
		for (int i = 1; i < nums.size(); ++i)
		{
			int tmp = min_product;
			min_product = min(nums[i], min(max_product * nums[i], min_product * nums[i]));
			max_product = max(nums[i], max(max_product * nums[i], tmp * nums[i]));
			res = max(res, max_product);
		}
		return res;
	}
};

int main()
{
	Solution solution;
	vector<int> nums{ 3, 4, -1, 6 };
	cout << solution.maxProduct(nums) << endl;
}

```


### [213\. House Robber II](https://leetcode.com/problems/house-robber-ii/)

Difficulty: **Medium**


You are a professional robber planning to rob houses along a street. Each house has a certain amount of money stashed. All houses at this place are **arranged in a circle.** That means the first house is the neighbor of the last one. Meanwhile, adjacent houses have security system connected and **it will automatically contact the police if two adjacent houses were broken into on the same night**.

Given a list of non-negative integers representing the amount of money of each house, determine the maximum amount of money you can rob tonight **without alerting the police**.

**Example 1:**

```
Input: [2,3,2]
Output: 3
Explanation: You cannot rob house 1 (money = 2) and then rob house 3 (money = 2),
             because they are adjacent houses.
```

**Example 2:**

```
Input: [1,2,3,1]
Output: 4
Explanation: Rob house 1 (money = 1) and then rob house 3 (money = 3).
             Total amount you can rob = 1 + 3 = 4.
```


#### Solution

Language: **C++**

```c++
#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

class Solution {
public:

	int rob_not_loop(vector<int>& nums) {
		if (nums.size() == 0) return 0;
		// maximum amout if robbing or not robbing the current one
		int not_rob = 0;
		int rob = nums[0];
		for (int i = 1; i < nums.size(); ++i)
		{
			int tmp_rob = rob;
			rob = not_rob + nums[i];
			not_rob = max(not_rob, tmp_rob);  // ATTENTION
		}
		return max(rob, not_rob);
	}

	int rob(vector<int>& nums)
	{
		if (nums.size() == 0) return 0;
		if (nums.size() == 1) return nums[0];
		vector<int> nums_wo_first(nums.begin() + 1, nums.end());
		nums.pop_back(); // nums_wo_last
		return max(rob_not_loop(nums_wo_first), rob_not_loop(nums));
	}
};

int main()
{
	Solution solution;
	vector<int> nums{ 2,3,2 };
	cout << solution.rob(nums) << endl;
}

```


### [221\. Maximal Square](https://leetcode.com/problems/maximal-square/)

Difficulty: **Medium**


Given a 2D binary matrix filled with 0's and 1's, find the largest square containing only 1's and return its area.

**Example:**

```
Input:

1 0 1 0 0
1 0 1 1 1
1 1 1 1 1
1 0 0 1 0

Output: 4
```


#### Solution

Language: **C++**

```c++
#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

class Solution {
public:
	int maximalSquare(vector<vector<char>>& matrix) {
		int row = matrix.size();
		if (row == 0) return 0;
		int col = matrix[0].size();
		if (col == 0) return 0;
		vector<vector<int>> edges(row, vector<int>(col, 0));
		int max_edge = 0;
		for (int r = 0; r < row; ++r)
			for (int c = 0; c < col; ++c)
			{
				if (matrix[r][c] == '0') continue;
				if (r == 0 || c == 0)
				{
					edges[r][c] = 1;
				}
				else
				{
					if (matrix[r][c - 1] == '1' && matrix[r - 1][c] == '1')
						edges[r][c] = min(edges[r - 1][c - 1], min(edges[r][c - 1], edges[r - 1][c])) + 1;
					else
						edges[r][c] = 1;
				}
				max_edge = max(max_edge, edges[r][c]);
			}
		return max_edge * max_edge;
	}
};

int main()
{
	Solution solution;
	vector<vector<char>> matrix1{
		{'1', '0', '1', '0', '0'},
		{'1', '0', '1', '1', '1'},
		{'1', '1', '1', '1', '1'},
		{'1', '0', '0', '1', '0'}
	};
	vector<vector<char>> matrix2{
		{'0', '0', '0', '1'},
		{'1', '1', '0', '1'},
		{'1', '1', '1', '1'},
		{'0', '1', '1', '1'},
		{'0', '1', '1', '1'}
	};
	cout << solution.maximalSquare(matrix1) << endl;
	cout << solution.maximalSquare(matrix2) << endl;
}

```


### [264\. Ugly Number II](https://leetcode.com/problems/ugly-number-ii/)

Difficulty: **Medium**


Write a program to find the `n`-th ugly number.

Ugly numbers are **positive numbers** whose prime factors only include `2, 3, 5`. 

**Example:**

```
Input: n = 10
Output: 12
Explanation: 1, 2, 3, 4, 5, 6, 8, 9, 10, 12 is the sequence of the first 10 ugly numbers.
```

**Note:**  

1.  `1` is typically treated as an ugly number.
2.  `n` **does not exceed 1690**.


#### Solution

Language: **C++**

```c++

#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

// https://www.youtube.com/watch?v=ZG86C_U-vRg
class Solution
{
public:
	int nthUglyNumber(int n)
	{
		static vector<int> nums{ 1 };
		static int i2 = 0;
		static int i3 = 0;
		static int i5 = 0;
		while (nums.size() < n)
		{
			const int next2 = nums[i2] * 2;
			const int next3 = nums[i3] * 3;
			const int next5 = nums[i5] * 5;
			const int next = min(next2, min(next3, next5));
			if (next == next2) ++i2;
			if (next == next3) ++i3;
			if (next == next5) ++i5;
			nums.push_back(next);
		}
		return nums[n - 1];
	}
};

int main()
{
	Solution solution;
	solution.nthUglyNumber(100);
}

```

### [279\. Perfect Squares](https://leetcode.com/problems/perfect-squares/)

Difficulty: **Medium**


Given a positive integer _n_, find the least number of perfect square numbers (for example, `1, 4, 9, 16, ...`) which sum to _n_.

**Example 1:**

```
Input: n = 12
Output: 3
Explanation: 12 = 4 + 4 + 4.
```

**Example 2:**

```
Input: n = 13
Output: 2
Explanation: 13 = 4 + 9.
```


#### Solution

Language: **C++**

```c++

#include <algorithm>

#include <iostream>

#include <vector>

#include <queue>

using namespace std;

// DP
class Solution1
{
public:
	int numSquares(int n)
	{
		if (n <= 0) return 0;
		vector<int> num_squares(n + 1, INT_MAX);
		num_squares[0] = 0;
		for (int i = 1; i <= n; ++i)
			for (int j = 1; j * j <= i; ++j)
				num_squares[i] = std::min(num_squares[i], num_squares[i - j * j] + 1);
		return num_squares.back();
	}
};

// BFS
class Solution2
{
public:
	int numSquares(int n)
	{
		if (n <= 0) return 0;
		vector<int> squares;
		vector<int> visited(n + 1, 0);
		queue<int> q;
		for (int i = 0; i * i <= n; ++i)
		{
			int tmp = i * i;
			squares.push_back(tmp);
			visited[tmp] = 1;
			q.push(tmp);
		}
		if (squares.back() == n) return 1;

		int num = 1;
		while (!q.empty()) // search in each level
		{
			++num;
			int breadth = q.size();
			for (int i = 0; i < breadth; ++i)
			{
				int cur = q.front();
				for (int& s : squares)
				{
					if (cur + s == n) return num;

					if (cur + s < n && visited[cur + s] == 0)
					{
						visited[cur + s] = 1;
						q.push(cur + s);
					}
					else if (cur + s > n)
					{
						break;
					}
				}
				q.pop();
			}
		}
		return 0;
	}
};

int main()
{
	Solution2 solution;
	cout << solution.numSquares(12) << endl;
}

```

### [300\. Longest Increasing Subsequence](https://leetcode.com/problems/longest-increasing-subsequence/)

Difficulty: **Medium**


Given an unsorted array of integers, find the length of longest increasing subsequence.

**Example:**

```
Input: [10,9,2,5,3,7,101,18]
Output: 4
Explanation: The longest increasing subsequence is [2,3,7,101], therefore the length is 4\.
```

**Note:**

*   There may be more than one LIS combination, it is only necessary for you to return the length.
*   Your algorithm should run in O(_n<sup>2</sup>_) complexity.

**Follow up:** Could you improve it to O(_n_ log _n_) time complexity?


#### Train of Thought

例如 [10,9,2,5,3,7,101,18]。

我们如何才能找到一个最长的子序列呢？

第一个元素 --> [10]

第二个元素是9，比10小，无法组成序列，为了后面更容易组成子序列，我们可以把10换成9. --> [9]

第三个元素是2，同理 --> [2]

第四个元素是5，我们可以接上去 --> [2, 5]

第五个元素是3，我们按照以前的思想，可以把较大的元素替换成这个3，为了找到合适的位置，我们可以采用二分查找 --> [2, 3]

第六个元素是7，--> [2, 3, 7]

第七个元素是101， --> [2, 3, 7, 101]

第八个元素是18， --> [2, 3, 7, 18]


#### Solution

Language: **C++**

```c++

#include <iostream>
#include <algorithm>
#include <vector>
#include <iterator>

using namespace std;

class Solution {
public:
	int lengthOfLIS(vector<int>& nums) {
		vector<int> res;
		for (int i = 0; i < nums.size(); i++)
		{
			// similar to binary search
			auto it = std::lower_bound(res.begin(), res.end(), nums[i]);
			if (it == res.end())
				res.push_back(nums[i]);
			else
				* it = nums[i];
		}
		return res.size();
	}
};

class NaiveSolution {
public:
	int lengthOfLIS(vector<int>& nums) {
		if (nums.size() == 0)
			return 0;

		int max_length = 1;
		vector<int> lengths{ 1 };
		for (int i = 1; i < nums.size(); ++i)
		{
			int cur_length = 1;
			for (int j = 0; j < i; ++j)
			{
				if (nums[j] < nums[i])
					cur_length = (lengths[j] + 1 > cur_length) ? (lengths[j] + 1) : cur_length;
			}
			lengths.push_back(cur_length);
			max_length = (cur_length > max_length) ? cur_length : max_length;
		}

		return max_length;
	}
};

int main()
{
	Solution solution;

	vector<int> test1{ 1,3,5,4,7 };

	cout << solution.lengthOfLIS(test1) << endl;
}

```



### [304\. Range Sum Query 2D - Immutable](https://leetcode.com/problems/range-sum-query-2d-immutable/)

Difficulty: **Medium**


Given a 2D matrix _matrix_, find the sum of the elements inside the rectangle defined by its upper left corner (_row_ 1, _col_ 1) and lower right corner (_row_ 2, _col_ 2).

![Range Sum Query 2D](/static/images/courses/range_sum_query_2d.png)  
<small style="display: inline;">The above rectangle (with the red border) is defined by (row1, col1) = **(2, 1)** and (row2, col2) = **(4, 3)**, which contains sum = **8**.</small>

**Example:**  

```
Given matrix = [
  [3, 0, 1, 4, 2],
  [5, 6, 3, 2, 1],
  [1, 2, 0, 1, 5],
  [4, 1, 0, 1, 7],
  [1, 0, 3, 0, 5]
]

sumRegion(2, 1, 4, 3) -> 8
sumRegion(1, 1, 2, 2) -> 11
sumRegion(1, 2, 2, 4) -> 12
```

**Note:**  

1.  You may assume that the matrix does not change.
2.  There are many calls to _sumRegion_ function.
3.  You may assume that _row_ 1 ≤ _row_ 2 and _col_ 1 ≤ _col_ 2.


#### Solution

Language: **C++**

```c++

#include <iostream>
#include <vector>

using namespace std;

typedef long long LL;

class NumMatrix
{
private:
	vector<vector<LL>> dict_;
public:
	NumMatrix(vector<vector<int>>& matrix)
	{
		if (matrix.size() > 0 && matrix[0].size() > 0)
		{
			int row = matrix.size();
			int col = matrix[0].size();
			dict_.resize(row, vector<LL>(col, 0));
			for (int r = 0; r < row; ++r)
				for (int c = 0; c < col; ++c)
				{
					if (r == 0 && c == 0) dict_[r][c] = matrix[0][0];
					else if (r == 0) dict_[r][c] = dict_[r][c - 1] + matrix[r][c];
					else if (c == 0) dict_[r][c] = dict_[r - 1][c] + matrix[r][c];
					else dict_[r][c] = dict_[r][c - 1] + dict_[r - 1][c] - dict_[r - 1][c - 1] + matrix[r][c];
				}
		}
	}

	int sumRegion(int row1, int col1, int row2, int col2)
	{
		if (row1 == 0 && col1 == 0) return dict_[row2][col2];
		if (row1 == 0) return dict_[row2][col2] - dict_[row2][col1 - 1];
		if (col1 == 0) return dict_[row2][col2] - dict_[row1 - 1][col2];
		return dict_[row2][col2] - dict_[row1 - 1][col2] - dict_[row2][col1 - 1] + dict_[row1 - 1][col1 - 1];
	}
};


int main()
{
	vector<vector<int>> matrix{ {3, 0, 1, 4, 2}, {5, 6, 3, 2, 1}, {1, 2, 0, 1, 5}, {4, 1, 0, 1, 7}, {1, 0, 3, 0, 5} };
	NumMatrix nm(matrix);
	cout << nm.sumRegion(2, 1, 4, 3) << endl;
	cout << nm.sumRegion(1, 1, 2, 2) << endl;
	cout << nm.sumRegion(1, 2, 2, 4) << endl;
}

/**
 * Your NumMatrix object will be instantiated and called as such:
 * NumMatrix* obj = new NumMatrix(matrix);
 * int param_1 = obj->sumRegion(row1,col1,row2,col2);
 */
```


### [309\. Best Time to Buy and Sell Stock with Cooldown](https://leetcode.com/problems/best-time-to-buy-and-sell-stock-with-cooldown/)

Difficulty: **Medium**


Say you have an array for which the _i_<sup>th</sup> element is the price of a given stock on day _i_.

Design an algorithm to find the maximum profit. You may complete as many transactions as you like (ie, buy one and sell one share of the stock multiple times) with the following restrictions:

*   You may not engage in multiple transactions at the same time (ie, you must sell the stock before you buy again).
*   After you sell your stock, you cannot buy stock on next day. (ie, cooldown 1 day)

**Example:**

```
Input: [1,2,3,0,2]
Output: 3
Explanation: transactions = [buy, sell, cooldown, buy, sell]
```


#### Solution

Language: **C++**

```c++
#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

class Solution {
public:
	int maxProfit(vector<int>& prices)
	{
		int n = prices.size();
		if (n == 0) return 0;

		// every day, we have three status
		// 0: we own a share at ith day (maybe we have bought it previously)
		// 1: we sell one share at ith day
		// 2: we do nothing (cooldown)
		// dp[i][j]: maximum profit under status j
		vector<vector<int>> dp(n, vector<int>(3, 0));  
		dp[0][0] = -prices[0];
		for (int i = 1; i < n; ++i)
		{
			// we can own a share at ith day only when:
			// 1) we have bought share before.
			// 2) we have passed cooldown and buy a share at ith day
			dp[i][0] = max(dp[i - 1][2] - prices[i], dp[i - 1][0]);

			// we can sell a share at ith only when we own a share
			dp[i][1] = dp[i - 1][0] + prices[i];

			// we can always do nothing
			// Attention: at ith day, the profit under status 0 can never exceed those under status 1 or 2.
			dp[i][2] = max(dp[i - 1][1], dp[i - 1][2]);
		}
		return max(dp[n - 1][1], dp[n - 1][2]);
	}
};

int main()
{
	Solution solution;
	vector<int> nums{ 1, 2, 3, 0, 2 };
	cout << solution.maxProfit(nums) << endl;
}

```


### [322\. Coin Change](https://leetcode.com/problems/coin-change/)

Difficulty: **Medium**


You are given coins of different denominations and a total amount of money _amount_. Write a function to compute the fewest number of coins that you need to make up that amount. If that amount of money cannot be made up by any combination of the coins, return `-1`.

**Example 1:**

```
Input: coins = [1, 2, 5], amount = 11
Output: 3
Explanation: 11 = 5 + 5 + 1
```

**Example 2:**

```
Input: coins = [2], amount = 3
Output: -1
```

**Note**:  
You may assume that you have an infinite number of each kind of coin.


#### Solution

Language: **C++**

```c++
#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

class Solution
{
public:
	int coinChange(vector<int>& coins, int amount)
	{
		if (amount == 0) return 0;

		// if dp[amount] > amount, that means we cannot construct amount using coins.
		vector<int> dp(amount + 1, amount + 1);
		dp[0] = 0;

		for (int i = 1; i <= amount; ++i)
			for (int j = 0; j < coins.size(); ++j)
				if (coins[j] <= i) dp[i] = min(dp[i], dp[i - coins[j]] + 1);

		return dp[amount] > amount ? -1 : dp[amount];
	}
};

int main()
{
	vector<int> coins1{ 1, 2, 5 };
	vector<int> coins2{ 333, 243, 214, 132, 281 };
	Solution solution;
	cout << solution.coinChange(coins1, 11) << endl;
	cout << solution.coinChange(coins2, 9334) << endl;
}

```

### [338\. Counting Bits](https://leetcode.com/problems/counting-bits/)

Difficulty: **Medium**


Given a non negative integer number **num**. For every numbers **i** in the range **0 ≤ i ≤ num** calculate the number of 1's in their binary representation and return them as an array.

**Example 1:**

```
Input: 2
Output: [0,1,1]
```

**Example 2:**

```
Input: 5
Output: [0,1,1,2,1,2]
```

**Follow up:**

*   It is very easy to come up with a solution with run time **O(n*sizeof(integer))**. But can you do it in linear time **O(n)** /possibly in a single pass?
*   Space complexity should be **O(n)**.
*   Can you do it like a boss? Do it without using any builtin function like **__builtin_popcount** in c++ or in any other language.


#### Solution

Language: **C++**

```c++
#include <iostream>
#include <vector>

using namespace std;

class Solution1 {
public:
	vector<int> countBits(int num) {
		vector<int> res{ 0 };
		for (int i = 1; i <= num; ++i)
		{
			if (i % 2 == 0) res.push_back(res[i / 2]);
			else res.push_back(res[i / 2] + 1);
		}
		return res;
	}
};

class Solution2 {
public:
	vector<int> countBits(int num) {
		vector<int> bits(num + 1, 0);
		for (int i = 1; i <= num; i++) {
			bits[i] = bits[i >> 1] + (i & 1);
		}
		return bits;
	}
};

int main()
{
	Solution1 solution;
	vector<int> res = solution.countBits(10);
	for (auto r : res)
		cout << r << endl;
}

```

### [343\. Integer Break](https://leetcode.com/problems/integer-break/)

Difficulty: **Medium**


Given a positive integer _n_, break it into the sum of **at least** two positive integers and maximize the product of those integers. Return the maximum product you can get.

**Example 1:**


```
Input: 2
Output: 1
Explanation: 2 = 1 + 1, 1 × 1 = 1.
```


**Example 2:**

```
Input: 10
Output: 36
Explanation: 10 = 3 + 3 + 4, 3 × 3 × 4 = 36.
```

**Note**: You may assume that _n_ is not less than 2 and not larger than 58.


#### Solution

Language: **C++**

```c++
#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

class Solution {
public:
	int integerBreak(int n) {
		vector<int> dp(n + 1, 1);
		for (int i = 3; i <= n; ++i)
			for (int j = 1; j <= i / 2; ++j)
				dp[i] = max(dp[i], max(j, dp[j]) * max(i - j, dp[i - j]));
		return dp[n];
	}
};

int main()
{
	Solution solution;
	cout << solution.integerBreak(10) << endl;
}

```

### [357\. Count Numbers with Unique Digits](https://leetcode.com/problems/count-numbers-with-unique-digits/)

Difficulty: **Medium**


Given a **non-negative** integer n, count all numbers with unique digits, x, where 0 ≤ x < 10<sup>n</sup>.


**Example:**

```
Input: 2
Output: 91
Explanation: The answer should be the total numbers in the range of 0 ≤ x < 100,
             excluding 11,22,33,44,55,66,77,88,99
```


#### Solution

Language: **C++**

```c++
#include <iostream>

using namespace std;

class Solution {
public:
	int countNumbersWithUniqueDigits(int n) {
		if (n == 0) return 1;
		if (n > 10) return 0;
		int cnt = 10;
		int cnt_i = 9; // i: number of digits
		int possible = 9;
		for (int i = 2; i <= n; ++i, --possible)
		{
			cnt_i * = possible;
			cnt += cnt_i;
		}
		return cnt;
	}
};

int main()
{
	Solution solution;
	cout << solution.countNumbersWithUniqueDigits(8) << endl;
}
```



### [368\. Largest Divisible Subset (Amazing)](https://leetcode.com/problems/largest-divisible-subset/)

Difficulty: **Medium**


Given a set of **distinct** positive integers, find the largest subset such that every pair (S<sub style="display: inline;">i</sub>, S<sub style="display: inline;">j</sub>) of elements in this subset satisfies:

S<sub style="display: inline;">i</sub> % S<sub style="display: inline;">j</sub> = 0 or S<sub style="display: inline;">j</sub> % S<sub style="display: inline;">i</sub> = 0.

If there are multiple solutions, return any subset is fine.

**Example 1:**


```
Input: [1,2,3]
Output: [1,2] (of course, [1,3] will also be ok)
```


**Example 2:**

```
Input: [1,2,4,8]
Output: [1,2,4,8]
```


#### Solution

Language: **C++**

```c++
#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

// https://leetcode-cn.com/problems/largest-divisible-subset/solution/dp-c-by-l2goer/
/*
这个题目要比一般的dp题目难，难在不只是输出满足条件的最大个数，而是要输出具体哪些。
所以需要记录下满足条件的下标值，解答中用idx来记录，最后一次的下标值放在了start中。
用了个比较巧妙的方式是start = idx[start];
*/
class Solution {
public:
	vector<int> largestDivisibleSubset(vector<int>& nums) {
		vector<int> dp(nums.size(), 1); // 记录到第i个位置的满足条件的个数
		vector<int> idx(nums.size(), -1); // 记录满足条件时的上一个的下标值

		int max = INT_MIN;
		int start = 0;
		sort(nums.begin(), nums.end()); // 1 % 2 不等于0，但是2 % 1 等于0，先排序
		for (int i = 0; i < nums.size(); ++i) { // i在j前面跑
			for (int j = 0; j < i; ++j) { // j 不大于 i 的那部分
				if (nums[i] % nums[j] == 0 && dp[i] < dp[j] + 1) { // 余数为0，同时还要满足最大
					dp[i] = dp[j] + 1;
					idx[i] = j;
				}
			}

			if (max < dp[i]) {
				max = dp[i];
				start = i; // 倒序push_back到返回结果集中
			}
		}

		// push_back res
		vector<int> res;
		for (int i = 0; i < max; ++i) {
			res.push_back(nums[start]); // 倒序
			start = idx[start];
		}
		return res;
	}
};


int main()
{
	Solution solution;
	vector<int> nums{ 1,2,4,6,8 };
	vector<int> res = solution.largestDivisibleSubset(nums);
	for (int r : res)
		cout << r << " ";
}

```


### [375\. Guess Number Higher or Lower II (Amazing)](https://leetcode.com/problems/guess-number-higher-or-lower-ii/)

Difficulty: **Medium**


We are playing the Guess Game. The game is as follows:

I pick a number from **1** to **n**. You have to guess which number I picked.

Every time you guess wrong, I'll tell you whether the number I picked is higher or lower.

However, when you guess a particular number x, and you guess wrong, you pay **$x**. You win the game when you guess the number I picked.

**Example:**

```
n = 10, I pick 8.

First round:  You guess 5, I tell you that it's higher. You pay $5.
Second round: You guess 7, I tell you that it's higher. You pay $7.
Third round:  You guess 9, I tell you that it's lower. You pay $9.

Game over. 8 is the number I picked.

You end up paying $5 + $7 + $9 = $21.
```

Given a particular **n ≥ 1**, find out how much money you need to have to guarantee a **win**.


#### Solution

Language: **C++**

```c++
#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

// https://leetcode-cn.com/problems/guess-number-higher-or-lower-ii/solution/cai-shu-zi-da-xiao-ii-by-leetcode/
class Solution
{
public:
	int getMoneyAmount(int n)
	{
		vector<vector<int>> dp(n + 1, vector<int>(n + 1, 0));
		for (int len = 2; len <= n; ++len)
			for (int start = 1; start <= n - len + 1; ++start)
			{
				int min_res = INT_MAX;
				for (int piv = start + (len - 1) / 2; piv < start + len - 1; ++piv)
				{
					int res = piv + max(dp[start][piv - 1], dp[piv + 1][start + len - 1]);
					min_res = min(min_res, res);
				}
				dp[start][start + len - 1] = min_res;
			}
		return dp[1][n];
	}
};

int main()
{
	Solution solution;
	cout << solution.getMoneyAmount(10) << endl;
}

```


### [376\. Wiggle Subsequence](https://leetcode.com/problems/wiggle-subsequence/)

Difficulty: **Medium**


A sequence of numbers is called a **wiggle sequence** if the differences between successive numbers strictly alternate between positive and negative. The first difference (if one exists) may be either positive or negative. A sequence with fewer than two elements is trivially a wiggle sequence.

For example, `[1,7,4,9,2,5]` is a wiggle sequence because the differences `(6,-3,5,-7,3)` are alternately positive and negative. In contrast, `[1,4,7,2,5]` and `[1,7,4,5,5]` are not wiggle sequences, the first because its first two differences are positive and the second because its last difference is zero.

Given a sequence of integers, return the length of the longest subsequence that is a wiggle sequence. A subsequence is obtained by deleting some number of elements (eventually, also zero) from the original sequence, leaving the remaining elements in their original order.

**Example 1:**

```
Input: [1,7,4,9,2,5]
Output: 6
Explanation: The entire sequence is a wiggle sequence.
```


**Example 2:**

```
Input: [1,17,5,10,13,15,10,5,16,8]
Output: 7
Explanation: There are several subsequences that achieve this length. One is [1,17,10,13,10,16,8].
```


**Example 3:**

```
Input: [1,2,3,4,5,6,7,8,9]
Output: 2
```

**Follow up:**  
Can you do it in O(_n_) time?


#### Solution

Language: **C++**

```c++
#include <vector>

#include <iostream>

using namespace std;

class Solution {
public:
	int wiggleMaxLength(vector<int>& nums) {
		int n = nums.size();
		if (n <= 1) return n;
		if (n == 2)
			if (nums[1] - nums[0]) return 2;
			else return 1;

		int last_diff = nums[1] - nums[0];
		int cnt = (last_diff != 0) ? 1 : 0; // count the number of differences
		for (int i = 2; i < n; ++i)
		{
			int cur_diff = nums[i] - nums[i - 1];
			if (cur_diff == 0) continue;
			if ((cur_diff > 0 && last_diff <= 0) || (cur_diff < 0 && last_diff >= 0))
			{
				++cnt;
				last_diff = cur_diff;
			}
		}
		return cnt + 1;
	}
};

int main()
{
	vector<int> nums1{ 3, 3, 3 };
	vector<int> nums2{ 0, 0 };
	vector<int> nums3{ 1,7,4,9,2,5 };
	vector<int> nums4{ 1,17,5,10,13,15,10,5,16,8 };
	vector<int> nums5{ 1,2,3,4,5,6,7,8,9 };
	Solution solution;
	cout << solution.wiggleMaxLength(nums1) << endl;
	cout << solution.wiggleMaxLength(nums2) << endl;
	cout << solution.wiggleMaxLength(nums3) << endl;
	cout << solution.wiggleMaxLength(nums4) << endl;
	cout << solution.wiggleMaxLength(nums5) << endl;
}

```

### [377\. Combination Sum IV](https://leetcode.com/problems/combination-sum-iv/)

Difficulty: **Medium**


Given an integer array with all positive numbers and no duplicates, find the number of possible combinations that add up to a positive integer target.

**Example:**

```
nums = [1, 2, 3]
target = 4

The possible combination ways are:
(1, 1, 1, 1)
(1, 1, 2)
(1, 2, 1)
(1, 3)
(2, 1, 1)
(2, 2)
(3, 1)

Note that different sequences are counted as different combinations.

Therefore the output is 7.
```

**Follow up:**  
What if negative numbers are allowed in the given array?  
How does it change the problem?  
What limitation we need to add to the question to allow negative numbers?


#### Solution

Language: **C++**

```c++
#include <iostream>
#include <vector>

using namespace std;

// recursion
class Solution {
private:
	vector<int> dict_;
	int dp(const vector<int>& nums, int target)
	{
		if (target < 0) return 0;
		if (dict_[target] != -1) return dict_[target];
		int ans = 0;
		for (const int num : nums)
			ans += dp(nums, target - num);
		return dict_[target] = ans;
	}
public:
	int combinationSum4(vector<int>& nums, int target) {
		dict_.resize(target + 1, -1);
		dict_[0] = 1;
		return dp(nums, target);
	}
};

int main()
{
	Solution solution;
	vector<int> nums{ 3, 33, 333 };
	cout << solution.combinationSum4(nums, 4) << endl;
}

```


### [413\. Arithmetic Slices](https://leetcode.com/problems/arithmetic-slices/)

Difficulty: **Medium**


A sequence of number is called arithmetic if it consists of at least three elements and if the difference between any two consecutive elements is the same.

For example, these are arithmetic sequence:

```
1, 3, 5, 7, 9
7, 7, 7, 7
3, -1, -5, -9
```

The following sequence is not arithmetic.

```
1, 1, 2, 5, 7
```

A zero-indexed array A consisting of N numbers is given. A slice of that array is any pair of integers (P, Q) such that 0 <= P < Q < N.

A slice (P, Q) of array A is called arithmetic if the sequence:  
A[P], A[p + 1], ..., A[Q - 1], A[Q] is arithmetic. In particular, this means that P + 1 < Q.

The function should return the number of arithmetic slices in the array A.

**Example:**

```
A = [1, 2, 3, 4]

return: 3, for 3 arithmetic slices in A: [1, 2, 3], [2, 3, 4] and [1, 2, 3, 4] itself.
```


#### Solution

Language: **C++**

```c++
class Solution {
public:
    int numberOfArithmeticSlices(vector<int>& A) {
        
    }
};
```


### [416\. Partition Equal Subset Sum](https://leetcode.com/problems/partition-equal-subset-sum/)

Difficulty: **Medium**


Given a **non-empty** array containing **only positive integers**, find if the array can be partitioned into two subsets such that the sum of elements in both subsets is equal.

**Note:**

1.  Each of the array element will not exceed 100.
2.  The array size will not exceed 200.

**Example 1:**

```
Input: [1, 5, 11, 5]

Output: true

Explanation: The array can be partitioned as [1, 5, 5] and [11].
```

**Example 2:**

```
Input: [1, 2, 3, 5]

Output: false

Explanation: The array cannot be partitioned into equal sum subsets.
```


#### Solution

Language: **C++**

```c++
class Solution {
public:
    bool canPartition(vector<int>& nums) {
        
    }
};
```


### [464\. Can I Win](https://leetcode.com/problems/can-i-win/)

Difficulty: **Medium**


In the "100 game," two players take turns adding, to a running total, any integer from 1..10\. The player who first causes the running total to reach or exceed 100 wins.

What if we change the game so that players cannot re-use integers?

For example, two players might take turns drawing from a common pool of numbers of 1..15 without replacement until they reach a total >= 100.

Given an integer `maxChoosableInteger` and another integer `desiredTotal`, determine if the first player to move can force a win, assuming both players play optimally.

You can always assume that `maxChoosableInteger` will not be larger than 20 and `desiredTotal` will not be larger than 300.

**Example**

```
Input:
maxChoosableInteger = 10
desiredTotal = 11

Output:
false

Explanation:
No matter which integer the first player choose, the first player will lose.
The first player can choose an integer from 1 up to 10.
If the first player choose 1, the second player can only choose integers from 2 up to 10.
The second player will win by choosing 10 and get a total = 11, which is >= desiredTotal.
Same with other integers chosen by the first player, the second player will always win.
```


#### Solution

Language: **C++**

```c++
class Solution {
public:
    bool canIWin(int maxChoosableInteger, int desiredTotal) {
        
    }
};
```



### [467\. Unique Substrings in Wraparound String](https://leetcode.com/problems/unique-substrings-in-wraparound-string/)

Difficulty: **Medium**


Consider the string `s` to be the infinite wraparound string of "abcdefghijklmnopqrstuvwxyz", so `s` will look like this: "...zabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcd....".

Now we have another string `p`. Your job is to find out how many unique non-empty substrings of `p` are present in `s`. In particular, your input is the string `p` and you need to output the number of different non-empty substrings of `p` in the string `s`.

**Note:** `p` consists of only lowercase English letters and the size of p might be over 10000.

**Example 1:**  

```
Input: "a"
Output: 1

Explanation: Only the substring "a" of string "a" is in the string s.
```

**Example 2:**  

```
Input: "cac"
Output: 2
Explanation: There are two substrings "a", "c" of string "cac" in the string s.
```

**Example 3:**  

```
Input: "zab"
Output: 6
Explanation: There are six substrings "z", "a", "b", "za", "ab", "zab" of string "zab" in the string s.
```


#### Solution

Language: **C++**

```c++
class Solution {
public:
    int findSubstringInWraproundString(string p) {
        
    }
};
```


### [474\. Ones and Zeroes](https://leetcode.com/problems/ones-and-zeroes/)

Difficulty: **Medium**


In the computer world, use restricted resource you have to generate maximum benefit is what we always want to pursue.

For now, suppose you are a dominator of **m** `0s` and **n** `1s` respectively. On the other hand, there is an array with strings consisting of only `0s` and `1s`.

Now your task is to find the maximum number of strings that you can form with given **m** `0s` and **n** `1s`. Each `0` and `1` can be used at most **once**.

**Note:**

1.  The given numbers of `0s` and `1s` will both not exceed `100`
2.  The size of given string array won't exceed `600`.

**Example 1:**

```
Input: Array = {"10", "0001", "111001", "1", "0"}, m = 5, n = 3
Output: 4

Explanation: This are totally 4 strings can be formed by the using of 5 0s and 3 1s, which are “10,”0001”,”1”,”0”
```

**Example 2:**

```
Input: Array = {"10", "0", "1"}, m = 1, n = 1
Output: 2

Explanation: You could form "10", but then you'd have nothing left. Better form "0" and "1".
```


#### Solution

Language: **C++**

```c++
class Solution {
public:
    int findMaxForm(vector<string>& strs, int m, int n) {
        
    }
};
```


### [486\. Predict the Winner](https://leetcode.com/problems/predict-the-winner/)

Difficulty: **Medium**


Given an array of scores that are non-negative integers. Player 1 picks one of the numbers from either end of the array followed by the player 2 and then player 1 and so on. Each time a player picks a number, that number will not be available for the next player. This continues until all the scores have been chosen. The player with the maximum score wins.

Given an array of scores, predict whether player 1 is the winner. You can assume each player plays to maximize his score.

**Example 1:**  

```
Input: [1, 5, 2]
Output: False
Explanation: Initially, player 1 can choose between 1 and 2\. If he chooses 2 (or 1), then player 2 can choose from 1 (or 2) and 5\. If player 2 chooses 5, then player 1 will be left with 1 (or 2). So, final score of player 1 is 1 + 2 = 3, and player 2 is 5\. Hence, player 1 will never be the winner and you need to return False.
```

**Example 2:**  

```
Input: [1, 5, 233, 7]
Output: True
Explanation: Player 1 first chooses 1\. Then player 2 have to choose between 5 and 7\. No matter which number player 2 choose, player 1 can choose 233.Finally, player 1 has more score (234) than player 2 (12), so you need to return True representing player1 can win.
```

**Note:**  

1.  1 <= length of the array <= 20\.
2.  Any scores in the given array are non-negative integers and will not exceed 10,000,000.
3.  If the scores of both players are equal, then player 1 is still the winner.


#### Solution

Language: **C++**

```c++
class Solution {
public:
    bool PredictTheWinner(vector<int>& nums) {
        
    }
};
```


### [494\. Target Sum](https://leetcode.com/problems/target-sum/)

Difficulty: **Medium**


You are given a list of non-negative integers, a1, a2, ..., an, and a target, S. Now you have 2 symbols `+` and `-`. For each integer, you should choose one from `+` and `-` as its new symbol.

Find out how many ways to assign symbols to make sum of integers equal to target S.

**Example 1:**  

```
Input: nums is [1, 1, 1, 1, 1], S is 3\.
Output: 5
Explanation:

-1+1+1+1+1 = 3
+1-1+1+1+1 = 3
+1+1-1+1+1 = 3
+1+1+1-1+1 = 3
+1+1+1+1-1 = 3

There are 5 ways to assign symbols to make the sum of nums be target 3.
```

**Note:**  

1.  The length of the given array is positive and will not exceed 20\.
2.  The sum of elements in the given array will not exceed 1000.
3.  Your output answer is guaranteed to be fitted in a 32-bit integer.


#### Solution

Language: **C++**

```c++
class Solution {
public:
    int findTargetSumWays(vector<int>& nums, int S) {
        
    }
};
```


### [516\. Longest Palindromic Subsequence](https://leetcode.com/problems/longest-palindromic-subsequence/)

Difficulty: **Medium**


Given a string s, find the longest palindromic subsequence's length in s. You may assume that the maximum length of s is 1000.

**Example 1:**  
Input:

```
"bbbab"
```

Output:

```
4
```

One possible longest palindromic subsequence is "bbbb".

**Example 2:**  
Input:

```
"cbbd"
```

Output:

```
2
```

One possible longest palindromic subsequence is "bb".

#### Solution

Language: **C++**

```c++
class Solution {
public:
    int longestPalindromeSubseq(string s) {
        
    }
};
```


### [523\. Continuous Subarray Sum](https://leetcode.com/problems/continuous-subarray-sum/)

Difficulty: **Medium**


Given a list of **non-negative** numbers and a target **integer** k, write a function to check if the array has a continuous subarray of size at least 2 that sums up to a multiple of **k**, that is, sums up to n*k where n is also an **integer**.

**Example 1:**

```
Input: [23, 2, 4, 6, 7],  k=6
Output: True
Explanation: Because [2, 4] is a continuous subarray of size 2 and sums up to 6.
```

**Example 2:**

```
Input: [23, 2, 6, 4, 7],  k=6
Output: True
Explanation: Because [23, 2, 6, 4, 7] is an continuous subarray of size 5 and sums up to 42.
```

**Note:**

1.  The length of the array won't exceed 10,000.
2.  You may assume the sum of all the numbers is in the range of a signed 32-bit integer.


#### Solution

Language: **C++**

```c++
class Solution {
public:
    bool checkSubarraySum(vector<int>& nums, int k) {
        
    }
};
```


### [576\. Out of Boundary Paths](https://leetcode.com/problems/out-of-boundary-paths/)

Difficulty: **Medium**


There is an **m** by **n** grid with a ball. Given the start coordinate **(i,j)** of the ball, you can move the ball to **adjacent** cell or cross the grid boundary in four directions (up, down, left, right). However, you can **at most** move **N** times. Find out the number of paths to move the ball out of grid boundary. The answer may be very large, return it after mod 10<sup>9</sup> + 7.

**Example 1:**

```
Input: m = 2, n = 2, N = 2, i = 0, j = 0
Output: 6
Explanation:

```

**Example 2:**

```
Input: m = 1, n = 3, N = 3, i = 0, j = 1
Output: 12
Explanation:

```

**Note:**

1.  Once you move the ball out of boundary, you cannot move it back.
2.  The length and height of the grid is in range [1,50].
3.  N is in range [0,50].


#### Solution

Language: **C++**

```c++
class Solution {
public:
    int findPaths(int m, int n, int N, int i, int j) {
        
    }
};
```


### [638\. Shopping Offers](https://leetcode.com/problems/shopping-offers/)

Difficulty: **Medium**


In LeetCode Store, there are some kinds of items to sell. Each item has a price.

However, there are some special offers, and a special offer consists of one or more different kinds of items with a sale price.

You are given the each item's price, a set of special offers, and the number we need to buy for each item. The job is to output the lowest price you have to pay for **exactly** certain items as given, where you could make optimal use of the special offers.

Each special offer is represented in the form of an array, the last number represents the price you need to pay for this special offer, other numbers represents how many specific items you could get if you buy this offer.

You could use any of special offers as many times as you want.

**Example 1:**  

```
Input: [2,5], [[3,0,5],[1,2,10]], [3,2]
Output: 14
Explanation:
There are two kinds of items, A and B. Their prices are $2 and $5 respectively.
In special offer 1, you can pay $5 for 3A and 0B
In special offer 2, you can pay $10 for 1A and 2B.
You need to buy 3A and 2B, so you may pay $10 for 1A and 2B (special offer #2), and $4 for 2A.
```

**Example 2:**  

```
Input: [2,3,4], [[1,1,0,4],[2,2,1,9]], [1,2,1]
Output: 11
Explanation:
The price of A is $2, and $3 for B, $4 for C.
You may pay $4 for 1A and 1B, and $9 for 2A ,2B and 1C.
You need to buy 1A ,2B and 1C, so you may pay $4 for 1A and 1B (special offer #1), and $3 for 1B, $4 for 1C.
You cannot add more items, though only $9 for 2A ,2B and 1C.
```

**Note:**  

1.  There are at most 6 kinds of items, 100 special offers.
2.  For each item, you need to buy at most 6 of them.
3.  You are **not** allowed to buy more items than you want, even if that would lower the overall price.


#### Solution

Language: **C++**

```c++
class Solution {
public:
    int shoppingOffers(vector<int>& price, vector<vector<int>>& special, vector<int>& needs) {
        
    }
};
```


### [1043\. Partition Array for Maximum Sum](https://leetcode.com/problems/partition-array-for-maximum-sum/)

Difficulty: **Medium**


Given an integer array `A`, you partition the array into (contiguous) subarrays of length at most `K`.  After partitioning, each subarray has their values changed to become the maximum value of that subarray.

Return the largest sum of the given array after partitioning.

**Example 1:**

```
Input: A = [1,15,7,9,2,5,10], K = 3
Output: 84
Explanation: A becomes [15,15,15,9,10,10,10]
```

**Note:**

1.  `1 <= K <= A.length <= 500`
2.  `0 <= A[i] <= 10^6`


#### Solution

Language: **C++**

```c++
#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

class Solution
{
public:
	int maxSumAfterPartitioning(vector<int>& A, int K)
	{
		int n = A.size();
		vector<int> dp(n + 1, 0);
		for (int i = 1; i <= n; ++i)
		{
			int m = INT_MIN;
			for (int k = 1; k <= min(i, K); ++k)
			{
				m = max(m, A[i - k]);
				dp[i] = max(dp[i], dp[i - k] + m * k);
			}
		}
		return dp[n];
	}
};

int main()
{
	vector<int> A{ 1, 15, 7, 9, 2, 5, 10 };
	Solution solution;
	cout << solution.maxSumAfterPartitioning(A, 3) << endl;
}

```
