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
class NumMatrix {
public:
    NumMatrix(vector<vector<int>>& matrix) {
        
    }
    
    int sumRegion(int row1, int col1, int row2, int col2) {
        
    }
};
​
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
class Solution {
public:
    int maxProfit(vector<int>& prices) {
        
    }
};
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
class Solution {
public:
    int coinChange(vector<int>& coins, int amount) {
        
    }
};
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
class Solution {
public:
    vector<int> countBits(int num) {
        
    }
};
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
class Solution {
public:
    int integerBreak(int n) {
        
    }
};
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
            cnt_i *= possible;
            cnt += cnt_i;
        }
        return cnt;
    }
};
```



### [368\. Largest Divisible Subset](https://leetcode.com/problems/largest-divisible-subset/)

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
class Solution {
public:
    vector<int> largestDivisibleSubset(vector<int>& nums) {
        
    }
};
```


### [375\. Guess Number Higher or Lower II](https://leetcode.com/problems/guess-number-higher-or-lower-ii/)

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
class Solution {
public:
    int getMoneyAmount(int n) {
        
    }
};
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
class Solution {
public:
    int wiggleMaxLength(vector<int>& nums) {
        
    }
};
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
class Solution {
public:
    int combinationSum4(vector<int>& nums, int target) {
        
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
