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


#### Solution

Language: **C++**

```c++
class Solution {
public:
    string longestPalindrome(string s) {
        
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
			trees.push_back(nullptr);  // importatn step
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
