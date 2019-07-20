---
layout:     post
title:      "Dynamic Programming"
date:       2019-7-18
author:     Tong
catalog: true
tags:
    - Algorithm
---

### Summary

1.


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
