---
layout:     post
title:      "Binary Tree"
date:       2019-7-9
author:     Tong
catalog: true
tags:
    - Tree
---

### Summary

1.


### [94. Binary Tree Inorder Traversal](https://leetcode.com/problems/binary-tree-inorder-traversal/)

#### Question

Given a binary tree, return the inorder traversal of its nodes' values.

__Example:__
```
Input: [1,null,2,3]
   1
    \
     2
    /
   3

Output: [1,3,2]
```

__Follow up:__ Recursive solution is trivial, could you do it iteratively?

#### Train of Thought

To iteratively solve this problem, we should use other helpers to store our roots so as to traverse left nodes firstly. One possible way is to use stack.

#### Solution
```cpp
#include <iostream>

#include <vector>

#include <stack>

using std::cout;
using std::endl;
using std::vector;
using std::stack;

struct TreeNode
{
	int val;
	TreeNode* left;
	TreeNode* right;
	TreeNode(int x) : val(x), left(NULL), right(NULL) {}
};

// Recursive solution
class Solution1 {
public:
	vector<int> inorderTraversal(TreeNode* root)
	{
		vector<int> vals;
		InorderTraversalTree(root, vals);
		return vals;
	}

	void InorderTraversalTree(TreeNode* root, vector<int>& vals)
	{
		if (!root)
			return;

		InorderTraversalTree(root->left, vals);
		vals.push_back(root->val);
		InorderTraversalTree(root->right, vals);
	}
};

// Iterative solution
class Solution2 {
public:
	vector<int> inorderTraversal(TreeNode* root)
	{
		vector<int> vals;
		stack<TreeNode*> nodes;

		TreeNode* node = root;

		while (node || !nodes.empty())
		{
			while (node)
			{
				nodes.push(node);
				node = node->left;
			}

			node = nodes.top();
			nodes.pop();
			vals.push_back(node->val);
			node = node->right;
		}

		return vals;
	}
};

int main()
{
	TreeNode node1(1);
	TreeNode node2(2);
	TreeNode node3(3);

	TreeNode* root = &node1;
	root->right = &node2;
	root->right->left = &node3;

	Solution2 solution;
	vector<int> vals = solution.inorderTraversal(root);
	for (auto& val : vals)
		cout << val << " ";
	cout << endl;
}
```
