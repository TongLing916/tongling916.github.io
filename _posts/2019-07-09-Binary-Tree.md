---
layout:     post
title:      "Binary Tree"
date:       2019-7-9
author:     Tong
catalog: true
tags:
    - Algorithm
---

### Summary

1. If we want to iteratively traverse a tree, we can use `stack<TreeNode*>` to store the parent nodes for later use.

2. For `stack`, `pop()` is `void` and does not provide a return value. `top()` provides the top element.


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

### [236. Lowest Common Ancestor of a Binary Tree](https://leetcode.com/problems/lowest-common-ancestor-of-a-binary-tree/)

#### Question

Given a binary tree, find the lowest common ancestor (LCA) of two given nodes in the tree.

According to the definition of LCA on Wikipedia: “The lowest common ancestor is defined between two nodes p and q as the lowest node in T that has both p and q as descendants (where we allow __a node to be a descendant of itself__).”

Given the following binary tree:  root = [3,5,1,6,2,0,8,null,null,7,4]

```
      3
     / \
    5    1
   / \  / \
  6  2  0  8
    / \
   7   4
```

__Example 1:__
```
Input: root = [3,5,1,6,2,0,8,null,null,7,4], p = 5, q = 1
Output: 3
Explanation: The LCA of nodes 5 and 1 is 3.
```

__Example 2:__
```
Input: root = [3,5,1,6,2,0,8,null,null,7,4], p = 5, q = 4
Output: 5
Explanation: The LCA of nodes 5 and 4 is 5, since a node can be a descendant of itself according to the LCA definition.
```

__Note:__
- All of the nodes' values will be unique.
- p and q are different and both values will exist in the binary tree.

#### Train of Thought

Consider the extreme cases: If the `root` is just the `p` or `q`, we can directly return `root`. (or `root` is `nullptr`).

If not, try to search in the left and right subtrees. If no node is found in the left subtree, that means the common ancestor is in the right subtree. If no node is found in the right subtree, that means the common ancestor is in the left subtree. If one node is found in the left one and the other is found in the right one, return the root.

#### Solution
```cpp
#include <iostream>

using std::cout;
using std::endl;

struct TreeNode
{
	int val;
	TreeNode* left;
	TreeNode* right;
	TreeNode(int x) : val(x), left(NULL), right(NULL) {}
};

class Solution1 {
public:
	TreeNode* lowestCommonAncestor(TreeNode* root, TreeNode* p, TreeNode* q) {
		if (!root || root == p || root == q)
			return root;
		TreeNode* left = lowestCommonAncestor(root->left, p, q);
		TreeNode* right = lowestCommonAncestor(root->right, p, q);
		if (!left)
			return right;
		if (!right)
			return left;
		return root;
	}
};

int main()
{
	TreeNode node0(0);
	TreeNode node1(1);
	TreeNode node2(2);
	TreeNode node3(3);
	TreeNode node4(4);
	TreeNode node5(5);
	TreeNode node6(6);
	TreeNode node7(7);
	TreeNode node8(8);

	TreeNode* root = &node3;
	root->left = &node5;
	root->right = &node1;
	root->left->left = &node6;
	root->left->right = &node2;
	root->left->right->left = &node7;
	root->left->right->right = &node4;
	root->right->left = &node0;
	root->right->right = &node8;

	Solution1 solution;
	cout << solution.lowestCommonAncestor(root, &node5, &node4)->val << endl;
}
```
