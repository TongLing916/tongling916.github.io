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

1. Should we use queue or stack?

2. when is the current level finished?

### [Leetcode 102. Binary Tree Level Order Traversal](https://leetcode.com/problems/binary-tree-level-order-traversal/)

#### Question

Given a binary tree, return the level order traversal of its nodes' values. (ie, from left to right, level by level).

For example:
Given binary tree `[3,9,20,null,null,15,7]`,

```
    3
   / \
  9  20
    /  \
   15   7
```

return its level order traversal as:
```
[
  [3],
  [9,20],
  [15,7]
]
```

#### Train of Thought

Use a `queue` to store the nodes in each level. Read the size as the breadth we should use in the current level.

#### Solution
```cpp
#include <iostream>

#include <vector>

#include <queue>

using std::cout;
using std::endl;
using std::queue;
using std::vector;

struct TreeNode
{
	int val;
	TreeNode* left;
	TreeNode* right;
	TreeNode(int x) : val(x), left(NULL), right(NULL) {}
};

// iterative
class Solution1 {
public:
	vector<vector<int>> levelOrder(TreeNode* root) {
		vector<vector<int>> ret;
		queue<TreeNode*> nodes;
		if (!root)
			return ret;
		nodes.push(root);
		while (!nodes.empty())
		{
			vector<int> cur_lvl;
			int breadth = nodes.size();
			for (int i = 0; i < breadth; ++i)
			{
				TreeNode* cur = nodes.front();
				nodes.pop();
				cur_lvl.push_back(cur->val);
				if (cur->left) nodes.push(cur->left);
				if (cur->right) nodes.push(cur->right);
			}
			ret.push_back(cur_lvl);
		}
		return ret;
	}
};

std::ostream& operator<<(std::ostream& stream, const vector<vector<int>>& nums)
{
	for (const auto& num : nums)
		for (const auto& n : num)
			stream << n << " ";
	cout << endl;
	return stream;
}

int main()
{
	TreeNode t1(1);
	t1.left = &TreeNode(2);
	t1.right = &TreeNode(2);
	t1.left->right = &TreeNode(3);
	t1.right->right = &TreeNode(3);

	TreeNode t2(1);
	t2.left = &TreeNode(2);
	t2.right = &TreeNode(2);
	t2.left->left = &TreeNode(3);
	t2.left->right = &TreeNode(4);
	t2.right->left = &TreeNode(4);
	t2.right->right = &TreeNode(3);

	Solution1 solution;
	cout << solution.levelOrder(&t1) << endl;
	cout << solution.levelOrder(&t2) << endl;
}
```

### [Leetcode 103. Binary Tree Zigzag Level Order Traversal](https://leetcode.com/problems/binary-tree-zigzag-level-order-traversal/)

#### Question

Given a binary tree, return the zigzag level order traversal of its nodes' values. (ie, from left to right, then right to left for the next level and alternate between).

For example:
Given binary tree `[3,9,20,null,null,15,7]`,

```
    3
   / \
  9  20
    /  \
   15   7
```

return its level order traversal as:
```
[
  [3],
  [20,9],
  [15,7]
]
```

#### Train of Thought

Use a `queue` to store the nodes in each level. Read the size as the breadth we should use in the current level.

Use a new `left_to_right` to check how we should store the nodes in the next level.

#### Solution
```cpp
#include <iostream>

#include <vector>

#include <queue>

#include <stack>

using std::cout;
using std::endl;
using std::vector;
using std::stack;
using std::queue;

struct TreeNode
{
	int val;
	TreeNode* left;
	TreeNode* right;
	TreeNode(int x) : val(x), left(NULL), right(NULL) {}
};

// Idea: store the nodes in the next level in a stack, than add back to the queue
class Solution {
public:
	vector<vector<int>> zigzagLevelOrder(TreeNode* root) {
		vector<vector<int>> ret;
		if (!root)
			return ret;
		queue<TreeNode*> nodes;
		stack<TreeNode*> next_nodes;
		nodes.push(root);
		bool left_to_right = true;
		while (!nodes.empty())
		{
			vector<int> cur_lvl;
			while (!nodes.empty())
			{
				TreeNode* cur = nodes.front();
				nodes.pop();
				cur_lvl.push_back(cur->val);
				if (left_to_right)
				{
					if (cur->left)
						next_nodes.push(cur->left);
					if (cur->right)
						next_nodes.push(cur->right);
				}
				else
				{
					if (cur->right)
						next_nodes.push(cur->right);
					if (cur->left)
						next_nodes.push(cur->left);
				}
			}
			left_to_right = !left_to_right;
			while (!next_nodes.empty())
			{
				TreeNode* cur = next_nodes.top();
				next_nodes.pop();
				nodes.push(cur);
			}
			ret.push_back(cur_lvl);
		}
		return ret;
	}
};

std::ostream& operator<<(std::ostream& stream, const vector<vector<int>>& nums)
{
	for (const auto& num : nums)
		for (const auto& n : num)
			stream << n << " ";
	cout << endl;
	return stream;
}

int main()
{
	TreeNode t1(3);
	t1.left = &TreeNode(9);
	t1.right = &TreeNode(20);
	t1.right->right = &TreeNode(15);
	t1.right->right = &TreeNode(7);

	Solution solution;
	cout << solution.zigzagLevelOrder(&t1) << endl;
}
```


### [127. Word Ladder](https://leetcode.com/problems/word-ladder/)

#### Question

Given two words (_beginWord_ and _endWord_), and a dictionary's word list, find the length of shortest transformation sequence from _beginWord_ to _endWord_, such that:

1. Only one letter can be changed at a time.
2. Each transformed word must exist in the word list. Note that _beginWord_ is _not_ a transformed word.

__Note:__
- Return 0 if there is no such transformation sequence.
- All words have the same length.
- All words contain only lowercase alphabetic characters.
- You may assume no duplicates in the word list.
- You may assume _beginWord_ and _endWord_ are non-empty and are not the same.

__Example 1:__
```
Input:
beginWord = "hit",
endWord = "cog",
wordList = ["hot","dot","dog","lot","log","cog"]

Output: 5

Explanation: As one shortest transformation is "hit" -> "hot" -> "dot" -> "dog" -> "cog",
return its length 5.
```

__Example 2:__
```
Input:
beginWord = "hit"
endWord = "cog"
wordList = ["hot","dot","dog","lot","log"]

Output: 0

Explanation: The endWord "cog" is not in wordList, therefore no possible transformation.
```

#### Train of Thought

[花花酱 LeetCode 127. Word Ladder](https://www.youtube.com/watch?v=vWPCm69MSfs)

#### Solution
```cpp
#include <iostream>

#include <queue>

#include <string>

#include <unordered_set>

#include <vector>

using std::cout;
using std::endl;
using std::unordered_set;
using std::string;
using std::queue;
using std::vector;

// BFS
class Solution1 {
public:
	int ladderLength(string beginWord, string endWord, vector<string>& wordList) {
		unordered_set<string> dict(wordList.begin(), wordList.end());
		if (!dict.count(endWord)) return 0;

		queue<string> q;
		q.push(beginWord);

		int l = beginWord.length();
		int step = 0;

		while (!q.empty()) {
			++step;
			for (int size = q.size(); size > 0; size--) {
				string w = q.front();
				q.pop();
				for (int i = 0; i < l; i++) {
					char ch = w[i];
					for (int j = 'a'; j <= 'z'; j++) {
						w[i] = j;
						// Found the solution
						if (w == endWord) return step + 1;
						// Not in dict, skip it
						if (!dict.count(w)) continue;
						// Remove new word from dict
						dict.erase(w);
						// Add new word into queue
						q.push(w);
					}
					w[i] = ch;
				}
			}
		}
		return 0;
	}
};

// bidirectional BFS
class Solution {
public:
	int ladderLength(string beginWord, string endWord, vector<string>& wordList) {
		unordered_set<string> dict(wordList.begin(), wordList.end());
		if (!dict.count(endWord)) return 0;

		int l = beginWord.length();

		unordered_set<string> q1{ beginWord };
		unordered_set<string> q2{ endWord };

		int step = 0;

		while (!q1.empty() && !q2.empty()) {
			++step;

			// Always expend the smaller queue first
			if (q1.size() > q2.size())
				std::swap(q1, q2);

			unordered_set<string> q;

			for (string w : q1) {
				for (int i = 0; i < l; i++) {
					char ch = w[i];
					for (int j = 'a'; j <= 'z'; j++) {
						w[i] = j;
						if (q2.count(w)) return step + 1;
						if (!dict.count(w)) continue;
						dict.erase(w);
						q.insert(w);
					}
					w[i] = ch;
				}
			}

			std::swap(q, q1);
		}

		return 0;
	}
};

int main()
{
	string beginWord = "hit";
	string endWord = "cog";
	vector<string> wordList = { "hot","dot","dog","lot","log","cog" };

	Solution1 solution;
	cout << solution.ladderLength(beginWord, endWord, wordList) << endl;
}

```
