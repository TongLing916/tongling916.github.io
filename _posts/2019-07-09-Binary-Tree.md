---
layout:     post
title:      "Binary Tree / Heap"
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


### [144\. Binary Tree Preorder Traversal](https://leetcode.com/problems/binary-tree-preorder-traversal/)

Difficulty: **Medium**


Given a binary tree, return the _preorder_ traversal of its nodes' values.

**Example:**

```
Input: [1,null,2,3]
   1
    \
     2
    /
   3

Output: [1,2,3]
```

**Follow up:** Recursive solution is trivial, could you do it iteratively?


#### Solution

Language: **C++**

```c++

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

class Solution
{
public:
	vector<int> preorderTraversal(TreeNode* root)
	{
		vector<int> res;
		stack<TreeNode*> s;
		s.push(root);
		TreeNode* cur;
		while (s.size())
		{
			cur = s.top();
			s.pop();
			if (cur)
			{
				res.push_back(cur->val);
				s.push(cur->right);
				s.push(cur->left);
			}
		}

		return res;
	}
};

class Solution2
{
public:
	vector<int> preorderTraversal(TreeNode* root)
	{
		vector<int> res;
		preorderTraversal(root, res);
		return res;
	}

	void preorderTraversal(TreeNode* root, vector<int>& res)
	{
		if (!root) return;

		res.push_back(root->val);
		preorderTraversal(root->left, res);
		preorderTraversal(root->right, res);
	}
};

int main()
{
    std::cout << "Hello World!\n";
}

```


### [145\. Binary Tree Postorder Traversal](https://leetcode.com/problems/binary-tree-postorder-traversal/)

Difficulty: **Hard**


Given a binary tree, return the _postorder_ traversal of its nodes' values.

**Example:**

```
Input: [1,null,2,3]
   1
    \
     2
    /
   3

Output: [3,2,1]
```

**Follow up:** Recursive solution is trivial, could you do it iteratively?


#### Solution

Language: **C++**

```c++
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

class Solution {
public:
	vector<int> postorderTraversal(TreeNode* root)
	{
		vector<int> res;
		stack<TreeNode*> s;
		s.push(root);
		TreeNode* cur;

		while (s.size())
		{
			cur = s.top();
			s.pop();
			if (cur)
			{
				res.push_back(cur->val);
				if (cur->left) s.push(cur->left);
				if (cur->right) s.push(cur->right);
			}
		}

		reverse(res.begin(), res.end());
		return res;
	}
};

class Solution2 {
public:
	vector<int> postorderTraversal(TreeNode* root)
	{
		vector<int> res;
		postorderTraversal(root, res);
		return res;
	}

	void postorderTraversal(TreeNode* root, vector<int>& res)
	{
		if (!root) return;

		postorderTraversal(root->left, res);
		postorderTraversal(root->right, res);
		res.push_back(root->val);
	}
};

int main()
{
    std::cout << "Hello World!\n";
}

```


### [105\. Construct Binary Tree from Preorder and Inorder Traversal](https://leetcode.com/problems/construct-binary-tree-from-preorder-and-inorder-traversal/)

Difficulty: **Medium**


Given preorder and inorder traversal of a tree, construct the binary tree.

**Note:**  
You may assume that duplicates do not exist in the tree.

For example, given

```
preorder = [3,9,20,15,7]
inorder = [9,3,15,20,7]
```

Return the following binary tree:

```
    3
   / \
  9  20
    /  \
   15   7
```


#### Solution

Language: **C++**

```c++

#include <iostream>

#include <vector>

#include <unordered_map>

using std::vector;
using std::unordered_map;

struct TreeNode
{
	int val;
	TreeNode* left;
	TreeNode* right;
	TreeNode(int x) : val(x), left(NULL), right(NULL) {}
};

class Solution1
{
public:
	TreeNode* buildTreeHelper(vector<int>& preorder, int pre_lo, int pre_hi, vector<int>& inorder, int in_lo, int in_hi)
	{
		if (pre_hi - pre_lo < 0) return nullptr;

		int root_val = preorder[pre_lo];
		TreeNode* root = new TreeNode(root_val);
		int i = in_lo;
		for (; i <= in_hi; ++i)
			if (inorder[i] == root_val) break;
		root->left = buildTreeHelper(preorder, pre_lo + 1, pre_lo + i - in_lo, inorder, in_lo, i - 1);
		root->right = buildTreeHelper(preorder, pre_lo + i - in_lo + 1, pre_hi, inorder, i + 1, in_hi);
		return root;
	}

	TreeNode* buildTree(vector<int>& preorder, vector<int>& inorder)
	{
		return buildTreeHelper(preorder, 0, preorder.size() - 1, inorder, 0, inorder.size() - 1);
	}
};

class Solution2
{
private:
	unordered_map<int, int> dict;
public:
	TreeNode* buildTreeHelper(vector<int>& preorder, int pre_lo, int pre_hi, vector<int>& inorder, int in_lo, int in_hi)
	{
		if (pre_hi - pre_lo < 0) return nullptr;

		TreeNode* root = new TreeNode(preorder[pre_lo]);
		int i = dict[root->val];
		root->left = buildTreeHelper(preorder, pre_lo + 1, pre_lo + i - in_lo, inorder, in_lo, i - 1);
		root->right = buildTreeHelper(preorder, pre_lo + i - in_lo + 1, pre_hi, inorder, i + 1, in_hi);
		return root;
	}

	TreeNode* buildTree(vector<int>& preorder, vector<int>& inorder)
	{
		for (int i = 0; i < inorder.size(); ++i)
			dict[inorder[i]] = i;
		return buildTreeHelper(preorder, 0, preorder.size() - 1, inorder, 0, inorder.size() - 1);
	}
};

int main()
{
    std::cout << "Hello World!\n";
}

```



### [106\. Construct Binary Tree from Inorder and Postorder Traversal](https://leetcode.com/problems/construct-binary-tree-from-inorder-and-postorder-traversal/)

Difficulty: **Medium**


Given inorder and postorder traversal of a tree, construct the binary tree.

**Note:**  
You may assume that duplicates do not exist in the tree.

For example, given

```
inorder = [9,3,15,20,7]
postorder = [9,15,7,20,3]
```

Return the following binary tree:

```
    3
   / \
  9  20
    /  \
   15   7
```


#### Solution

Language: **C++**

```c++
 #include <iostream>

 #include <vector>

 #include <unordered_map>

 using std::vector;
 using std::unordered_map;

 struct TreeNode
 {
 	int val;
 	TreeNode* left;
 	TreeNode* right;
 	TreeNode(int x) : val(x), left(NULL), right(NULL) {}
 };

 class Solution {
 private:
 	TreeNode* buildTreeHelper(vector<int>& inorder, int in_lo, int in_hi, vector<int>& postorder, int post_lo, int post_hi)
 	{
 		if (post_lo > post_hi) return nullptr;
 		int root_val = postorder[post_hi];
 		TreeNode* root = new TreeNode(root_val);

 		int i = in_lo;
 		for (; i <= in_hi; ++i)
 			if (inorder[i] == root_val) break;

 		root->left = buildTreeHelper(inorder, in_lo, i - 1, postorder, post_lo, post_lo + i - in_lo - 1);
 		root->right = buildTreeHelper(inorder, i + 1, in_hi, postorder, post_lo + i - in_lo, post_hi - 1);
 		return root;
 	}
 public:
 	TreeNode* buildTree(vector<int>& inorder, vector<int>& postorder) {
 		return buildTreeHelper(inorder, 0, inorder.size() - 1, postorder, 0, postorder.size() - 1);
 	}
 };

 class Solution2
 {
 private:
 	unordered_map<int, int> dict;
 public:
 	TreeNode* buildTreeHelper(vector<int>& inorder, int in_lo, int in_hi, vector<int>& postorder, int post_lo, int post_hi)
 	{
 		if (post_hi < post_lo) return nullptr;
 		TreeNode* root = new TreeNode(postorder[post_hi]);
 		int i = dict[root->val];
 		root->left = buildTreeHelper(inorder, in_lo, i - 1, postorder, post_lo, post_lo + i - in_lo - 1);
 		root->right = buildTreeHelper(inorder, i + 1, in_hi, postorder, post_lo + i - in_lo, post_hi - 1);
 		return root;
 	}
 	TreeNode* buildTree(vector<int>& inorder, vector<int>& postorder)
 	{
 		for (int i = 0; i < inorder.size(); ++i)
 			dict[inorder[i]] = i;
 		return buildTreeHelper(inorder, 0, inorder.size() - 1, postorder, 0, postorder.size() - 1);
 	}
 };

 int main()
 {
     std::cout << "Hello World!\n";
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



### [373\. Find K Pairs with Smallest Sums](https://leetcode.com/problems/find-k-pairs-with-smallest-sums/)

Difficulty: **Medium**


You are given two integer arrays **nums1** and **nums2** sorted in ascending order and an integer **k**.

Define a pair **(u,v)** which consists of one element from the first array and one element from the second array.

Find the k pairs **(u<sub style="display: inline;">1</sub>,v<sub style="display: inline;">1</sub>),(u<sub style="display: inline;">2</sub>,v<sub style="display: inline;">2</sub>) ...(u<sub style="display: inline;">k</sub>,v<sub style="display: inline;">k</sub>)** with the smallest sums.

**Example 1:**

```
Input: nums1 = [1,7,11], nums2 = [2,4,6], k = 3
Output: [[1,2],[1,4],[1,6]]
Explanation: The first 3 pairs are returned from the sequence:
             [1,2],[1,4],[1,6],[7,2],[7,4],[11,2],[7,6],[11,4],[11,6]
```

**Example 2:**

```
Input: nums1 = [1,1,2], nums2 = [1,2,3], k = 2
Output: [1,1],[1,1]
Explanation: The first 2 pairs are returned from the sequence:
             [1,1],[1,1],[1,2],[2,1],[1,2],[2,2],[1,3],[1,3],[2,3]
```

**Example 3:**

```
Input: nums1 = [1,2], nums2 = [3], k = 3
Output: [1,3],[2,3]
Explanation: All possible pairs are returned from the sequence: [1,3],[2,3]
```


#### Solution

Language: **C++**

```c++
#include <iostream>
#include <vector>
#include <queue>
#include <utility>

using namespace std;

typedef pair<int, int> PII;

class Solution
{
public:
	vector<vector<int>> kSmallestPairs(vector<int>& nums1, vector<int>& nums2, int k)
	{
		vector<vector<int>> res;
		if (nums1.empty() || nums2.empty() || k <= 0) return res;
		auto comp = [&nums1, &nums2](PII& a, PII& b) {
			return nums1[a.first] + nums2[a.second] > nums1[b.first] + nums2[b.second];
		};
		priority_queue<PII, vector<PII>, decltype(comp)> min_pq(comp);
		min_pq.push({ 0, 0 });
		while (k-- > 0 && min_pq.size())
		{
			PII ids = min_pq.top();
			min_pq.pop();
			res.push_back(vector<int>{nums1[ids.first], nums2[ids.second]});
			if (ids.first + 1 < nums1.size()) min_pq.push({ ids.first + 1, ids.second });
			if (ids.first == 0 && ids.second + 1 < nums2.size()) min_pq.push({ ids.first, ids.second + 1 });
		}
		return res;
	}
};

ostream& operator<< (ostream& stream, const vector<vector<int>>& nums)
{
	for (const vector<int>& num : nums)
		stream << num[0] << " " << num[1] << endl;
	return stream;
}

int main()
{
	vector<int> nums1{ 1, 7, 11 };
	vector<int> nums2{ 2, 4, 6 };
	Solution solution;
	cout << solution.kSmallestPairs(nums1, nums2, 3) << endl;
}
```


### [671\. Second Minimum Node In a Binary Tree](https://leetcode.com/problems/second-minimum-node-in-a-binary-tree/)

Difficulty: **Easy**


Given a non-empty special binary tree consisting of nodes with the non-negative value, where each node in this tree has exactly `two` or `zero` sub-node. If the node has two sub-nodes, then this node's value is the smaller value among its two sub-nodes. More formally, the property `root.val = min(root.left.val, root.right.val)` always holds.

Given such a binary tree, you need to output the **second minimum** value in the set made of all the nodes' value in the whole tree.

If no such second minimum value exists, output -1 instead.

**Example 1:**

```
Input:
    2
   / \
  2   5
     / \
    5   7

Output: 5
Explanation: The smallest value is 2, the second smallest value is 5.
```

**Example 2:**

```
Input:
    2
   / \
  2   2

Output: -1
Explanation: The smallest value is 2, but there isn't any second smallest value.
```


#### Solution

Language: **C++**

```c++
struct TreeNode
{
	int val;
	TreeNode* left;
	TreeNode* right;
	TreeNode(int x) : val(x), left(NULL), right(NULL) {}
};

class Solution {
public:
	int findSecondMinimumValue(TreeNode* root)
	{
		first_ = LONG_MAX;
		second_ = LONG_MAX;
		dfs(root);
		return second_ == LONG_MAX ? -1 : second_;
	}

private:
	void dfs(TreeNode* root)
	{
		if (!root) return;
		if (root->val < first_)
		{
			second_ = first_;
			first_ = root->val;
		}
		else if (root->val != first_ && root->val < second_)
		{
			second_ = root->val;
		}
		dfs(root->left);
		dfs(root->right);
	}


	long first_;
	long second_;
};
```


### [703\. Kth Largest Element in a Stream](https://leetcode.com/problems/kth-largest-element-in-a-stream/)

Difficulty: **Easy**


Design a class to find the **k**th largest element in a stream. Note that it is the kth largest element in the sorted order, not the kth distinct element.

Your `KthLargest` class will have a constructor which accepts an integer `k` and an integer array `nums`, which contains initial elements from the stream. For each call to the method `KthLargest.add`, return the element representing the kth largest element in the stream.

**Example:**

```
int k = 3;
int[] arr = [4,5,8,2];
KthLargest kthLargest = new KthLargest(3, arr);
kthLargest.add(3);   // returns 4
kthLargest.add(5);   // returns 5
kthLargest.add(10);  // returns 5
kthLargest.add(9);   // returns 8
kthLargest.add(4);   // returns 8
```

**Note:**  
You may assume that `nums`' length ≥ `k-1` and `k` ≥ 1.


#### Solution

Language: **C++**

```c++
#include <iostream>
#include <vector>
#include <queue>

using namespace std;

class KthLargest {
private:
	priority_queue<int, vector<int>, greater<int>> pq_; // min heap
	int size_;

public:
	KthLargest(int k, vector<int>& nums)
	{
		size_ = k;
		for (const int num : nums)
		{
			pq_.push(num);
			if (pq_.size() > k) pq_.pop(); // remove the top element
		}
	}

	int add(int val)
	{
		pq_.push(val);
		if (pq_.size() > size_) pq_.pop();
		return pq_.top();
	}
};

/**
 * Your KthLargest object will be instantiated and called as such:
 * KthLargest* obj = new KthLargest(k, nums);
 * int param_1 = obj->add(val);
 */

int main()
{
	vector<int> nums{ 4, 5, 8, 2 };
	KthLargest* obj = new KthLargest(3, nums);
	cout << obj->add(3) << endl;
	cout << obj->add(5) << endl;
}

```
