---
layout:     post
title:      "Binary Search"
date:       2019-7-7
author:     Tong
catalog: true
tags:
    - Algorithm
---

### Summary

1. Cast the Integer number to a `long int`. Then, use `<<` to speed up the computation. (See `29. Divide Two Integers` and `50. Pow(x, n)`)


### [29. Divide Two Integers](https://leetcode.com/problems/divide-two-integers/)

#### Question

Given two integers dividend and divisor, divide two integers without using multiplication, division and mod operator.

Return the quotient after dividing dividend by divisor.

The integer division should truncate toward zero.

__Example 1:__
```
Input: dividend = 10, divisor = 3
Output: 3
```

__Example 2:__
```
Input: dividend = 7, divisor = -3
Output: -2
```

__Note:__
Both dividend and divisor will be 32-bit signed integers.
The divisor will never be 0.
Assume we are dealing with an environment which could only store integers within the 32-bit signed integer range: $$[−2^{31},  2^{31} − 1]$$. For the purpose of this problem, assume that your function returns $$2^{31} − 1$$ when the division result overflows.

#### Train of Thought

Keep always the extreme case in mind: -2147483648 / -1. Only in this case, overflow can happen.

To get the result, we need to count how many times divisor can be substracted from the dividend. To avoid overflow, we should convert Integer to Long Integer.

Then, due to the fact that we cannot use x, /, %, we should consider another operator to speed up the computation. Here, we can use `<<` to double the value.

#### Solution
```cpp
#include <iostream>

using std::cout;
using std::endl;

class Solution {
public:
	int divide(int dividend, int divisor)
	{
		if (divisor == INT_MIN)
		{
			if (dividend == INT_MIN)
				return 1;
			else
				return 0;
		}

		if (dividend == INT_MIN && divisor == -1)
			return INT_MAX;

		bool is_positive = (dividend > 0 && divisor > 0) || (dividend < 0 && divisor < 0) ? true : false;

		long int dividend_l = dividend;
		long int divisor_l = divisor;

		dividend_l = abs(dividend_l);
		divisor_l = abs(divisor_l);

		long int res = 0;

		while (dividend_l >= divisor_l)
		{
			long int tmp_res = 1;
			long int tmp_divisor = divisor_l;
			while ((tmp_divisor << 1) < dividend_l)
			{
				tmp_divisor <<= 1;
				tmp_res <<= 1;
			}

			dividend_l -= tmp_divisor;
			res += tmp_res;
		}

		if (is_positive)
			return res;
		else
			return -res;
	}
};

int main()
{
	Solution solution;
	cout << solution.divide(INT_MIN, -1) << endl;
	cout << solution.divide(INT_MIN, 1) << endl;
}

```

### [33. Search in Rotated Sorted Array](https://leetcode.com/problems/search-in-rotated-sorted-array/)

#### Question

Suppose an array sorted in ascending order is rotated at some pivot unknown to you beforehand.

(i.e., [0,1,2,4,5,6,7] might become [4,5,6,7,0,1,2]).

You are given a target value to search. If found in the array return its index, otherwise return -1.

You may assume no duplicate exists in the array.

Your algorithm's runtime complexity must be in the order of O(log n).

__Example 1:__
```
Input: nums = [4,5,6,7,0,1,2], target = 0
Output: 4
```

__Example 2:__
```
Input: nums = [4,5,6,7,0,1,2], target = 3
Output: -1
```

#### Train of Thought

__Solution 1:__

Because the sorted array is rotated, we can find the smallest element firstly to do a binary search. The way to find the pivot is also a binary search by comparing `nums[mid]` and `nums[hi]`.

After finding the pivot, we can iteratively find the median value, doing a binary search to check if the target exists.


__Solution 2:__

Without finding the pivot, we can directly do a binary search in the rotated array. However, we need to check the `nums[lo]` and `nums[mid]`.


#### Solution
```cpp
#include <iostream>

#include <vector>

using std::cout;
using std::endl;
using std::vector;

class Solution1 {
public:
	int search(vector<int>& nums, int target) {
		int n = nums.size();
		int lo = 0;
		int hi = n - 1;
		while (lo < hi)
		{
			int mid = lo + (hi - lo) / 2;
			if (nums[mid] > nums[hi])
				lo = mid + 1;
			else
				hi = mid;
		}

		int pivot = lo;
		lo = 0;
		hi = n - 1;
		while (lo <= hi)
		{
			int mid = lo + (hi - lo) / 2;
			int rMid = (mid + pivot) % n;
			if (nums[rMid] == target)
				return rMid;
			else if (nums[rMid] < target)
				lo = mid + 1;
			else
				hi = mid - 1;
		}

		return -1;
	}
};

class Solution2 {
public:
	int search(vector<int>& nums, int target)
	{
		int lo = 0;
		int hi = nums.size() - 1;

		while (lo <= hi)
		{
			int mid = lo + (hi - lo) / 2;
			if (nums[mid] == target)
			{
				return mid;
			}
			else if (nums[mid] < target)
			{
				if (nums[mid] < nums[lo] && nums[lo] <= target)
					hi = mid - 1;
				else
					lo = mid + 1;
			}
			else
			{
				if (nums[lo] <= nums[mid] && nums[lo] > target)
					lo = mid + 1;
				else
					hi = mid - 1;
			}
		}

		return -1;
	}
};

int main()
{
	vector<int> test1{ 4,5,6,7,0,1,2 };
	vector<int> test2{ 1,3 };
	vector<int> test3{ 3,1 };

	Solution1 solution;
	cout << solution.search(test1, 0) << endl;
	cout << solution.search(test1, 3) << endl;
	cout << solution.search(test2, 3) << endl;
	cout << solution.search(test3, 1) << endl;
}

```

### [81. Search in Rotated Sorted Array II](https://leetcode.com/problems/search-in-rotated-sorted-array-ii/)

#### Question

Suppose an array sorted in ascending order is rotated at some pivot unknown to you beforehand.

(i.e., `[0,0,1,2,2,5,6]` might become `[2,5,6,0,0,1,2]`).

You are given a target value to search. If found in the array return `true`, otherwise return `false`.

__Example 1:__
```
Input: nums = [2,5,6,0,0,1,2], target = 0
Output: true
```

__Example 2:__
```
Input: nums = [2,5,6,0,0,1,2], target = 3
Output: false
```

__Follow up:__
- This is a follow up problem to `Search in Rotated Sorted Array`, where `nums` may contain duplicates.
- Would this affect the run-time complexity? How and why?

#### Train of Thought

Because of the existence of duplicates, we cannot use the old way to find the pivot.

Consider the extreme cases `[1,1,3,1]` to find `3`.

__Solution 1:__

We can directly start to search the target using a modified binary search. The caution should be taken if the `nums[mid] != target`. If not equal, we need to check the `nums[mid]` and `nums[hi]` to deduce where the pivot is.

If `nums[mid] < nums[hi]`, the pivot must be in `[lo, mid]`. Therefore, only when `nums[mid] < target <= nums[hi]` have we the chance to find the target in `[mid + 1, hi]`. Otherwise, we need to search it in `[lo, mid - 1]`.

If `nums[mid] > nums[hi]`, the pivot must be in `[mid, hi]`. Therefore, only when `nums[lo] <= target < nums[mid]` have we the chance to find the target in `[lo, mid - 1]`. Otherwise, we need to search it in `[mid + 1, hi]`.

__Solution2:__

Traverse the whole array to find the pivot, either the first element or the first time `nums[pivot - 1] > nums[pivot]`. Then, everything is the same as before.

#### Solution
```cpp
#include <iostream>

#include <vector>

using std::cout;
using std::endl;
using std::vector;

class Solution1 {
public:
	bool search(vector<int>& nums, int target) {
		int n = nums.size();

		int lo = 0;
		int hi = n - 1;
		while (lo <= hi)
		{
			int mid = lo + (hi - lo) / 2;

			if (nums[mid] == target)
				return true;

			if (nums[mid] < nums[hi])
			{
				if (nums[mid] < target && nums[hi] >= target)
					lo = mid + 1;
				else
					hi = mid - 1;
			}
			else if (nums[mid] > nums[hi])
			{
				if (nums[lo] <= target && nums[mid] > target)
					hi = mid - 1;
				else
					lo = mid + 1;
			}
			else
				--hi;
		}

		return false;
	}
};

class Solution2 {
public:
	bool search(vector<int>& nums, int target) {
		int n = nums.size();
		if (n == 0)
			return false;

		// Because we have duplicates, we cannot use the old way to find the pivot
		int pivot = 0;
		for (int i = 1; i < n; ++i)
			if ((nums[i] < nums[pivot]) || (nums[i] == nums[pivot] && nums[i - 1] > nums[i]))
				pivot = i;

		int lo = 0;
		int hi = n - 1;
		while (lo <= hi)
		{
			int mid = lo + (hi - lo) / 2;
			int r_mid = (pivot + mid) % n;

			if (nums[r_mid] == target)
				return true;
			else if (nums[r_mid] > target)
				hi = mid - 1;
			else
				lo = mid + 1;
		}

		return false;
	}
};

int main()
{
	vector<int> test1{ 4,5,6,7,0,1,2 };
	vector<int> test2{ 1,1,3,1 };
	vector<int> test3{ 1,3,1,1 };

	Solution1 solution;
	cout << solution.search(test1, 0) << endl;
	cout << solution.search(test1, 3) << endl;
	cout << solution.search(test2, 3) << endl;
	cout << solution.search(test3, 3) << endl;
}

```


### [34. Find First and Last Position of Element in Sorted Array](https://leetcode.com/problems/find-first-and-last-position-of-element-in-sorted-array/)

#### Question

Given an array of integers `nums` sorted in ascending order, find the starting and ending position of a given `target` value.

Your algorithm's runtime complexity must be in the order of O(log n).

If the target is not found in the array, return `[-1, -1]`.

__Example 1:__
```
Input: nums = [5,7,7,8,8,10], target = 8
Output: [3,4]
```

__Example 2:__
```
Input: nums = [5,7,7,8,8,10], target = 6
Output: [-1,-1]
```

#### Train of Thought

__Solution 1:__

Firstly, we can use a binary search to find a target at `i`. If no target is found, we directly return `[-1, -1]`. If found, we just need to find the first value in `[lo, i]` and the last target in `[i, hi]`.

__Solution 2:__

We can also directly seek the lower bound of `target` and `target + 1`. If the `target` is  at `i`  and `target + 1`  at `j`, then we return `[i, j - 1]`. However, before returning values, we need to check if these values are valid: 1) if the `i` is beyond the size of the array. 2) if `nums[i]` is equal to the `target`.

#### Solution
```cpp
#include <iostream>

#include <vector>

using std::cout;
using std::endl;
using std::vector;

class Solution1 {
public:
	vector<int> searchRange(vector<int>& nums, int target) {
		int lo = 0;
		int hi = nums.size() - 1;

		vector<int> ret;

		while (lo <= hi)
		{
			int mid = lo + (hi - lo) / 2;
			if (nums[mid] == target)
			{
				ret.push_back(mid);
				ret.push_back(mid);
				break;
			}
			else if (nums[mid] < target)
			{
				lo = mid + 1;
			}
			else
			{
				hi = mid - 1;
			}
		}

		if (ret.size() == 0)
		{
			ret.push_back(-1);
			ret.push_back(-1);
		}
		else
		{
			// find first
			lo = 0;
			hi = ret[0];
			while (lo <= hi)
			{
				int mid = lo + (hi - lo) / 2;
				if (nums[mid] == target)
				{
					ret[0] = mid;
					hi = mid - 1;
				}
				else
					lo = mid + 1;
			}
			ret[0] = lo;

			// find last
			lo = ret[1];
			hi = nums.size() - 1;
			while (lo <= hi)
			{
				int mid = lo + (hi - lo) / 2;
				if (nums[mid] <= target)
				{
					ret[1] = mid;
					lo = mid + 1;
				}
				else
					hi = mid - 1;
			}
		}

		return ret;
	}
};

class Solution2 {
public:
	void searchLowerBound(int& i, vector<int>& nums, int target) {
		int lo = 0;
		int hi = nums.size();
		while (lo < hi)
		{
			int mid = lo + (hi - lo) / 2;
			if (nums[mid] >= target)
				hi = mid;
			else
				lo = mid + 1;
		}
		i = lo;
	}

	vector<int> searchRange(vector<int>& nums, int target) {
		int i1 = -1;
		int i2 = -1;

		searchLowerBound(i1, nums, target);
		searchLowerBound(i2, nums, target + 1);

		if (i1 == nums.size() || nums[i1] != target)
			return { -1, -1 };

		return { i1, i2 - 1 };
	}
};

int main()
{
	vector<int> test1{ 5,7,7,8,8,10 };

	Solution1 solution;
	vector<int> res = solution.searchRange(test1, 8);
	for (auto e : res)
		cout << e << " ";
	cout << endl;
}

```

### [50. Pow(x, n)](https://leetcode.com/problems/powx-n/)

#### Question

Implement pow(x, n), which calculates x raised to the power n($$x^n$$).

__Example 1:__
```
Input: 2.00000, 10
Output: 1024.00000
```

__Example 2:__
```
Input: 2.10000, 3
Output: 9.26100
```

__Example 3:__
```
Input: 2.00000, -2
Output: 0.25000
Explanation: $$2^{-2} = 1/2^2 = 1/4 = 0.25$$
```

__Note:__
- -100.0 < x < 100.0
- n is a 32-bit signed integer, within the range [$$−2^{31}$$, $$2^{31} − 1$$]

#### Train of Thought

As the question `29. Divide Two Integers`, we can cast the Integer number to a `long int`. Then, use `<<` to speed up the computation.

#### Solution
```cpp
#include <iostream>

#include <vector>

using std::cout;
using std::endl;
using std::vector;

class Solution {
public:
	double myPow(double x, int n)
	{
		double res = 1;
		bool is_positive = n > 0 ? true : false;

		long n_l = n;
		n_l = abs(n_l);

		while (n_l != 0)
		{
			long i = 1;
			double tmp_res = x;
			while ((i << 1) < n_l)
			{
				tmp_res * = tmp_res;
				i <<= 1;
			}
			n_l -= i;
			res *= tmp_res;
		}

		res = is_positive ? res : 1 / res;
		return res;
	}
};

int main()
{
	Solution solution;
	cout << solution.myPow(0.00001, 2147483647) << endl;   // bound case
	cout << solution.myPow(1.00000, -2147483648) << endl;
}
```


### [69. Sqrt(x)](https://leetcode.com/problems/sqrtx/)

#### Question

Implement `int sqrt(int x)`.

Compute and return the square root of x, where x is guaranteed to be a non-negative integer.

Since the return type is an integer, the decimal digits are truncated and only the integer part of the result is returned.

__Example 1:__
```
Input: 4
Output: 2
```

__Example 2:__
```
Input: 8
Output: 2
Explanation: The square root of 8 is 2.82842..., and since
             the decimal part is truncated, 2 is returned.
```

#### Train of Thought

Be careful about the stop condition when using binary search.

You can also use [Newton method](https://blog.csdn.net/wumuzi520/article/details/7026808).


#### Solution
```cpp
#include <iostream>

#include <vector>

#include <stdexcept>

using std::cout;
using std::endl;
using std::vector;

class Solution1 {
public:
	int mySqrt(int x)
	{
		try {
			if (x < 0)
				throw std::invalid_argument("x must be greater than 0");

			long lo = 0;
			long hi = x;

			while (lo < hi)
			{
				long mid = lo + (hi - lo) / 2;
				if (mid * mid == x)
					return mid;
				else if (mid * mid < x)
					lo = mid + 1;
				else
					hi = mid - 1;
			}

			if (lo * lo > x)
				return lo - 1;
			else
				return lo;
		}
		catch (std::exception& e)
		{
			std::cerr << "exception: " << e.what() << std::endl;
		}
	}
};


// https://blog.csdn.net/wumuzi520/article/details/7026808
class Solution2 {
public:
	int mySqrt(int x) {
		double precision = 0.00001;
		double guess = x;
		while (guess * guess - x > 0.00001)
			guess = 0.5 * (guess + x / guess);
		return (int)guess;
	}
};

int main()
{
	Solution1 solution;
	cout << solution.mySqrt(-1) << endl;
	cout << solution.mySqrt(1) << endl;
	cout << solution.mySqrt(INT_MAX) << endl;
}

```

### [162. Find Peak Element](https://leetcode.com/problems/find-peak-element/)

#### Question

A peak element is an element that is greater than its neighbors.

Given an input array `nums`, where `nums[i] ≠ nums[i+1]`, find a peak element and return its index.

The array may contain multiple peaks, in that case return the index to any one of the peaks is fine.

You may imagine that `nums[-1] = nums[n] = -∞`.

__Example1:__
```
Input: nums = [1,2,3,1]
Output: 2
Explanation: 3 is a peak element and your function should return the index number 2.
```

__Example2:__
```
Input: nums = [1,2,1,3,5,6,4]
Output: 1 or 5
Explanation: Your function can return either index number 1 where the peak element is 2,
             or index number 5 where the peak element is 6.
```

__Note:__
Your solution should be in logarithmic complexity.

#### Train of Thought

First of all, we need to know that there could be three possible positions of the peak element.

(1) Elements in descending order, e.g. `[5,4,3,2,1]`. Then, the first element is the peak element, because `nums[0] > nums[1]` (we do not need to consider `nums[-1]`).

(2) Elements in ascending order, e.g. `[1,2,3,4,5]`. Then, the last element is the peak element, because `nums[4] > nums[3]` (we do not need to consider `nums[5]`).

(3) Elements in random order, e.g. `[1,3,2,4,5]`. Then, the first element (assuming at `i`), which fulfills `nums[i] > nums[i+1]`, is our peak element.

Therefore, no matter in which case, we just need to find the first element (assuming at `i`), which fulfills `nums[i] > nums[i+1]`. To achieve this, we can use binary seach.

#### Solution
```cpp
#include <iostream>

#include <vector>

#include <algorithm>

using std::vector;
using std::cout;
using std::endl;

class Solution
{
public:
	int findPeakElement(vector<int>& nums)
	{
		int n = nums.size();
		int lo = 0;
		int hi = n - 1;

		while (lo < hi)
		{
			int mid = lo + (hi - lo) / 2;
			if (nums[mid] < nums[mid + 1])
				lo = mid + 1;
			else
				hi = mid;
		}

		return lo;
	}
};

int main()
{
	vector<int> test1{ 1,2,3,1 };
	Solution solution;
	cout << solution.findPeakElement(test1) << endl;
}

```


### [209. Minimum Size Subarray Sum](https://leetcode.com/problems/minimum-size-subarray-sum/)

#### Question

Given an array of n positive integers and a positive integer `s`, find the minimal length of a __contiguous__ subarray of which the sum ≥ `s`. If there isn't one, return 0 instead.

__Example:__
```
Input: s = 7, nums = [2,3,1,2,4,3]
Output: 2
Explanation: the subarray [4,3] has the minimal length under the problem constraint.
```

__Follow up:__
If you have figured out the O(n) solution, try coding another solution of which the time complexity is O(nlogn).

#### Train of Thought

To check the mininum number array, we need to traverse the whole array one time. At the same time when the index increases, we need to delete the starting elements if the sum already surpasses the target `s`. Then, check if the result length is smaller the minimum length so far. If yes, update the minimum length.

#### Solution
```cpp
class Solution {
public:
    int minSubArrayLen(int s, vector<int>& nums) {
        int n = nums.size();
        int sum = 0;

        int min_len = n + 1;

        int lo = 0;
        for (int i = 0; i < n; ++i)
        {
            sum += nums[i];

            if (sum >= s)
            {
                while (sum >= s)
                {
                    sum -= nums[lo];
                    ++lo;
                }

                min_len = (i - lo + 2) < min_len ? (i - lo + 2) : min_len;
            }
        }

        return min_len == n + 1 ? 0 : min_len;
    }
};
```

### [222. Count Complete Tree Nodes](https://leetcode.com/problems/count-complete-tree-nodes/)

#### Question

Given a __complete__ binary tree, count the number of nodes.

__Note:__
Definition of a complete binary tree from [Wikipedia](https://en.wikipedia.org/wiki/Binary_tree#Types_of_binary_trees):
In a complete binary tree every level, except possibly the last, is completely filled, and all nodes in the last level are as far left as possible. It can have between $$1$$ and $$2^h$$ nodes inclusive at the last level $$h$$.

__Example:__
```
Input:
    1
   / \
  2   3
 / \  /
4  5 6

Output: 6
```

#### Train of Thought

In a complete binary tree every level, except possibly the last, is completely filled, and all nodes in the last level are as far left as possible.

A perfect binary tree is a binary tree in which all interior nodes have two children and all leaves have the same depth or same level

How to calculate the number of nodes... Hmm... It's not easy. However, the problem will become easy if the tree is a perfect binary tree. For a perfect binary tree with depth `n` (root at depth `0`), there are totally $$2^{n+1} - 1$$ nodes. By checking whether the most left and the most right elements are at the same level (assuming we already know this is a complete tree), we can know if the complete tree is a perfect one.

#### Solution
```cpp
#include <iostream>

#include <cmath>

using std::cout;
using std::endl;

struct TreeNode
{
	int val;
	TreeNode *left;
	TreeNode *right;
	TreeNode(int x) : val(x), left(NULL), right(NULL) {}
};

class Solution {
public:
	int countNodes(TreeNode* root) {
		if (!root)
			return 0;

		int l_d = 0, r_d = 0;

		TreeNode* l_node = root;
		TreeNode* r_node = root;

		while (l_node->left)
		{
			++l_d;
			l_node = l_node->left;
		}

		while (r_node->right)
		{
			++r_d;
			r_node = r_node->right;
		}

		if (l_d == r_d)
			return pow(2, l_d + 1) - 1;

		return 1 + countNodes(root->left) + countNodes(root->right);
	}
};

int main()
{
	TreeNode node1(1);
	TreeNode node2(2);
	TreeNode node3(3);
	TreeNode node4(4);
	TreeNode node5(5);
	TreeNode node6(6);

	TreeNode* root = &node1;
	root->left = &node2;
	root->right = &node3;
	root->left->left = &node4;
	root->left->right = &node5;
	root->right->left = &node6;

	Solution solution;
	cout << solution.countNodes(root);
}

```
