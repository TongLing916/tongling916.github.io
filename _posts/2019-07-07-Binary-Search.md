---
layout:     post
title:      "Binary Search"
date:       2019-7-7
author:     Tong
catalog: true
tags:
    - Search
---

### Summary

1. Cast the Integer number to a `long int`. Then, use `<<` to speed up the computation. (See `29. Divide Two Integers` and `50. Pow(x, n)`)


### [29. Divide Two Integers](https://leetcode.com/problems/divide-two-integers/)

#### Question

Given two integers dividend and divisor, divide two integers without using multiplication, division and mod operator.

Return the quotient after dividing dividend by divisor.

The integer division should truncate toward zero.

__Example 1:__
Input: dividend = 10, divisor = 3
Output: 3

__Example 2:__
Input: dividend = 7, divisor = -3
Output: -2

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
Input: nums = [4,5,6,7,0,1,2], target = 0
Output: 4

__Example 2:__
Input: nums = [4,5,6,7,0,1,2], target = 3
Output: -1

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


### [34. Find First and Last Position of Element in Sorted Array](https://leetcode.com/problems/find-first-and-last-position-of-element-in-sorted-array/)

#### Question

Given an array of integers `nums` sorted in ascending order, find the starting and ending position of a given `target` value.

Your algorithm's runtime complexity must be in the order of O(log n).

If the target is not found in the array, return `[-1, -1]`.

__Example 1:__
Input: nums = [5,7,7,8,8,10], target = 8
Output: [3,4]

__Example 2:__
Input: nums = [5,7,7,8,8,10], target = 6
Output: [-1,-1]

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
Input: 2.00000, 10
Output: 1024.00000

__Example 2:__
Input: 2.10000, 3
Output: 9.26100

__Example 3:__
Input: 2.00000, -2
Output: 0.25000
Explanation: $$2^{-2} = 1/2^2 = 1/4 = 0.25$$

__Note:__
- -100.0 < x < 100.0
- n is a 32-bit signed integer, within the range [−2^{31}, 2^{31} − 1]

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
				tmp_res *= tmp_res;
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
