---
layout:     post
title:      "Divide and Conquer"
date:       2019-5-2
author:     Tong
catalog: true
tags:
    - Divide and Conquer
---

### Summary

1.


### [4. Median of Two Sorted Arrays](https://leetcode.com/problems/median-of-two-sorted-arrays/)

#### Question

There are two sorted arrays __nums1__ and __nums2__ of size m and n respectively.

Find the median of the two sorted arrays. The overall run time complexity should be `O(log (m+n))`.

You may assume __nums1__ and __nums2__ cannot be both empty.

__Example 1:__
```
nums1 = [1, 3]
nums2 = [2]

The median is 2.0
```

__Example 2:__
```
nums1 = [1, 2]
nums2 = [3, 4]

The median is (2 + 3)/2 = 2.5
```

#### Train of Thought

#### Solution
```cpp

```

### [215. Kth Largest Element in an Array](https://leetcode.com/problems/kth-largest-element-in-an-array/)

#### Question

Find the <b>k</b>th largest element in an unsorted array. Note that it is the kth largest element in the sorted order, not the kth distinct element.

__Example 1:__
```
Input: [3,2,1,5,6,4] and k = 2
Output: 5
```

__Example 2:__
```
Input: [3,2,3,1,2,4,5,5,6] and k = 4
Output: 4
```

__Note:__

You may assume k is always valid, 1 ≤ k ≤ array's length.

#### Train of Thought

We can use the quickSort method to find the expected element. It is necessary to be sure what the index of the <b>k</b>th largest element is. It should be `nums.size() - k`.

#### Solution
```cpp
#include <iostream>

#include <vector>

#include <algorithm>

using namespace std;

class Solution {
public:
	int findKthLargest(vector<int>& nums, int k) {
		if (k > nums.size())
			throw("k is too big");
		random_shuffle(nums.begin(), nums.end());
		return quickSelect(nums, 0, nums.size() - 1, nums.size() - k);
	}

	int quickSelect(vector<int>& nums, int lo, int hi, int k)
	{
		int i = lo, j = hi + 1;
		int v = nums[lo];
		if (hi <= lo)
			return nums[lo];
		while (1)
		{
			while (nums[++i] < v)
				if (i == hi)
					break;
			while (nums[--j] > v)
				if (j == lo)
					break;
			if (i >= j)
				break;
			swap(nums[i], nums[j]);
		}
		swap(nums[lo], nums[j]);
		if (j == k)    
			return nums[j];
		else if (j < k)
			return quickSelect(nums, j + 1, hi, k);
		else
			return quickSelect(nums, lo, j - 1, k);
	}
};

int main()
{
	vector<int> test1{ 3,2,3,1,2,4,5,5,6 };
	vector<int> test2{ 3,2,1,5,6,4 };
	int k1 = 4;
	int k2 = 2;
	Solution solution;
	cout << solution.findKthLargest(test2, k2) << endl;
}
```

### [241. Different Ways to Add Parentheses](https://leetcode.com/problems/different-ways-to-add-parentheses/)

#### Question

Given a string of numbers and operators, return all possible results from computing all the different possible ways to group numbers and operators. The valid operators are `+`, `-` and `*`.
__Example 1:__
```
Input: "2-1-1"
Output: [0, 2]
Explanation:
((2-1)-1) = 0
(2-(1-1)) = 2
```

__Example 2:__
```
Input: "2*3-4*5"
Output: [-34, -14, -10, -10, 10]
Explanation:
(2*(3-(4*5))) = -34
((2*3)-(4*5)) = -14
((2*(3-4))*5) = -10
(2*((3-4)*5)) = -10
(((2*3)-4)*5) = 10
```

#### Train of Thought



#### Solution
```cpp

```
