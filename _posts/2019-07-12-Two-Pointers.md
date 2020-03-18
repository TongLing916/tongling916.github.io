---
layout:     post
title:      "Two Pointers"
date:       2019-7-12
author:     Tong
catalog: true
tags:
    - Algorithm
---

### [287. Find the Duplicate Number](https://leetcode.com/problems/find-the-duplicate-number/)

#### Question

Given an array nums containing $$n + 1$$ integers where each integer is between $$1$$ and $$n$$ (inclusive), prove that at least one duplicate number must exist. Assume that there is only one duplicate number, find the duplicate one.

__Example 1:__
```
Input: [1,3,4,2,2]
Output: 2
```

__Example 2:__
```
Input: [3,1,3,4,2]
Output: 3
```

__Note:__
1. You must not modify the array (assume the array is read only).
2. You must use only constant, O(1) extra space.
3. Your runtime complexity should be less than O(n2).
4. There is only one duplicate number in the array, but it could be repeated more than once.

#### Train of Thought

The very first thought is probably to sort the array firstly, then check if there are two elements with the same value.

The second solution could be using `unordered_set`. For each element, we check if we have already inserted it into the set. If yes, return this duplicate one. If not, we insert this element into the set.

The third solution is based on the solution of the question [LeetCode 142. Linked List Cycle II](https://leetcode.com/problems/linked-list-cycle-ii/). If we consider this array as a linked list, the duplicate numbers will cause a loop. Use two pointers can help us to find the start of the loop.

#### Solution
```cpp
#include <algorithm>

#include <iostream>

#include <unordered_set>

#include <vector>

using std::cout;
using std::endl;
using std::unordered_set;
using std::vector;

class Solution1 {
public:
	int findDuplicate(vector<int>& nums) {
		std::sort(nums.begin(), nums.end());
		for (int i = 1; i < nums.size(); ++i)
			if (nums[i] == nums[i - 1])
				return nums[i];
		return -1;
	}
};

class Solution2 {
public:
	int findDuplicate(vector<int>& nums) {
		unordered_set<int> nums_set;
		for (int i = 0; i < nums.size(); ++i)
		{
			if (nums_set.find(nums[i]) != nums_set.end())
				return nums[i];
			nums_set.insert(nums[i]);
		}
		return -1;
	}
};

// regard the array as a linked list
// check LeetCode 142. Linked List Cycle II
class Solution3 {
public:
	int findDuplicate(const vector<int>& nums) {
		if (nums.empty()) {
			return -1;
		}

		int slow = nums[0];
		int fast = nums[0];

		do {
			slow = nums[slow];
			fast = nums[nums[fast]];
		} while (slow != fast);

		slow = nums[0];
		while (slow != fast) {
			slow = nums[slow];
			fast = nums[fast];
		}
		return fast;
	}
};


int main()
{
	Solution1 solution;

	vector<int> test1{ 1,3,4,2,2 };
	vector<int> test2{ 3,1,3,4,2 };

	cout << solution.findDuplicate(test1) << endl;
}

```
