---
layout:     post
title:      "Union Find"
date:       2019-4-29
author:     Tong
catalog: true
tags:
    - Search
---

### Summary

1. From where to start? (given coordinates? one by one? border?)

2. Do we need to remember the visited elements? If yes, how? (set to another value?)


### [128. Longest Consecutive Sequence](https://leetcode.com/problems/longest-consecutive-sequence/)

#### Question

Given an unsorted array of integers, find the length of the longest consecutive elements sequence.

Your algorithm should run in `O(n)` complexity.

__Example:__
```
Input: [100, 4, 200, 1, 3, 2]
Output: 4
Explanation: The longest consecutive elements sequence is [1, 2, 3, 4]. Therefore its length is 4.
```

#### Train of Thought

To count the consecutive elements, it is best to find the first one of that sequence (or we can also [extend](https://leetcode.com/problems/longest-consecutive-sequence/discuss/41088/Possibly-shortest-cpp-solution-only-6-lines.) it?) Then the question is, how to find the first one? and then how to find the following ones?

The first idea is usually sorting the array at first, then try to traverse all numbers (Read `longestConsecutive_sort`). However, the time complexity will depend on the time of sorting, which is usually not `O(n)`. Therefore, we need to find another way.

To find a visited element in short time, we can consider using Hash. Then, to count a consecutive sequence, we need to find its start. E.g., for number `i`, By checking if `i-1` in our hash set, we can know if this is a start. If it is, count all the following elements. Using this method, we only access the whole vector twice, once for initializing hash set, the other for traversal.

#### Solution - Sort and [Hash](https://www.geeksforgeeks.org/longest-consecutive-subsequence/)
```cpp
#include <iostream>

#include <vector>

#include <algorithm>

#include <unordered_set>

using namespace std;

static const auto _ = []() {
	ios::sync_with_stdio(false);
	cin.tie(nullptr);
	cout.tie(nullptr);
	return nullptr;
}();

class Solution {
public:
	int longestConsecutive_sort(vector<int>& nums) {
		int n = nums.size();
		if (n <= 1)
			return n;

		sort(nums.begin(), nums.end());
		int maxLen = 1;
		int curLen = 1;
		int first = 0, last = 1;
		for (; last < nums.size(); ++first, ++last)
		{
			if (nums[last] == nums[first])
				continue;
			else if (nums[last] == nums[first] + 1)
			{
				curLen += 1;
				maxLen = max(maxLen, curLen);
			}
			else
			{
				maxLen = max(maxLen, curLen);
				curLen = 1;
			}
		}

		return maxLen;
	}

	int longestConsecutive(vector<int>& nums) {
		unordered_set<int> sNums(nums.begin(), nums.end());
		int maxLen = 0;
		for (int num : sNums)
			if (sNums.find(num - 1) == sNums.end())
			{
				int curLen = 1;
				while (sNums.find(++num) != sNums.end())
					++curLen;
				maxLen = max(maxLen, curLen);
			}

		return maxLen;
	}
};

int main()
{
	int n;
	cin >> n;
	vector<int> nums(n);
	int tmp;
	for (int i = 0; i < n; ++i)
	{
		cin >> tmp;
		nums[i] = tmp;
	}
	Solution solution;
	cout << endl << endl;
	cout << "Sort: " << solution.longestConsecutive_sort(nums) << endl;
	cout << "Hash: " << solution.longestConsecutive(nums) << endl;
}
```
