---
layout:     post
title:      "Segment Tree"
date:       2019-9-2
author:     Tong
catalog: true
tags:
    - Algorithm
---

### [307\. Range Sum Query - Mutable](https://leetcode.com/problems/range-sum-query-mutable/)

Difficulty: **Medium**


Given an integer array _nums_, find the sum of the elements between indices _i_ and _j_ (_i_ ≤ _j_), inclusive.

The _update(i, val)_ function modifies _nums_ by updating the element at index _i_ to _val_.

**Example:**

```
Given nums = [1, 3, 5]

sumRange(0, 2) -> 9
update(1, 2)
sumRange(0, 2) -> 8
```

**Note:**

1.  The array is only modifiable by the _update_ function.
2.  You may assume the number of calls to _update_ and _sumRange_ function is distributed evenly.


#### Solution

Language: **C++**

```c++
class NumArray {
public:
    NumArray(vector<int>& nums) {
        
    }
    
    void update(int i, int val) {
        
    }
    
    int sumRange(int i, int j) {
        
    }
};
​
/**
 * Your NumArray object will be instantiated and called as such:
 * NumArray* obj = new NumArray(nums);
 * obj->update(i,val);
 * int param_2 = obj->sumRange(i,j);
 */
```



### [673\. Number of Longest Increasing Subsequence](https://leetcode.com/problems/number-of-longest-increasing-subsequence/)

Difficulty: **Medium**


Given an unsorted array of integers, find the number of longest increasing subsequence.

**Example 1:**  

```
Input: [1,3,5,4,7]
Output: 2
Explanation: The two longest increasing subsequence are [1, 3, 4, 7] and [1, 3, 5, 7].
```

**Example 2:**  

```
Input: [2,2,2,2,2]
Output: 5
Explanation: The length of longest continuous increasing subsequence is 1, and there are 5 subsequences' length is 1, so output 5.
```

**Note:** Length of the given array will be not exceed 2000 and the answer is guaranteed to be fit in 32-bit signed int.


#### Solution

Language: **C++**

```c++
class Solution {
public:
    int findNumberOfLIS(vector<int>& nums) {
        
    }
};
```
