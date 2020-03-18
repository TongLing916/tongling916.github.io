---
layout:     post
title:      "Divide and Conquer"
date:       2019-5-2
author:     Tong
catalog: true
tags:
    - Algorithm
---

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

Every time we meet an operator, we split the string into two parts and compute the results from the two parts. Then, we use the operator to calculate the combination of the two results.

Use `std::stoi(input)` to convert `(string) input` into an integer number.

`std::stol()` to long.
`std::stoll()` to long long.
`std::stof()` and `std::stod()` to float and double.

#### Solution
```cpp
#include <iostream>

#include <vector>

#include <string>

using std::cout;
using std::endl;
using std::vector;
using std::string;

class Solution {
public:
	vector<int> diffWaysToCompute(string input)
	{
		vector<int> res;
		for (size_t i = 0; i < input.size(); ++i)
		{
			if (input[i] < '0' || input[i] > '9')
			{
				vector<int> res1 = diffWaysToCompute(input.substr(0, i));
				vector<int> res2 = diffWaysToCompute(input.substr(i + 1));

				for (const auto& r1 : res1)
				{
					for (const auto& r2 : res2)
					{
						switch (input[i])
						{
						case '+':
							res.emplace_back(r1 + r2);
							break;
						case '-':
							res.emplace_back(r1 - r2);
							break;
						case '*':
							res.emplace_back(r1 * r2);
						}
					}
				}
			}
		}

		if (res.empty())
			res.emplace_back(std::stoi(input));

		return res;
	}
};

int main()
{
	string s1 = "2-1-1";
	string s2 = "2*3-4*5";

	Solution solution;
	vector<int> res1 = solution.diffWaysToCompute(s1);
	vector<int> res2 = solution.diffWaysToCompute(s2);
	for (auto i : res1)
		cout << i << " ";
	cout << endl;
}
```

### [399. Nuts 和 Bolts 的问题](https://www.lintcode.com/problem/nuts-bolts-problem/)

给定一组 `n` 个不同大小的 nuts 和 `n` 个不同大小的 bolts. nuts 和 `bolts 一一匹配.

不允许将 nut 之间互相比较, 也不允许将 bolt 之间互相比较. 也就是说, 只许将 nut 与 bolt 进行比较, 或将 bolt 与 nut 进行比较. 我们会提供一个比较函数, 用于nut和bolt的比较.

利用我们提供的函数, 你需要将 nuts 或者 bolts 重新排列, 使得它们按照顺序一一匹配.
样例

给出 ```nuts = ['ab','bc','dd','gg'], bolts = ['AB','GG', 'DD', 'BC']```

你的程序应该找出bolts和nuts的匹配.

根据比较函数, 一组可能的返回结果是：
```
nuts = ['ab','bc','dd','gg'], bolts = ['AB','BC','DD','GG']
```
如果我们给你另外的比较函数，可能返回的结果是：
```
nuts = ['ab','bc','dd','gg'], bolts = ['BC','AB','DD','GG']
```
因此的结果完全取决于比较函数，而不是字符串本身。因为你必须使用比较函数来进行排序。

各自的排序当中nuts和bolts的顺序是无关紧要的，只要他们一一匹配就可以。


#### Solution

```c++
class Solution {
 public:
  /**
   * @param nuts: a vector of integers
   * @param bolts: a vector of integers
   * @param compare: a instance of Comparator
   * @return: nothing
   */
  void sortNutsAndBolts(vector<string>& nuts, vector<string>& bolts,
                        Comparator& compare) {
    std::random_shuffle(nuts.begin(), nuts.end());
    std::random_shuffle(bolts.begin(), bolts.end());
    quickSort(nuts, bolts, 0, nuts.size() - 1, compare);
  }

 private:
  void quickSort(vector<string>& nuts, vector<string>& bolts, const int l,
                 const int r, Comparator& compare) {
    if (l >= r) {
      return;
    }

    string pivot = nuts[l];  // 选取nuts[l]作为快速排序的pivot
    int pos = 0, cnt = 0;

    // 一次循环, 处理出pivot的排名
    for (int i = l; i <= r; ++i) {
      const int t = compare.cmp(pivot, bolts[i]);
      if (t == 1) {
        ++cnt;
      } else if (t == 0) {
        pos = i;
      }
    }
    cnt += l;
    std::swap(bolts[pos], bolts[cnt]);
    pos = cnt;

    // pivot处于pos位置

    // 对bolts进行划分, 比pivot小的归到pos左边, 大的归到右边
    for (int i = l, j = r; i < pos && pos < j;) {
      while (i < pos && compare.cmp(pivot, bolts[i]) == 1) {
        ++i;
      }
      while (pos < j && compare.cmp(pivot, bolts[j]) == -1) {
        --j;
      }
      if (i < pos && pos < j) {
        std::swap(bolts[i++], bolts[j--]);
      }
    }

    // 对nuts进行划分, 同上的过程
    pivot = bolts[pos];
    std::swap(nuts[pos], nuts[l]);
    for (int i = l, j = r; i < pos && pos < j;) {
      while (i < pos && compare.cmp(nuts[i], pivot) == -1) {
        ++i;
      }
      while (pos < j && compare.cmp(nuts[j], pivot) == 1) {
        --j;
      }
      if (i < pos && pos < j) {
        std::swap(nuts[i++], nuts[j--]);
      }
    }

    // 递归处理左右
    quickSort(nuts, bolts, l, pos - 1, compare);
    quickSort(nuts, bolts, pos + 1, r, compare);
  }
};
```
