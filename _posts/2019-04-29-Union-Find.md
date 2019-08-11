---
layout:     post
title:      "Union Find"
date:       2019-4-29
author:     Tong
catalog: true
tags:
    - Algorithm
---

### Summary

1. From where to start? (given coordinates? one by one? border?)

2. Do we need to remember the visited elements? If yes, how? (set to another value?)

3. Simple implementation of weighted union find

```cpp
#include <iostream>

#include <vector>

#include <algorithm>

#include <string>

#include <unordered_set>

#include <unordered_map>

using namespace std;

// WeightedQuickUnionUF: standard implementation
// RandomWeightedQuickUnionUF: Input: N - number of pairs, Pairs, Output: number of grounps

class WeightedQuickUnionUF
{
private:
	vector<int> parent;   // parent[i] = parent of i
	vector<int> size;     // size[i] = number of sites in subtree rooted at i;
	int count;
	void validate(int p)
	{
		int n = parent.size();
		if (p < 0 || p >= n)
			throw to_string(p) + " is way too big!";
	}

public:
	WeightedQuickUnionUF(int n) : count(n)
	{
		parent.reserve(n);
		size.reserve(n);
		for (int i = 0; i < n; ++i)
		{
			parent[i] = i;
			size[i] = 1;
		}
	}

	int getCount()
	{
		return count;
	}

	int find(int p)
	{
		validate(p);
		while (p != parent[p])
			p = parent[p];
		return p;
	}

	bool connected(int p, int q)
	{
		return find(p) == find(q);
	}

	void unionTwo(int p, int q)
	{
		int rootP = find(p);
		int rootQ = find(q);
		if (rootP == rootQ) return;

		// A potential imporvement is path compression
		// That is, directly connect every node to its new root.

		if (size[rootP] < size[rootQ])            
		{
			parent[rootP] = rootQ;
			size[rootQ] += size[rootP];
		}
		else
		{
			parent[rootQ] = rootP;
			size[rootP] += size[rootQ];
		}
		--count;
	}
};

class RandomWeightedQuickUnionUF
{
private:
	unordered_map<int, int> parent;   // parent[i] = parent of i

	unordered_map<int, int> size;     // size[i] = number of sites in subtree rooted at i;

	int count;
	void validate(int p)
	{
		int n = parent.size();
		if (p < 0 || p >= n)
			throw to_string(p) + " is way too big!";
	}

public:
	RandomWeightedQuickUnionUF(unordered_set<int>& nums)
	{
		count = nums.size();
		for (auto& num : nums)
		{
			parent.insert({ num, num });
			size.insert({ num, 1 });
		}
	}

	int getCount()
	{
		return count;
	}

	int find(int p)
	{
		validate(p);
		while (p != parent[p])
			p = parent[p];
		return p;
	}

	bool connected(int p, int q)
	{
		return find(p) == find(q);
	}

	void unionTwo(int p, int q)
	{
		int rootP = find(p);
		int rootQ = find(q);
		if (rootP == rootQ) return;

		// A potential imporvement is path compression

		// That is, directly connect every node to its new root.

		if (size[rootP] < size[rootQ])
		{
			parent[rootP] = rootQ;
			size[rootQ] += size[rootP];
		}
		else
		{
			parent[rootQ] = rootP;
			size[rootP] += size[rootQ];
		}
		--count;
	}
};


int main()
{
	unordered_set<int> uSet;
	int N;        // number of input pairs

	cin >> N;
	vector<pair<int, int>> pairs;
	int a, b;
	for (int i = 0; i < N; ++i)
	{
		cin >> a >> b;
		pairs.push_back({ a, b });
		uSet.insert(a);
		uSet.insert(b);
	}
	RandomWeightedQuickUnionUF uf(uSet);
	for (int i = 0; i < N; ++i)
		uf.unionTwo(pairs[i].first, pairs[i].second);
	cout << uf.getCount() << endl;
}
```


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


### [399\. Evaluate Division](https://leetcode.com/problems/evaluate-division/)

Difficulty: **Medium**


Equations are given in the format `A / B = k`, where `A` and `B` are variables represented as strings, and `k` is a real number (floating point number). Given some queries, return the answers. If the answer does not exist, return `-1.0`.

**Example:**  
Given `a / b = 2.0, b / c = 3.0.`  
queries are: `a / c = ?, b / a = ?, a / e = ?, a / a = ?, x / x = ? .`  
return `[6.0, 0.5, -1.0, 1.0, -1.0 ].`

The input is: `vector<pair<string, string>> equations, vector<double>& values, vector<pair<string, string>> queries` , where `equations.size() == values.size()`, and the values are positive. This represents the equations. Return `vector<double>`.

According to the example above:

```
equations = [ ["a", "b"], ["b", "c"] ],
values = [2.0, 3.0],
queries = [ ["a", "c"], ["b", "a"], ["a", "e"], ["a", "a"], ["x", "x"] ].
```

The input is always valid. You may assume that evaluating the queries will result in no division by zero and there is no contradiction.


#### Solution

Language: **C++**

```c++
class Solution {
public:
    vector<double> calcEquation(vector<vector<string>>& equations, vector<double>& values, vector<vector<string>>& queries) {
        unordered_map<string, int> dict_id;
        unordered_map<string, double> dict_val;
        unordered_map<int, int> id_cnt;
        int id = 0;
        for (int i = 0; i < equations.size(); ++i)
        {
            vector<string> eq = equations[i];
            if (!dict_id.count(eq[0]) && !dict_id.count(eq[1]))
            {
                dict_id[eq[0]] = id;
                dict_id[eq[1]] = id;
                id_cnt[id] += 2;
                ++id;
                dict_val[eq[0]] = values[i];
                dict_val[eq[1]] = 1.;
            }
            else if (dict_id.count(eq[0]) && !dict_id.count(eq[1]))
            {
                dict_id[eq[1]] = dict_id[eq[0]];
                ++id_cnt[dict_id[eq[0]]];
                dict_val[eq[1]] = dict_val[eq[0]] / values[i];
            }
            else if (!dict_id.count(eq[0]) && dict_id.count(eq[1]))
            {
                dict_id[eq[0]] = dict_id[eq[1]];
                ++id_cnt[dict_id[eq[1]]];
                dict_val[eq[0]] = dict_val[eq[1]] * values[i];
            }
            else
            {
                int id_0 = dict_id[eq[0]];
                int id_1 = dict_id[eq[1]];
                double val_0 = dict_val[eq[0]];
                double val_1 = dict_val[eq[1]];

                if (id_cnt[id_0] < id_cnt[id_1])
                {
                    double coef = values[i] * val_1 / val_0;
                    for (auto& d : dict_id)
                        if (d.second == id_0)
                        {
                            d.second = id_1;
                            dict_val[d.first] *= coef;
                        }
                }
                else
                {
                    double coef = val_0 / values[i] / val_1;
                    for (auto& d : dict_id)
                        if (d.second == id_1)
                        {
                            d.second = id_0;
                            dict_val[d.first] *= coef;
                        }
                }
            }
```

### [947\. Most Stones Removed with Same Row or Column](https://leetcode.com/problems/most-stones-removed-with-same-row-or-column/)

Difficulty: **Medium**


On a 2D plane, we place stones at some integer coordinate points.  Each coordinate point may have at most one stone.

Now, a _move_ consists of removing a stone that shares a column or row with another stone on the grid.

What is the largest possible number of moves we can make?


**Example 1:**

```
Input: stones = [[0,0],[0,1],[1,0],[1,2],[2,1],[2,2]]
Output: 5
```


**Example 2:**

```
Input: stones = [[0,0],[0,2],[1,1],[2,0],[2,2]]
Output: 3
```


**Example 3:**

```
Input: stones = [[0,0]]
Output: 0
```

**<span style="display: inline;">Note:</span>**

1.  `1 <= stones.length <= 1000`
2.  `0 <= stones[i][j] < 10000`


#### Solution

Language: **C++**

```c++
class DSU
{
private:
    vector<int> parents;
public:
    DSU(int n)
    {
        parents.resize(n);
        for (int i = 0; i < n; ++i)
            parents[i] = i;
    }
    int find(int i)
    {
        if (parents[i] != i) parents[i] = find(parents[i]);
        return parents[i];
    }
    void uni(int i, int j)
    {
        parents[find(parents[i])] = find(parents[j]);
    }
};
​
class Solution
{
public:
    int removeStones(vector<vector<int>>& stones) {
        int n = stones.size();
        DSU dsu(20000);
        for (auto& s : stones)
            dsu.uni(s[0], s[1] + 10000);
        unordered_set<int> seen;
        for (auto& s : stones)
            seen.insert(dsu.find(s[0]));
        return n - seen.size();
    }
};
```
