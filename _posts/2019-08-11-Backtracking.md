---
layout:     post
title:      "Backtracking"
date:       2019-8-11
author:     Tong
catalog: true
tags:
    - Algorithm
---

### Summary

1.

### [17\. Letter Combinations of a Phone Number](https://leetcode.com/problems/letter-combinations-of-a-phone-number/)

Difficulty: **Medium**


Given a string containing digits from `2-9` inclusive, return all possible letter combinations that the number could represent.

A mapping of digit to letters (just like on the telephone buttons) is given below. Note that 1 does not map to any letters.

![](http://upload.wikimedia.org/wikipedia/commons/thumb/7/73/Telephone-keypad2.svg/200px-Telephone-keypad2.svg.png)

**Example:**

```
Input: "23"
Output: ["ad", "ae", "af", "bd", "be", "bf", "cd", "ce", "cf"].
```

**Note:**

Although the above answer is in lexicographical order, your answer could be in any order you want.

#### Solution

Language: **C++**

```c++

#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>

using namespace std;

class Solution {
private:
	unordered_map<char, vector<char>> dict_;
public:
	void combination(vector<string>& res, string& digits, int start, string cur)
	{
		int n = digits.size();
		if (start == n) res.push_back(cur);
		int char_size = dict_[digits[start]].size();
		for (int i = 0; i < char_size; ++i)
		{
			string new_cur = cur + dict_[digits[start]][i];
			combination(res, digits, start + 1, new_cur);
		}
	}
	vector<string> letterCombinations(string digits) {
		dict_['2'] = { 'a', 'b', 'c' };
		dict_['3'] = { 'd', 'e', 'f' };
		dict_['4'] = { 'g', 'h', 'i' };
		dict_['5'] = { 'j', 'k', 'l' };
		dict_['6'] = { 'm', 'n', 'o' };
		dict_['7'] = { 'p', 'q', 'r', 's' };
		dict_['8'] = { 't', 'u', 'v' };
		dict_['9'] = { 'w', 'x', 'y', 'z' };
		vector<string> res;
		if (digits.size() == 0) return res;
		combination(res, digits, 0, "");
		return res;
	}
};

int main()
{
	Solution solution;
	vector<string> res = solution.letterCombinations("23");
	for (auto& r : res)
		cout << r << " ";
}

```

### [22\. Generate Parentheses](https://leetcode.com/problems/generate-parentheses/)

Difficulty: **Medium**


Given _n_ pairs of parentheses, write a function to generate all combinations of well-formed parentheses.

For example, given _n_ = 3, a solution set is:

```
[
  "((()))",
  "(()())",
  "(())()",
  "()(())",
  "()()()"
]
```


#### Solution

Language: **C++**

```c++
#include <iostream>
#include <vector>
#include <string>

using namespace std;

class Solution {
public:
	void backtracking(vector<string>& res, int& n, int left, int right, string cur)
	{
		if (right == n)
		{
			res.push_back(cur);
			return;
		}

		if (left < n) backtracking(res, n, left + 1, right, cur + "(");

		if (left > right) backtracking(res, n, left, right + 1, cur + ")");
	}

	vector<string> generateParenthesis(int n) {
		vector<string> res;
		if (n == 0) return res;
		int left = 0, right = 0;
		backtracking(res, n, left, right, "");
		return res;
	}
};

int main()
{
	Solution solution;
	vector<string> res = solution.generateParenthesis(5);
	for (auto& r : res)
		cout << r << endl;
}

```


### [39\. Combination Sum](https://leetcode.com/problems/combination-sum/)

Difficulty: **Medium**


Given a **set** of candidate numbers (`candidates`) **(without duplicates)** and a target number (`target`), find all unique combinations in `candidates` where the candidate numbers sums to `target`.

The **same** repeated number may be chosen from `candidates` unlimited number of times.

**Note:**

*   All numbers (including `target`) will be positive integers.
*   The solution set must not contain duplicate combinations.

**Example 1:**

```
Input: candidates = [2,3,6,7], target = 7,
A solution set is:
[
  [7],
  [2,2,3]
]
```

**Example 2:**

```
Input: candidates = [2,3,5], target = 8,
A solution set is:
[
  [2,2,2,2],
  [2,3,3],
  [3,5]
]
```


#### Solution

Language: **C++**

```c++
#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

class Solution {
public:
	void backtracking(vector<vector<int>>& res, vector<int>& candidates, int target, vector<int>& cur, int start)
	{
		if (target == 0)
		{
			res.push_back(cur);
			return;
		}
		for (int i = start; i < candidates.size(); ++i)
		{
			if (target - candidates[i] < 0) break;
			cur.push_back(candidates[i]);
			backtracking(res, candidates, target - candidates[i], cur, i);
			cur.pop_back();
		}
	}
	vector<vector<int>> combinationSum(vector<int>& candidates, int target) {
		vector<vector<int>> res;
		sort(candidates.begin(), candidates.end());
		if (candidates.size() == 0) return res;
		vector<int> cur;
		backtracking(res, candidates, target, cur, 0);
		return res;
	}
};

int main()
{
	vector<int> candidates{ 2, 3, 6, 7 };
	Solution solution;
	vector<vector<int>> res = solution.combinationSum(candidates, 8);
	for (auto& r : res)
	{
		for (auto& e : r)
			cout << e << " ";
		cout << endl;
	}
}
```

### [40\. Combination Sum II](https://leetcode.com/problems/combination-sum-ii/)

Difficulty: **Medium**


Given a collection of candidate numbers (`candidates`) and a target number (`target`), find all unique combinations in `candidates` where the candidate numbers sums to `target`.

Each number in `candidates` may only be used **once** in the combination.

**Note:**

*   All numbers (including `target`) will be positive integers.
*   The solution set must not contain duplicate combinations.

**Example 1:**

```
Input: candidates = [10,1,2,7,6,1,5], target = 8,
A solution set is:
[
  [1, 7],
  [1, 2, 5],
  [2, 6],
  [1, 1, 6]
]
```

**Example 2:**

```
Input: candidates = [2,5,2,1,2], target = 5,
A solution set is:
[
  [1,2,2],
  [5]
]
```


#### Solution

Language: **C++**

```c++

#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

class Solution {
public:
	void backtracking(vector<vector<int>>& res, vector<int>& candidates, int target, vector<int>& cur, int start)
	{
		if (target == 0)
		{
			res.push_back(cur);
			return;
		}

		for (int i = start; i < candidates.size(); ++i)
		{
			if (i > start && candidates[i] == candidates[i - 1]) continue;
			if (candidates[i] > target) break;
			cur.push_back(candidates[i]);
			backtracking(res, candidates, target - candidates[i], cur, i + 1);
			cur.pop_back();
		}
	}

	vector<vector<int>> combinationSum2(vector<int>& candidates, int target) {
		vector<vector<int>> res;
		int n = candidates.size();
		if (n == 0) return res;
		vector<int> cur;
		sort(candidates.begin(), candidates.end());
		backtracking(res, candidates, target, cur, 0);
		return res;
	}
};

int main()
{
	Solution solution;
	vector<int> candidates{ 10, 1, 2, 7, 6, 1, 5 };
	vector<vector<int>> res = solution.combinationSum2(candidates, 8);
	for (auto& r : res)
	{
		for (auto& e : r)
			cout << e << " ";
		cout << endl;
	}
}

```


### [46\. Permutations](https://leetcode.com/problems/permutations/)

Difficulty: **Medium**


Given a collection of **distinct** integers, return all possible permutations.

**Example:**

```
Input: [1,2,3]
Output:
[
  [1,2,3],
  [1,3,2],
  [2,1,3],
  [2,3,1],
  [3,1,2],
  [3,2,1]
]
```


#### Solution

Language: **C++**

```c++
#include <iostream>
#include <vector>

using namespace std;

class Solution {
private:
	void backtracking(vector<vector<int>>& res, vector<int>& nums, vector<int>& cur, int pos)
	{
		if (pos == nums.size())
		{
			res.push_back(cur);
			return;
		}

		for (int i = pos; i < nums.size(); ++i)
		{
			std::swap(nums[pos], nums[i]);
			cur.push_back(nums[pos]);
			backtracking(res, nums, cur, pos + 1);
			cur.pop_back();
			std::swap(nums[pos], nums[i]);
		}
	}
public:
	vector<vector<int>> permute(vector<int>& nums) {
		vector<vector<int>> res;
		if (nums.size() == 0) return res;
		vector<int> cur;
		backtracking(res, nums, cur, 0);
		return res;
	}
};

int main()
{
	vector<int> nums{ 1,2,3 };
	Solution solution;
	vector<vector<int>> res = solution.permute(nums);
	for (auto& r : res)
	{
		for (auto& e : r)
			cout << e << " ";
		cout << endl;
	}
}

```

### [47\. Permutations II](https://leetcode.com/problems/permutations-ii/)

Difficulty: **Medium**


Given a collection of numbers that might contain duplicates, return all possible unique permutations.

**Example:**

```
Input: [1,1,2]
Output:
[
  [1,1,2],
  [1,2,1],
  [2,1,1]
]
```


#### Solution

Language: **C++**

```c++
#include <algorithm>
#include <iostream>
#include <vector>
#include <unordered_set>

using namespace std;

class Solution {
public:
	void backtracking(vector<vector<int>>& res, vector<int> nums, int n, int i)
	{
		if (i == n - 1)
		{
			res.push_back(nums);
			return;
		}

		for (int k = i; k < n; ++k)
		{
			if (k != i && nums[k] == nums[i]) continue;
			swap(nums[k], nums[i]);
			backtracking(res, nums, n, i + 1);
		}
	}

	vector<vector<int>> permuteUnique(vector<int>& nums) {
		vector<vector<int>> res;
		std::sort(nums.begin(), nums.end());
		backtracking(res, nums, nums.size(), 0);
		return res;
	}
};

class Solution2
{
private:
	void backtracking(vector<vector<int>>& res, vector<int>& nums, vector<int>& cur, int pos)
	{
		if (pos == nums.size())
		{
			res.push_back(cur);
			return;
		}

		unordered_set<int> used;
		for (int i = pos; i < nums.size(); ++i)
		{
			if (used.count(nums[i])) continue;
			used.insert(nums[i]);
			std::swap(nums[i], nums[pos]);
			cur.push_back(nums[pos]);
			backtracking(res, nums, cur, pos + 1);
			cur.pop_back();
			std::swap(nums[i], nums[pos]);
		}
	}
public:
	vector<vector<int>> permuteUnique(vector<int>& nums)
	{
		vector<vector<int>> res;
		if (nums.size() == 0) return res;
		std::sort(nums.begin(), nums.end());
		vector<int> cur;
		backtracking(res, nums, cur, 0);
		return res;
	}
};


int main()
{
	vector<int> nums{ 1, 1, 2, 3, 3 };
	Solution solution;
	vector<vector<int>> res = solution.permuteUnique(nums);
	for (auto& r : res)
	{
		for (auto& e : r)
			cout << e << " ";
		cout << endl;
	}
}

```

### [60\. Permutation Sequence](https://leetcode.com/problems/permutation-sequence/)

Difficulty: **Medium**


The set `[1,2,3,...,_n_]` contains a total of _n_! unique permutations.

By listing and labeling all of the permutations in order, we get the following sequence for _n_ = 3:

1.  `"123"`
2.  `"132"`
3.  `"213"`
4.  `"231"`
5.  `"312"`
6.  `"321"`

Given _n_ and _k_, return the _k_<sup>th</sup> permutation sequence.

**Note:**

*   Given _n_ will be between 1 and 9 inclusive.
*   Given _k_ will be between 1 and _n_! inclusive.

**Example 1:**

```
Input: n = 3, k = 3
Output: "213"
```

**Example 2:**

```
Input: n = 4, k = 9
Output: "2314"
```


#### Solution

Language: **C++**

```c++
#include <iostream>
#include <vector>
#include <string>
#include <set>

using namespace std;

class Solution {
public:
	string getPermutation(int n, int k) {
		string str;
		vector<int>fact = { 1 };

		for (int i = 1; i <= n; i++) {
			str.push_back(i + '0');
			fact.push_back(fact.back() * i);
		}

		k--;
		string res;
		for (int i = 1; i <= n; i++) {
			int index = k / fact[n - i];
			res.push_back(str[index]);
			str.erase(str.begin() + index);
			k -= index * fact[n - i];
		}
		return res;

	}
};

class Solution2
{
private:
	void backtracking(string& res, int k, vector<int>& factorial, set<int>& dict)
	{
		if (dict.size() == 1)
		{
			res += ('0' + * dict.begin());
			return;
		}
		int cnt = factorial[dict.size() - 1];
		for (const int& n : dict)
		{
			if (cnt >= k)
			{
				res += ('0' + n);
				dict.erase(n);
				backtracking(res, k, factorial, dict);
				break;
			}
			else
			{
				k -= cnt;
			}
		}
	}

public:
	string getPermutation(int n, int k) {
		vector<int> factorial(n + 1, 1);
		set<int> dict;
		for (int i = 1; i <= n; ++i)
		{
			factorial[i] = factorial[i - 1] * i;
			dict.insert(i);
		}
		string res = "";
		backtracking(res, k, factorial, dict);
		return res;
	}
};

int main()
{
	Solution solution;
	cout << solution.getPermutation(3, 3) << endl;
	cout << solution.getPermutation(4, 9) << endl;
}

```


### [77\. Combinations](https://leetcode.com/problems/combinations/)

Difficulty: **Medium**


Given two integers _n_ and _k_, return all possible combinations of _k_ numbers out of 1 ... _n_.

**Example:**

```
Input: n = 4, k = 2
Output:
[
  [2,4],
  [3,4],
  [2,3],
  [1,2],
  [1,3],
  [1,4],
]
```


#### Solution

Language: **C++**

```c++
#include <iostream>
#include <vector>

using namespace std;

class Solution
{
private:
	void backtracking(vector<vector<int>>& res, int start, int& end, int& k, vector<int>& cur)
	{
		if (cur.size() == k)
		{
			res.push_back(cur);
			return;
		}

		for (int i = start; i <= end; ++i)
		{
			if (cur.size() + end - start + 1 < k) break; // we can stop if there are not enough numbers
			cur.push_back(i);
			backtracking(res, i + 1, end, k, cur);
			cur.pop_back();
		}
	}

public:
	vector<vector<int>> combine(int n, int k) {
		vector<vector<int>> res;
		if (k == 0) return res;
		vector<int> cur;
		backtracking(res, 1, n, k, cur);
		return res;
	}
};

int main()
{
	Solution solution;
	vector<vector<int>> res = solution.combine(5, 3);
	for (auto& r : res)
	{
		for (auto& e : r)
			cout << e << " ";
		cout << endl;
	}
}

```


### [78\. Subsets](https://leetcode.com/problems/subsets/)

Difficulty: **Medium**


Given a set of **distinct** integers, _nums_, return all possible subsets (the power set).

**Note:** The solution set must not contain duplicate subsets.

**Example:**

```
Input: nums = [1,2,3]
Output:
[
  [3],
  [1],
  [2],
  [1,2,3],
  [1,3],
  [2,3],
  [1,2],
  []
]
```


#### Solution

Language: **C++**

```c++
#include <iostream>

#include <vector>

using namespace std;

class Solution {
private:
	void backtracking(vector<vector<int>>& res, vector<int>& cur, vector<int>& nums, int start)
	{
		for (int i = start; i < nums.size(); ++i)
		{
			cur.push_back(nums[i]);
			backtracking(res, cur, nums, i + 1);
			cur.pop_back();
		}
		res.push_back(cur);
	}
public:
	vector<vector<int>> subsets(vector<int>& nums) {
		vector<vector<int>> res;
		vector<int> cur;
		backtracking(res, cur, nums, 0);
		return res;
	}
};

int main()
{
	vector<int> nums{ 1, 2, 3 };
	Solution solution;
	vector<vector<int>> res = solution.subsets(nums);
	for (auto& r : res)
	{
		for (auto& e : r)
			cout << e << " ";
		cout << endl;
	}
}

```


### [89\. Gray Code](https://leetcode.com/problems/gray-code/)

Difficulty: **Medium**


The gray code is a binary numeral system where two successive values differ in only one bit.

Given a non-negative integer _n_ representing the total number of bits in the code, print the sequence of gray code. A gray code sequence must begin with 0.

**Example 1:**

```
Input: 2
Output: [0,1,3,2]
Explanation:
00 - 0
01 - 1
11 - 3
10 - 2

For a given n, a gray code sequence may not be uniquely defined.
For example, [0,2,3,1] is also a valid gray code sequence.

00 - 0
10 - 2
11 - 3
01 - 1
```

**Example 2:**

```
Input: 0
Output: [0]
Explanation: We define the gray code sequence to begin with 0.
             A gray code sequence of n has size = 2n, which for n = 0 the size is 20 = 1.
             Therefore, for n = 0 the gray code sequence is [0].
```


#### Solution

Language: **C++**

```c++
// https://leetcode.com/problems/gray-code/discuss/29881/An-accepted-three-line-solution-in-JAVA

class Solution {
public:
	vector<int> grayCode(int n) {
		vector<int> res;
		for (int i = 0; i < 1 << n; ++i)
			res.push_back(i ^ (i >> 1));
		return res;
	}
};
```


### [90\. Subsets II](https://leetcode.com/problems/subsets-ii/)

Difficulty: **Medium**


Given a collection of integers that might contain duplicates, **_nums_**, return all possible subsets (the power set).

**Note:** The solution set must not contain duplicate subsets.

**Example:**

```
Input: [1,2,2]
Output:
[
  [2],
  [1],
  [1,2,2],
  [2,2],
  [1,2],
  []
]
```


#### Solution

Language: **C++**

```c++
#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

class Solution
{
private:
	void backtracking(vector<vector<int>>& res, vector<int>& cur, vector<int>& nums, int start)
	{
		res.push_back(cur);
		for (int i = start; i < nums.size(); ++i)
		{
			if (i > start && nums[i] == nums[i - 1]) continue;
			cur.push_back(nums[i]);
			backtracking(res, cur, nums, i + 1);
			cur.pop_back();
		}
	}
public:
	vector<vector<int>> subsetsWithDup(vector<int>& nums) {
		vector<vector<int>> res;
		vector<int> cur;
		std::sort(nums.begin(), nums.end());
		backtracking(res, cur, nums, 0);
		return res;
	}
};

int main()
{
	vector<int> nums{ 1, 2, 2 };
	Solution solution;
	vector<vector<int>> res = solution.subsetsWithDup(nums);
	for (auto& r : res)
	{
		for (auto& e : r)
			cout << e << " ";
		cout << endl;
	}
}

```


### [93\. Restore IP Addresses](https://leetcode.com/problems/restore-ip-addresses/)

Difficulty: **Medium**


Given a string containing only digits, restore it by returning all possible valid IP address combinations.

**Example:**

```
Input: "25525511135"
Output: ["255.255.11.135", "255.255.111.35"]
```


#### Solution

Language: **C++**

```c++
#include <iostream>
#include <vector>
#include <string>

using namespace std;

class Solution {
public:
	vector<string> restoreIpAddresses(string s) {
		vector<string> ret;
		string ans;

		for (int a = 1; a <= 3; a++)
			for (int b = 1; b <= 3; b++)
				for (int c = 1; c <= 3; c++)
					for (int d = 1; d <= 3; d++)
						if (a + b + c + d == s.length()) {
							int A = stoi(s.substr(0, a));
							int B = stoi(s.substr(a, b));
							int C = stoi(s.substr(a + b, c));
							int D = stoi(s.substr(a + b + c, d));
							if (A <= 255 && B <= 255 && C <= 255 && D <= 255)
								if ((ans = to_string(A) + "." + to_string(B) + "." + to_string(C) + "." + to_string(D)).length() == s.length() + 3)
									ret.push_back(ans);
						}

		return ret;
	}
};

class Solution2 {
public:
	vector<string> restoreIpAddresses(string s) {
		vector<string> result;
		string ip;
		dfs(s, 0, 0, ip, result); //paras:string s,start index of s,step(from0-3),intermediate ip,final result
		return result;
	}
	void dfs(string s, int start, int step, string ip, vector<string>& result) {
		if (start == s.size() && step == 4) {
			ip.erase(ip.end() - 1); //remove the last '.' from the last decimal number
			result.push_back(ip);
			return;
		}
		if (s.size() - start > (4 - step) * 3) return;
		if (s.size() - start < (4 - step)) return;
		int num = 0;
		for (int i = start; i < start + 3; i++) {
			num = num * 10 + (s[i] - '0');
			if (num <= 255) {
				ip += s[i];
				dfs(s, i + 1, step + 1, ip + '.', result);
			}
			if (num == 0) break;
		}
	}
};


int main()
{
	Solution solution;
	vector<string> res = solution.restoreIpAddresses("25525511135");
	for (string& s : res)
		cout << s << " ";
}

```


### [131\. Palindrome Partitioning](https://leetcode.com/problems/palindrome-partitioning/)

Difficulty: **Medium**


Given a string _s_, partition _s_ such that every substring of the partition is a palindrome.

Return all possible palindrome partitioning of _s_.

**Example:**

```
Input: "aab"
Output:
[
  ["aa","b"],
  ["a","a","b"]
]
```


#### Solution

Language: **C++**

```c++
#include <iostream>
#include <string>
#include <vector>

using namespace std;

class Solution
{
public:
	vector<vector<string>> partition(string s)
	{
		vector<vector<string>> pars;
		vector<string> par;
		partition(s, 0, par, pars);
		return pars;
	}
private:
	void partition(string& s, int start, vector<string>& par, vector<vector<string>>& pars)
	{
		int n = s.length();
		if (start == n)
		{
			pars.push_back(par);
		}
		else
		{
			for (int i = start; i < n; i++)
			{
				if (isPalindrome(s, start, i))
				{
					par.push_back(s.substr(start, i - start + 1));
					partition(s, i + 1, par, pars);
					par.pop_back();
				}
			}
		}
	}

	bool isPalindrome(string& s, int l, int r)
	{
		while (l < r)
			if (s[l++] != s[r--]) return false;
		return true;
	}
};

int main()
{
	Solution solution;
	vector<vector<string>> res = solution.partition("aab");
	for (auto& r : res)
	{
		for (auto& s : r)
			cout << s << " ";
		cout << endl;
	}
}

```

### [216\. Combination Sum III](https://leetcode.com/problems/combination-sum-iii/)

Difficulty: **Medium**


Find all possible combinations of _**k**_ numbers that add up to a number _**n**_, given that only numbers from 1 to 9 can be used and each combination should be a unique set of numbers.

**Note:**

*   All numbers will be positive integers.
*   The solution set must not contain duplicate combinations.

**Example 1:**

```
Input: k = 3, n = 7
Output: [[1,2,4]]
```

**Example 2:**

```
Input: k = 3, n = 9
Output: [[1,2,6], [1,3,5], [2,3,4]]
```


#### Solution

Language: **C++**

```c++
#include <iostream>
#include <vector>

using namespace std;

class Solution
{
private:
	vector<int> dict_;
	void backtracking(vector<vector<int>>& res, vector<int>& cur, int& k, int n, int start)
	{
		if (cur.size() == k)
		{
			if (n == 0) res.push_back(cur);
			return;
		}
		for (int i = start; i <= 9; ++i)
		{
			if (i > n) break;							   // i is too large
			if (dict_[i] || n - i > (k - 1) * 9) continue; // i is too small
			cur.push_back(i);
			dict_[i] = 1;
			backtracking(res, cur, k, n - i, i + 1);
			cur.pop_back();
			dict_[i] = 0;
		}
	}
public:
	vector<vector<int>> combinationSum3(int k, int n)
	{
		vector<vector<int>> res;
		vector<int> cur;
		dict_ = vector<int>(10, 0);
		backtracking(res, cur, k, n, 1);
		return res;
	}
};

int main()
{
	Solution solution;
	vector<vector<int>> res = solution.combinationSum3(3, 9);
	for (auto& r : res)
	{
		for (auto& e : r)
			cout << e << " ";
		cout << endl;
	}
}

```


### [357\. Count Numbers with Unique Digits](https://leetcode.com/problems/count-numbers-with-unique-digits/)

Difficulty: **Medium**


Given a **non-negative** integer n, count all numbers with unique digits, x, where 0 ≤ x < 10<sup>n</sup>.


**Example:**

```
Input: 2
Output: 91
Explanation: The answer should be the total numbers in the range of 0 ≤ x < 100,
             excluding 11,22,33,44,55,66,77,88,99
```


#### Solution

Language: **C++**

```c++
class Solution {
public:
    int countNumbersWithUniqueDigits(int n) {
        if (n == 0) return 1;
        if (n > 10) return 0;
        int cnt = 10;
        int cnt_i = 9; // i: number of digits
        int possible = 9;
        for (int i = 2; i <= n; ++i, --possible)
        {
            cnt_i * = possible;
            cnt += cnt_i;
        }
        return cnt;
    }
};
```


### [526\. Beautiful Arrangement](https://leetcode.com/problems/beautiful-arrangement/)

Difficulty: **Medium**


Suppose you have **N** integers from 1 to N. We define a beautiful arrangement as an array that is constructed by these **N** numbers successfully if one of the following is true for the i<sub style="display: inline;">th</sub> position (1 <= i <= N) in this array:

1.  The number at the i<sub style="display: inline;">th</sub> position is divisible by **i**.
2.  **i** is divisible by the number at the i<sub style="display: inline;">th</sub> position.

Now given N, how many beautiful arrangements can you construct?

**Example 1:**

```
Input: 2
Output: 2
Explanation:

The first beautiful arrangement is [1, 2]:

Number at the 1st position (i=1) is 1, and 1 is divisible by i (i=1).

Number at the 2nd position (i=2) is 2, and 2 is divisible by i (i=2).

The second beautiful arrangement is [2, 1]:

Number at the 1st position (i=1) is 2, and 2 is divisible by i (i=1).

Number at the 2nd position (i=2) is 1, and i (i=2) is divisible by 1.
```

**Note:**

1.  **N** is a positive integer and will not exceed 15.


#### Solution

Language: **C++**

```c++
class Solution {
public:
    int countArrangement(int N) {
        
    }
};
```


### [842\. Split Array into Fibonacci Sequence](https://leetcode.com/problems/split-array-into-fibonacci-sequence/)

Difficulty: **Medium**


Given a string `S` of digits, such as `S = "123456579"`, we can split it into a _Fibonacci-like sequence_ `[123, 456, 579].`

Formally, a Fibonacci-like sequence is a list `F` of non-negative integers such that:

*   `0 <= F[i] <= 2^31 - 1`, (that is, each integer fits a 32-bit signed integer type);
*   `F.length >= 3`;
*   and `F[i] + F[i+1] = F[i+2]` for all `0 <= i < F.length - 2`.

Also, note that when splitting the string into pieces, each piece must not have extra leading zeroes, except if the piece is the number 0 itself.

Return any Fibonacci-like sequence split from `S`, or return `[]` if it cannot be done.

**Example 1:**

```
Input: "123456579"
Output: [123,456,579]
```

**Example 2:**

```
Input: "11235813"
Output: [1,1,2,3,5,8,13]
```

**Example 3:**

```
Input: "112358130"
Output: []
Explanation: The task is impossible.
```

**Example 4:**

```
Input: "0123"
Output: []
Explanation: Leading zeroes are not allowed, so "01", "2", "3" is not valid.
```

**Example 5:**

```
Input: "1101111"
Output: [110, 1, 111]
Explanation: The output [11, 0, 11, 11] would also be accepted.
```

**Note:**

1.  `1 <= S.length <= 200`
2.  `S` contains only digits.


#### Solution

Language: **C++**

```c++
class Solution {
public:
    vector<int> splitIntoFibonacci(string S) {
        
    }
};
```



### [1079\. Letter Tile Possibilities](https://leetcode.com/problems/letter-tile-possibilities/)

Difficulty: **Medium**


You have a set of `tiles`, where each tile has one letter `tiles[i]` printed on it.  Return the number of possible non-empty sequences of letters you can make.

**Example 1:**

```
Input: "AAB"
Output: 8
Explanation: The possible sequences are "A", "B", "AA", "AB", "BA", "AAB", "ABA", "BAA".
```


**Example 2:**

```
Input: "AAABBC"
Output: 188
```


**Note:**

1.  `1 <= tiles.length <= 7`
2.  `tiles` consists of uppercase English letters.


#### Solution

Language: **C++**

```c++
class Solution {
public:
    int numTilePossibilities(string tiles) {
        
    }
};
```
