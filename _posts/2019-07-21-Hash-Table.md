---
layout:     post
title:      "Hash Table"
date:       2019-7-21
author:     Tong
catalog: true
tags:
    - Algorithm
---

### Summary

1.


### [3. Longest Substring Without Repeating Characters](https://leetcode.com/problems/longest-substring-without-repeating-characters/)

#### Question

Given a string, find the length of the __longest substring__ without repeating characters.

__Example 1:__
```
Input: "abcabcbb"
Output: 3
Explanation: The answer is "abc", with the length of 3.
```

__Example 2:__
```
Input: "bbbbb"
Output: 1
Explanation: The answer is "b", with the length of 1.
```

__Example 3:__
```
Input: "pwwkew"
Output: 3
Explanation: The answer is "wke", with the length of 3.
             Note that the answer must be a substring, "pwke" is a subsequence and not a substring.
```

#### Train of Thought

For example, we have `abcabcbb`. If we do it manually, we will start from the first character `a`. Until `c`, everything is fine. Then, `a` occurs again. Then, we should cut the substring off. Specifically, we now consider `bca`. Then continually, we will cut off `b` and get `cab`. Same, we get then `abc`. Then, `cb`. Then, `b`.

This algorithm means that we need to remember the last index of the current character, and then cut some characters off. One thing needs to be paid attention to, namely, for `abcba`, once we cut `b` off and get `cb`. Then, when the last `a` comes, we do not need to change anything, because we already cut it off.

To remember the last index of some character, we can use either `unordered_map` or `vector<int>`.

#### Solution
```cpp
#include <algorithm>

#include <iostream>

#include <string>

#include <unordered_map>

#include <vector>

using std::cout;
using std::endl;
using std::string;
using std::unordered_map;
using std::vector;

// use unordered_map
class Solution1 {
public:
	int lengthOfLongestSubstring(string s) {
		unordered_map<char, int> char_map;
		int longest = 0;
		int start = -1;
		for (int i = 0; i < s.size(); ++i)
		{
			if (char_map.find(s[i]) != char_map.end())
				if (char_map[s[i]] > start)
					start = char_map[s[i]];

			char_map[s[i]] = i;

			if (i - start > longest)
				longest = i - start;
		}
		return longest;
	}
};

// use vector
class Solution2 {
public:
	int lengthOfLongestSubstring(string s)
	{
		vector<int> dict(256, -1);
		int start = -1;
		int maxLen = 0;
		for (int i = 0; i < s.size(); ++i)
		{
			if (dict[s[i]] > start)
				start = dict[s[i]];
			dict[s[i]] = i;
			maxLen = std::max(maxLen, i - start);
		}

		return maxLen;
	}
};

int main()
{
	string a = "abc";
	string b = "aba";
	Solution1 solution;
	cout << solution.lengthOfLongestSubstring(a) << endl;
	cout << solution.lengthOfLongestSubstring(b) << endl;
}
```
