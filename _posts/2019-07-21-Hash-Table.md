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


### [49. Group Anagrams](https://leetcode.com/problems/group-anagrams/)

#### Question

Given an array of strings, group anagrams together.

__Example:__
```
Input: ["eat", "tea", "tan", "ate", "nat", "bat"],
Output:
[
  ["ate","eat","tea"],
  ["nat","tan"],
  ["bat"]
]
```

__Note:__
- All inputs will be in lowercase.
- The order of your output does not matter.

#### Train of Thought

All the anagrams have the same property: they use the same characters. Therefore, if we sort the characters used in the string, the new strings of anagrams should be the same. According to this, we can build a `unordered_map` to save the strings of anagrams into a `vector`.

#### Solution
```cpp
#include <algorithm>

#include <iostream>

#include <string>

#include <unordered_map>

#include <vector>

using std::cout;
using std::endl;
using std::unordered_map;
using std::vector;
using std::string;

class Solution {
public:
	vector<vector<string>> groupAnagrams(vector<string>& strs) {
		vector<vector<string>> anas;
		unordered_map<string, vector<string>> map_anas;
		for (const string& s : strs)
		{
			string sorted_str = s;
			std::sort(sorted_str.begin(), sorted_str.end());
			map_anas[sorted_str].push_back(s);
		}

		for (const auto& m_anas : map_anas)
			anas.push_back(m_anas.second);
		return anas;
	}
};

int main()
{
	vector<string> test1{ "eat", "tea", "tan", "ate", "nat", "bat" };
	Solution solution;
	vector<vector<string>> res1 = solution.groupAnagrams(test1);
	for (const auto& v_s : res1)
	{
		for (const auto& s : v_s)
			cout << s << " ";
		cout << endl;
	}
}

```


### [138. Copy List with Random Pointer](https://leetcode.com/problems/copy-list-with-random-pointer/)

#### Question

A linked list is given such that each node contains an additional random pointer which could point to any node in the list or null.

Return a __deep copy__ of the list.

__Example:__
```
Input:
{"$id":"1","next":{"$id":"2","next":null,"random":{"$ref":"2"},"val":2},"random":{"$ref":"2"},"val":1}

Explanation:
Node 1's value is 1, both of its next and random pointer points to Node 2.
Node 2's value is 2, its next pointer points to null and its random pointer points to itself.
```

__Note:__
You must return the __copy of the given head__ as a reference to the cloned list.

#### Train of Thought


#### Solution
```cpp
#include <iostream>

class Node {
public:
	int val;
	Node* next;
	Node* random;

	Node() {}

	Node(int _val, Node* _next, Node* _random) {
		val = _val;
		next = _next;
		random = _random;
	}
};

class Solution {
public:
	Node* copyRandomList(Node* head) {
		if (!head)
			return nullptr;

		// copy nodes
		Node* cur = head;
		while (cur)
		{
			Node* new_node = new Node();
			new_node->val = cur->val;
			new_node->next = cur->next;
			cur->next = new_node;
			cur = cur->next->next;
		}

		// assign random pointer
		cur = head;
		while (cur)
		{
			if (cur->random)
				cur->next->random = cur->random->next;
			else
				cur->next->random = nullptr;
			cur = cur->next->next;
		}

		// split nodes
		cur = head;
		Node* new_head = head->next;
		Node* new_cur = new_head;
		while (cur)
		{
			cur->next = cur->next->next;
			if (cur->next)
				new_cur->next = cur->next->next;
			else
				new_cur->next = nullptr;
			cur = cur->next;
			new_cur = new_cur->next;
		}

		return new_head;
	}
};

int main()
{
    std::cout << "Hello World!\n";
}
```

### [347. Top K Frequent Elements](https://leetcode.com/problems/top-k-frequent-elements/)

Given a non-empty array of integers, return the __k__ most frequent elements.

__Example 1:__
```
Input: nums = [1,1,1,2,2,3], k = 2
Output: [1,2]
```

__Example 2:__
```
Input: nums = [1], k = 1
Output: [1]
```

__Note:__
- You may assume k is always valid, 1 ≤ k ≤ number of unique elements.
- Your algorithm's time complexity __must be__ better than O(nlogn), where n is the array's size.

#### Train of Thought

To find the k most frequent elements, we need to document how many times each number appears. To do that, we can use a unordered_map. Then, to rank from the most frequent to the least frequent element, we can use a priority_queue (in <queue>). The last step is just poping the top element out until we get k elements.

#### Solution
```cpp
#include <utility>

#include <iostream>

#include <map>

#include <unordered_map>

#include <vector>

#include <queue>

using std::cout;
using std::endl;
using std::vector;
using std::unordered_map;
using std::map;
using std::priority_queue;
using std::pair;

class Solution1
{
public:
	vector<int> topKFrequent(vector<int>& nums, int k)
	{
		unordered_map<int, int> map_nums;
		for (const int& n : nums)
			++map_nums[n];


		priority_queue<pair<int, int>> freq_nums;
		for (const auto& mn : map_nums)
			freq_nums.push({ mn.second, mn.first });

		vector<int> res;
		while (!freq_nums.empty() && k--)
		{
			res.push_back(freq_nums.top().second);
			freq_nums.pop();
		}
		return res;
	}
};

class Solution2
{
public:
	vector<int> topKFrequent(vector<int>& nums, int k)
	{
		unordered_map<int, int> map_nums;
		for (const int& n : nums)
			++map_nums[n];

		map<int, vector<int>, std::greater<int>> freq_nums;
		for (const auto& mn : map_nums)
			freq_nums[mn.second].push_back(mn.first);

		vector<int> res;
		for (const auto& fn : freq_nums)
		{
			if (res.size() >= k)
				break;
			res.insert(res.end(), fn.second.begin(), fn.second.end());
		}
		return res;
	}
};

std::ostream& operator<<(std::ostream& stream, const vector<int>& nums)
{
	for (const auto& n : nums)
		stream << n << " ";
	stream << endl;
	return stream;
}

int main()
{
	Solution1 solution;
	vector<int> nums1{ 1,1,1,2,2,3 };
	vector<int> nums2{ 1,2 };
	cout << solution.topKFrequent(nums1, 2) << endl;
	cout << solution.topKFrequent(nums2, 2) << endl;
}
```
