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
class Solution {
public:
    vector<string> letterCombinations(string digits) {
        
    }
};
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
class Solution {
public:
    vector<string> generateParenthesis(int n) {
        
    }
};
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
class Solution {
public:
    vector<vector<int>> combinationSum(vector<int>& candidates, int target) {
        
    }
};
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
class Solution {
public:
    vector<vector<int>> combinationSum2(vector<int>& candidates, int target) {
        
    }
};
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
class Solution {
public:
    vector<vector<int>> permute(vector<int>& nums) {
        
    }
};
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
class Solution {
public:
    vector<vector<int>> permuteUnique(vector<int>& nums) {
        
    }
};
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
class Solution {
public:
    string getPermutation(int n, int k) {
        
    }
};
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
class Solution {
public:
    vector<vector<int>> combine(int n, int k) {
        
    }
};
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
class Solution {
public:
    vector<vector<int>> subsets(vector<int>& nums) {
        
    }
};
```



### [79\. Word Search](https://leetcode.com/problems/word-search/)

Difficulty: **Medium**


Given a 2D board and a word, find if the word exists in the grid.

The word can be constructed from letters of sequentially adjacent cell, where "adjacent" cells are those horizontally or vertically neighboring. The same letter cell may not be used more than once.

**Example:**

```
board =
[
  ['A','B','C','E'],
  ['S','F','C','S'],
  ['A','D','E','E']
]

Given word = "ABCCED", return true.
Given word = "SEE", return true.
Given word = "ABCB", return false.
```


#### Solution

Language: **C++**

```c++
class Solution {
public:
    bool exist(vector<vector<char>>& board, string word) {
        
    }
};
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
class Solution {
public:
    vector<int> grayCode(int n) {
        
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
class Solution {
public:
    vector<vector<int>> subsetsWithDup(vector<int>& nums) {
        
    }
};
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
class Solution {
public:
    vector<string> restoreIpAddresses(string s) {
        
    }
};
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
class Solution {
public:
    vector<vector<string>> partition(string s) {
        
    }
};
```

### [211\. Add and Search Word - Data structure design](https://leetcode.com/problems/add-and-search-word-data-structure-design/)

Difficulty: **Medium**


Design a data structure that supports the following two operations:

```
void addWord(word)
bool search(word)
```

search(word) can search a literal word or a regular expression string containing only letters `a-z` or `.`. A `.` means it can represent any one letter.

**Example:**

```
addWord("bad")
addWord("dad")
addWord("mad")
search("pad") -> false
search("bad") -> true
search(".ad") -> true
search("b..") -> true
```

**Note:**  
You may assume that all words are consist of lowercase letters `a-z`.


#### Solution

Language: **C++**

```c++
class WordDictionary {
public:
    /** Initialize your data structure here. */
    WordDictionary() {
        
    }
    
    /** Adds a word into the data structure. */
    void addWord(string word) {
        
    }
    
    /** Returns if the word is in the data structure. A word could contain the dot character '.' to represent any one letter. */
    bool search(string word) {
        
    }
};
​
/**
 * Your WordDictionary object will be instantiated and called as such:
 * WordDictionary* obj = new WordDictionary();
 * obj->addWord(word);
 * bool param_2 = obj->search(word);
 */
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
class Solution {
public:
    vector<vector<int>> combinationSum3(int k, int n) {
        
    }
};
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
