---
layout:     post
title:      "Knapsack Problem"
date:       2019-9-1
author:     Tong
catalog: true
tags:
    - Algorithm
---

### 参考资料

1. https://github.com/tianyicui/pack/blob/master/V2.pdf

2. https://www.jianshu.com/p/26d8dbeacf19

3. https://blog.csdn.net/u013166817/article/details/85449218

### 背包问题 I —— [0-1背包无价值](https://www.lintcode.com/problem/backpack/description)


在`n`个物品中挑选若干物品装入背包，最多能装多满？假设背包的大小为`m`，每个物品的大小为`A[i]`

你不可以将物品进行切割。


**Example**

样例 1:
```
输入:  [3,4,8,5], backpack size=10
输出:  9
```

样例 2:
```
输入:  [2,3,5,7], backpack size=12
输出:  12
```

**Challenge**
`O(n x m)`的时间复杂度 and `O(m)`空间复杂度
如果不知道如何优化空间`O(n x m)`的空间复杂度也可以通过.


```c++
#include <iostream>
#include <vector>
#include <algorithm>
using namespace std;

class Solution
{
public:
	/**
	 * @param m: An integer m denotes the size of a backpack
	 * @param A: Given n items with size A[i]
	 * @return: The maximum size
	 */
	int backPack(int m, vector<int>& A)
	{
		vector<int> capacity(m + 1, 0);

		// 为什么i从m开始？
		// 因为0...m-1的部分并没有使用当前物体a，从而避免循环计算
		// capacity[i - a]代表了我们使用a之前的物体能达到的最大capacity
		for (auto& a : A)
			for (int i = m; i >= a; --i)
				capacity[i] = max(capacity[i], a + capacity[i - a]);

		return capacity[m];
	}
};

int main()
{
	Solution solution;
	vector<int> A{ 3, 4, 8, 5 };
	cout << solution.backPack(10, A) << endl;
}

```


### 背包问题 II —— [0-1背包有价值](https://www.lintcode.com/problem/backpack-ii/)

有 `n` 个物品和一个大小为 `m` 的背包. 给定数组 `A` 表示每个物品的大小和数组 `V` 表示每个物品的价值.

问最多能装入背包的总价值是多大?

```
1. A[i], V[i], n, m 均为整数
2. 你不能将物品进行切分
3. 你所挑选的要装入背包的物品的总大小不能超过 m
4. 每个物品只能取一次
```

**样例1**
```
输入: m = 10, A = [2, 3, 5, 7], V = [1, 5, 2, 4]
输出: 9
解释: 装入 A[1] 和 A[3] 可以得到最大价值, V[1] + V[3] = 9
```

**样例2**
```
输入: m = 10, A = [2, 3, 8], V = [2, 5, 8]
输出: 10
解释: 装入 A[0] 和 A[2] 可以得到最大价值, V[0] + V[2] = 10
```

**挑战**
`O(nm)` 空间复杂度可以通过, 不过你可以尝试 `O(m)` 空间复杂度吗?

```c++
#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;


class Solution
{
public:
	/**
	 * @param m: An integer m denotes the size of a backpack
	 * @param A: Given n items with size A[i]
	 * @param V: Given n items with value V[i]
	 * @return: The maximum value
	 */
	int backPackII(int m, vector<int>& A, vector<int>& V)
	{
		vector<int> val(m + 1, 0);
		for (int i = 0; i < A.size(); ++i) // object index
			for (int c = m; c >= A[i]; --c)
				val[c] = max(val[c], V[i] + val[c - A[i]]);
		return val[m];
	}
};

int main()
{
	int m = 10;
	vector<int> A{ 2, 3, 5, 7 };
	vector<int> V{ 1, 5, 2, 4 };
	Solution solution;
	cout << solution.backPackII(m, A, V) << endl;
}

```



### 背包问题 III —— [完全背包问题](https://www.lintcode.com/problem/backpack-iii/)

给定`n`种具有大小 `Ai` 和价值 `Vi` 的物品(每个物品可以取用无限次)和一个大小为 `m` 的一个背包, 你可以放入背包里的最大价值是多少?

**样例**
给出四个物品, 大小为 `[2, 3, 5, 7]`, 价值为 `[1, 5, 2, 4]`, 和一个大小为 10 的背包. 最大的价值为 15.

**注意事项**
你不能将物品分成小块, 选择的项目的总大小应 小于或等于 m

```c++
#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;


class Solution
{
public:
	/**
	 * @param m: An integer m denotes the size of a backpack
	 * @param A: Given n items with size A[i]
	 * @param V: Given n items with value V[i]
	 * @return: The maximum value
	 */
	int backPackIII(int m, vector<int>& A, vector<int>& V)
	{
		vector<int> val(m + 1, 0);
		for (int i = 0; i < A.size(); ++i) // 用前(i + 1)个物体（必须包含第(i+1)个）
			for (int c = A[i]; c <= m; ++c) // // 注意c递增，为了多次运用以前用过的物品
				val[c] = max(val[c], V[i] + val[c - A[i]]);
		return val[m];
	}
};

int main()
{
	int m = 20;
	vector<int> A{ 2, 3, 5, 7, 10 };
	vector<int> V{ 1, 5, 2, 4, 21 };
	Solution solution;
	cout << solution.backPackII(m, A, V) << endl;
}
```


### 背包问题 IV —— [完全背包问题，返回方案数](https://www.lintcode.com/problem/backpack-iV/)


给出 `n` 个物品, 以及一个数组, `nums[i]`代表第`i`个物品的大小, 保证大小均为正数并且没有重复, 正整数 `target` 表示背包的大小, 找到能填满背包的方案数。
每一个物品可以使用无数次

**Example**

样例 1:
```
给出候选物品集合 [2,3,6,7] 和 target 7,

[7]
[2,2,3]

返回 2
```


```c++
#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;


class Solution
{
public:
	/**
	 * @param A: Given n items with size A[i]
	 * @param target: target
	 * @return: The number of combinations
	 */
	int backPackIV(vector<int>& A, int target)
	{
		vector<int> res(target + 1, 0);
		res[0] = 1;								 // target为0时算一种，即什么也不用
		for (int i = 0; i < A.size(); ++i)		 // 用前(i + 1)个物体（必须包含第(i+1)个）
			for (int j = A[i]; j <= target; ++j) // 注意j递增，为了多次运用以前用过的物品
				res[j] += res[j - A[i]];
		return res.back();
	}
};


int main()
{
	vector<int> A{ 2, 3, 6, 5 };
	Solution solution;
	cout << solution.backPackIV(A, 9) << endl;
}
```

### 背包问题 V —— [0-1 背包问题，返回方案数](https://www.lintcode.com/problem/backpack-iV/)


给出 `n` 个物品, 以及一个数组, `nums[i]` 代表第`i`个物品的大小, 保证大小均为正数并且没有重复, 正整数 `target` 表示背包的大小, 找到能填满背包的方案数。
每一个物品只能使用一次

**Example**

样例 1:
```
给出候选物品集合 [1,2,3,3,7] 以及 target 7

[7]
[1,3,3]

返回 2
```


```c++
#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;


class Solution
{
public:
	/**
	 * @param A: Given n items with size A[i]
	 * @param target: target
	 * @return: The number of combinations
	 */
	int backPackIV(vector<int>& A, int target)
	{
		vector<int> res(target + 1, 0);
		res[0] = 1;								 // target为0时算一种，即什么也不用
		for (int i = 0; i < A.size(); ++i)		 // 用前(i + 1)个物体（必须包含第(i+1)个）
			for (int j = target; j >= A[i]; --j) // 注意j递减，为了只用一次以前用过的物品
				res[j] += res[j - A[i]];
		return res.back();
	}
};


int main()
{
	vector<int> A{ 1, 2, 3, 3, 7 };
	Solution solution;
	cout << solution.backPackIV(A, 7) << endl;
}
```



### 背包问题 VII —— [多重，价值](https://www.lintcode.com/problem/backpack-vii/description)


假设你身上有 n 元，超市里有多种大米可以选择，每种大米都是袋装的，必须整袋购买，给出每种大米的重量，价格以及数量，求最多能买多少公斤的大米

**Example**

样例 1:
```
Given:
n = 8
prices = [2,4]
weight = [100,100]
amounts = [4,2]

Return:400
```


```c++
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

using namespace std;


class Solution
{
public:
	/**
	 * @param n: the money of you
	 * @param prices: the price of rice[i]
	 * @param weight: the weight of rice[i]
	 * @param amounts: the amount of rice[i]
     * @return: the maximum weight
	 */
	int backPackVII(int n, vector<int>& prices, vector<int>& weight, vector<int>& amounts)
	{
		vector<int> new_prices;
		vector<int> new_weight;
		for (int i = 0; i < amounts.size(); ++i)
		{
			int k = (int)log2(amounts[i]) + 1;  // eg: log2(10) = 3.32.. --> k = 4
			for (int j = 0; j < k - 1; ++j) // 10 = 1 + 2 + 4 + 3
			{
				int coef = (int)pow(2, j);
				new_prices.push_back(coef * prices[i]);
				new_weight.push_back(coef * weight[i]);
			}
			int coef = amounts[i] - ((int)pow(2, k - 1) - 1);
			new_prices.push_back(coef * prices[i]);
			new_weight.push_back(coef * weight[i]);
		}

		// The problem becomes a 0-1 backpack
		vector<int> dp(n + 1, 0);
		for (int i = 0; i < new_prices.size(); ++i) // object index
			for (int c = n; c >= new_prices[i]; --c) // 注意c从m开始递减，防止多次运用以前用过的物品
				dp[c] = max(dp[c], new_weight[i] + dp[c - new_prices[i]]);
		return dp[n];
	}
};


int main()
{
	vector<int> prices{ 2, 3, 4 };
	vector<int> weight{ 100, 300, 300 };
	vector<int> amounts{ 4, 1, 3};
	Solution solution;
	cout << solution.backPackVII(8, prices, weight, amounts) << endl;
}
```




### 背包问题 VIII —— [多重，组合数](https://www.lintcode.com/problem/backpack-viii/description)


给一些不同价值和数量的硬币。找出这些硬币可以组合在1 ~ n范围内的值

**Example**

样例 1:
```
Given:
n = 10
value = [1,2,4]
amount = [2,1,1]

Return: 8
They can combine all the values in 1 ~ 8
```


```c++
#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

// O(VN)
class Solution1
{
public:
	/**
	 * @param n: the value from 1 - n
	 * @param value: the value of coins
	 * @param amount: the number of coins
	 * @return: how many different value
	 */
	int backPackVII(int n, vector<int>& value, vector<int>& amount)
	{
		vector<int> dp(n + 1, -1);
		dp[0] = 0;

		// dp[i][j]表示“用了前i种物品填满容量为j的背包后，最多还剩下几个第i种物品可用”
		for (int i = 0; i < value.size(); ++i)
		{
			// 如果我们根本不适用第i个物品
			// 有两种情况： 1）前i-1个物品已经能构成j， 2）前i-1个物品不能构成j
			for (int j = 0; j <= n; ++j)
			{
				if (dp[j] >= 0) dp[j] = amount[i]; // 前i-1个物体已经能填满j了，所以我们不需要使用第i个，所以剩下amount[i]
				else dp[j] = -1;				   // 前i-1个物体不能填满j，状态为-1
			}

			for (int j = 0; j <= n - value[i]; ++j)
				if (dp[j] > 0)	// 我们无法在dp[j]==0的地方累加，因为物体都用完了
					dp[j + value[i]] = max(dp[j + value[i]], dp[j] - 1); // 每用一个value[i], 对应的dp[j]就要少一个, 所以dp[j]-1
		}

		int cnt = 0;
		for (int i = 1; i <= n; ++i)
			if (dp[i] >= 0) ++cnt;
		return cnt;
	}
};


// O(V sum(log2(M)))
class Solution2
{
public:
	/**
	 * @param n: the value from 1 - n
	 * @param value: the value of coins
	 * @param amount: the number of coins
	 * @return: how many different value
	 */
	int backPackVII(int n, vector<int>& value, vector<int>& amount)
	{

		vector<int> new_value;
		for (int i = 0; i < amount.size(); ++i)
		{
			int k = (int)log2(amount[i]) + 1;
			for (int j = 0; j < k - 1; ++j)
				new_value.push_back((int)pow(2, j) * value[i]);
			int rest = amount[i] - ((int)pow(2, k - 1) - 1);
			new_value.push_back(rest * value[i]);
		}


		// 0-1 backpack
		vector<int> dp(n + 1, 0);
		dp[0] = 1;
		int cnt = 0;
		for (int i = 0; i < new_value.size(); ++i)
			for (int j = n; j >= new_value[i]; --j)
				if (dp[j] == 0 && dp[j - new_value[i]])
				{
					dp[j] = 1;
					++cnt;
				}
		return cnt;
	}
};


int main()
{
	vector<int> value{ 1, 2, 4, 3 };
	vector<int> amount{ 2, 1, 1, 2 };
	Solution1 solution1;
	Solution2 solution2;
	cout << solution1.backPackVII(20, value, amount) << endl;
	cout << solution2.backPackVII(20, value, amount) << endl;
}
```


### 背包问题 IX —— [多重，最好方案](https://www.lintcode.com/problem/backpack-ix/description)


你总共有n万元，希望申请国外的大学，要申请的话需要交一定的申请费用，给出每个大学的申请费用以及你得到这个大学offer的成功概率，大学的数量是 m。如果经济条件允许，你可以申请多所大学。找到获得至少一份工作的最高可能性。

**Example**

样例 1:
```
给定:
n = 10
prices = [4,4,5]
probability = [0.1,0.2,0.3]

返回:0.440
```



**注意事项**
0<=n<=10000,0<=m<=10000


```c++
#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;


class Solution
{
public:
	/**
	 * @param n: Your money
	 * @param prices: Cost of each university application
	 * @param probability: Probability of getting the University's offer
	 * @return: the highest probability
	 */
	double backPackIX(int n, vector<int>& prices, vector<double>& probability)
	{

		vector<double> probability_rejected;
		for (double& p : probability)
			probability_rejected.push_back(1 - p);


		vector<double> dp(n + 1, 1); // get the min probability of rejection
		for (int i = 0; i < prices.size(); ++i)
			for (int j = n; j >= prices[i]; --j)
				dp[j] = min(dp[j], dp[j - prices[i]] * probability_rejected[i]);

		return 1 - dp[n];
	}
};


int main()
{
	vector<int> prices{ 4, 4, 5 };
	vector<double> probability{ 0.1, 0.2, 0.3 };
	Solution solution;
	cout << solution.backPackIX(10, prices, probability) << endl;
}
```


### [322\. Coin Change](https://leetcode.com/problems/coin-change/)

Difficulty: **Medium**


You are given coins of different denominations and a total amount of money _amount_. Write a function to compute the fewest number of coins that you need to make up that amount. If that amount of money cannot be made up by any combination of the coins, return `-1`.

**Example 1:**

```
Input: coins = [1, 2, 5], amount = 11
Output: 3
Explanation: 11 = 5 + 5 + 1
```

**Example 2:**

```
Input: coins = [2], amount = 3
Output: -1
```

**Note**:  
You may assume that you have an infinite number of each kind of coin.


#### Solution

Language: **C++**

```c++
#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

class Solution
{
public:
	int coinChange(vector<int>& coins, int amount)
	{
		if (amount == 0) return 0;

		// if dp[amount] > amount, that means we cannot construct amount using coins.
		vector<int> dp(amount + 1, amount + 1);
		dp[0] = 0;

		for (int i = 1; i <= amount; ++i)
			for (int j = 0; j < coins.size(); ++j)
				if (coins[j] <= i) dp[i] = min(dp[i], dp[i - coins[j]] + 1);

		return dp[amount] > amount ? -1 : dp[amount];
	}
};

int main()
{
	vector<int> coins1{ 1, 2, 5 };
	vector<int> coins2{ 333, 243, 214, 132, 281 };
	Solution solution;
	cout << solution.coinChange(coins1, 11) << endl;
	cout << solution.coinChange(coins2, 9334) << endl;
}

```


### [518\. Coin Change 2](https://leetcode.com/problems/coin-change-2/)

Difficulty: **Medium**


You are given coins of different denominations and a total amount of money. Write a function to compute the number of combinations that make up that amount. You may assume that you have infinite number of each kind of coin.

**Example 1:**

```
Input: amount = 5, coins = [1, 2, 5]
Output: 4
Explanation: there are four ways to make up the amount:
5=5
5=2+2+1
5=2+1+1+1
5=1+1+1+1+1
```

**Example 2:**

```
Input: amount = 3, coins = [2]
Output: 0
Explanation: the amount of 3 cannot be made up just with coins of 2.
```

**Example 3:**

```
Input: amount = 10, coins = [10]
Output: 1
```

**Note:**

You can assume that

*   0 <= amount <= 5000
*   1 <= coin <= 5000
*   the number of coins is less than 500
*   the answer is guaranteed to fit into signed 32-bit integer


#### Solution

Language: **C++**

```c++

#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

class Solution
{
public:
	int change(int amount, vector<int>& coins)
	{
		vector<int> dp(amount + 1, 0);
		dp[0] = 1;
		for (int i = 0; i < coins.size(); ++i)
			for (int j = 0; j <= amount - coins[i]; ++j)
				if (dp[j]) dp[j + coins[i]] += dp[j];
		return dp[amount];
	}
};

int main()
{
	vector<int> coins1{ 1, 2, 5 };
	vector<int> coins2{ 333, 243, 214, 132, 281 };
	Solution solution;
	cout << solution.change(11, coins1) << endl;
	cout << solution.change(9334, coins2) << endl;
}

```


### [474\. Ones and Zeroes](https://leetcode.com/problems/ones-and-zeroes/)

Difficulty: **Medium**


In the computer world, use restricted resource you have to generate maximum benefit is what we always want to pursue.

For now, suppose you are a dominator of **m** `0s` and **n** `1s` respectively. On the other hand, there is an array with strings consisting of only `0s` and `1s`.

Now your task is to find the maximum number of strings that you can form with given **m** `0s` and **n** `1s`. Each `0` and `1` can be used at most **once**.

**Note:**

1.  The given numbers of `0s` and `1s` will both not exceed `100`
2.  The size of given string array won't exceed `600`.

**Example 1:**

```
Input: Array = {"10", "0001", "111001", "1", "0"}, m = 5, n = 3
Output: 4

Explanation: This are totally 4 strings can be formed by the using of 5 0s and 3 1s, which are “10,”0001”,”1”,”0”
```

**Example 2:**

```
Input: Array = {"10", "0", "1"}, m = 1, n = 1
Output: 2

Explanation: You could form "10", but then you'd have nothing left. Better form "0" and "1".
```


#### Solution

Language: **C++**

```c++
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>

using namespace std;

class Solution {
public:
	int findMaxForm(vector<string>& strs, int m, int n)
	{
		// dp[i][j]: the max number of combinations using i '0' and j '1'
		vector<vector<int>> dp(m + 1, vector<int>(n + 1, 0));
		for (string& s : strs)
		{
			int zeros = count(s.begin(), s.end(), '0');
			int ones = count(s.begin(), s.end(), '1');

			for (int i = m; i >= zeros; --i)
				for (int j = n; j >= ones; --j)
					dp[i][j] = max(dp[i][j], dp[i - zeros][j - ones] + 1);
		}
		return dp[m][n];
	}
};

int main()
{
	vector<string> strs{ "10", "0001", "111001", "1", "0" };
	Solution solution;
	cout << solution.findMaxForm(strs, 5, 3) << endl;
}

```



### [494\. Target Sum](https://leetcode.com/problems/target-sum/)

Difficulty: **Medium**


You are given a list of non-negative integers, a1, a2, ..., an, and a target, S. Now you have 2 symbols `+` and `-`. For each integer, you should choose one from `+` and `-` as its new symbol.

Find out how many ways to assign symbols to make sum of integers equal to target S.

**Example 1:**  

```
Input: nums is [1, 1, 1, 1, 1], S is 3\.
Output: 5
Explanation:

-1+1+1+1+1 = 3
+1-1+1+1+1 = 3
+1+1-1+1+1 = 3
+1+1+1-1+1 = 3
+1+1+1+1-1 = 3

There are 5 ways to assign symbols to make the sum of nums be target 3.
```

**Note:**  

1.  The length of the given array is positive and will not exceed 20\.
2.  The sum of elements in the given array will not exceed 1000.
3.  Your output answer is guaranteed to be fitted in a 32-bit integer.


#### Solution

Language: **C++**

```c++
#include <iostream>
#include <vector>
#include <unordered_map>
#include <numeric>

using namespace std;

// 1D DP
// https://www.youtube.com/watch?v=zks6mN06xdQ&t=9s
class Solution0
{
public:
	int findTargetSumWays(vector<int>& nums, int S)
	{
		S = abs(S);
		const int sum = accumulate(nums.begin(), nums.end(), 0);
		if (sum < S || (S + sum) % 2 == 1) return 0;
		const int target = (S + sum) / 2;
		vector<int> dp(target + 1, 0);
		dp[0] = 1;
		for (const int& num : nums)
			for (int j = target; j >= num; --j)
				dp[j] += dp[j - num];
		return dp[target];
	}
};


// 1D DP
// https://www.youtube.com/watch?v=r6Wz4W1TbuI&t=139s
class Solution1
{
public:
	int findTargetSumWays(vector<int>& nums, int S)
	{
		const int n = nums.size();
		const int sum = accumulate(nums.begin(), nums.end(), 0);
		if (sum < abs(S)) return 0;
		const int offset = sum;
		const int max_n = sum * 2 + 1;
		vector<int> ways(max_n, 0);
		ways[offset] = 1;
		for (int i = 0; i < n; ++i)
		{
			vector<int> tmp(max_n, 0);
			for (int j = nums[i]; j < max_n - nums[i]; ++j)
				if (ways[j])
				{
					tmp[j + nums[i]] += ways[j];
					tmp[j - nums[i]] += ways[j];
				}
			swap(ways, tmp);
		}
		return ways[S + offset];
	}
};


// 2D DP
// https://www.youtube.com/watch?v=r6Wz4W1TbuI&t=139s
class Solution2
{
public:
	int findTargetSumWays(vector<int>& nums, int S)
	{
		const int n = nums.size();
		const int sum = accumulate(nums.begin(), nums.end(), 0);
		if (sum < abs(S)) return 0;
		const int offset = sum;
		vector<vector<int>> ways(n + 1, vector<int>(sum + offset + 1, 0));
		ways[0][offset] = 1;
		for (int i = 0; i < n; ++i)
			for (int j = nums[i]; j < 2 * sum + 1 - nums[i]; ++j)
				if (ways[i][j])
				{
					ways[i + 1][j + nums[i]] += ways[i][j];
					ways[i + 1][j - nums[i]] += ways[i][j];
				}
		return ways.back()[S + offset];
	}
};

// slow
class Solution3
{
public:
	int findTargetSumWays(vector<int>& nums, int S)
	{
		if (nums.size() == 0) return 0;

		unordered_map<int, int> cur;
		cur[0] = 1;

		for (int i = 0; i < nums.size(); ++i)
		{
			unordered_map<int, int> tmp;
			for (auto& d : cur)
			{
				tmp[d.first + nums[i]] += cur[d.first];
				tmp[d.first - nums[i]] += cur[d.first];
			}
			cur = tmp;
		}
		return cur.count(S) ? cur[S] : 0;
	}
};

int main()
{
	vector<int> nums{ 1, 1, 1, 1, 1 };
	Solution0 solution0;
	Solution1 solution1;
	Solution2 solution2;
	Solution3 solution3;
	cout << solution0.findTargetSumWays(nums, 3) << endl;
	cout << solution1.findTargetSumWays(nums, 3) << endl;
	cout << solution2.findTargetSumWays(nums, 3) << endl;
	cout << solution3.findTargetSumWays(nums, 3) << endl;
}

```
