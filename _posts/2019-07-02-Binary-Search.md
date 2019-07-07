---
layout:     post
title:      "Sort"
date:       2019-7-1
author:     Tong
catalog: true
tags:
    - Sort
---

### Summary

1.


### [29. Divide Two Integers](https://leetcode.com/problems/divide-two-integers/)

#### Question

Given two integers dividend and divisor, divide two integers without using multiplication, division and mod operator.

Return the quotient after dividing dividend by divisor.

The integer division should truncate toward zero.

__Example 1:__
Input: dividend = 10, divisor = 3
Output: 3

__Example 2:__
Input: dividend = 7, divisor = -3
Output: -2

__Note:__
Both dividend and divisor will be 32-bit signed integers.
The divisor will never be 0.
Assume we are dealing with an environment which could only store integers within the 32-bit signed integer range: $$[−2^31,  2^31 − 1]$$. For the purpose of this problem, assume that your function returns $$2^31 − 1$$ when the division result overflows.

#### Train of Thought

Keep always the extreme case in mind: -2147483648 / -1. Only in this case, overflow can happen.

To get the result, we need to count how many times divisor can be substracted from the dividend. To avoid overflow, we should convert Integer to Long Integer.

Then, due to the fact that we cannot use x, /, %, we should consider another operator to speed up the computation. Here, we can use `<<` to double the value. 

#### Solution
```cpp
#include <iostream>

using std::cout;
using std::endl;

class Solution {
public:
	int divide(int dividend, int divisor)
	{
		if (divisor == INT_MIN)
		{
			if (dividend == INT_MIN)
				return 1;
			else
				return 0;
		}

		if (dividend == INT_MIN && divisor == -1)
			return INT_MAX;

		bool is_positive = (dividend > 0 && divisor > 0) || (dividend < 0 && divisor < 0) ? true : false;

		long int dividend_l = dividend;
		long int divisor_l = divisor;

		dividend_l = abs(dividend_l);
		divisor_l = abs(divisor_l);

		long int res = 0;

		while (dividend_l >= divisor_l)
		{
			long int tmp_res = 1;
			long int tmp_divisor = divisor_l;
			while ((tmp_divisor << 1) < dividend_l)
			{
				tmp_divisor <<= 1;
				tmp_res <<= 1;
			}

			dividend_l -= tmp_divisor;
			res += tmp_res;
		}

		if (is_positive)
			return res;
		else
			return -res;
	}
};

int main()
{
	Solution solution;
	cout << solution.divide(INT_MIN, -1) << endl;
	cout << solution.divide(INT_MIN, 1) << endl;
}

```
