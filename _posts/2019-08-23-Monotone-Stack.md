---
layout:     post
title:      "Monotone Stack"
date:       2019-8-23
author:     Tong
catalog: true
tags:
    - Algorithm
---

### Summary

1. 单调递增还是单调递减？

2. 什么时候出栈？代表了什么含义？

3. 是否需要在input最后加上一个元素来强制所有元素出栈？


### 寻找右侧比自己大的元素

给一个数组，返回一个大小相同的数组。返回的数组的第`i`个位置的值应当是，对于原数组中的第`i`个元素，至少往右走多少步，才能遇到一个比自己大的元素（如果之后没有比自己大的元素，或者已经是最后一个元素，则在返回数组的对应位置放上`-1`）。

**Example**
```
input: 5,3,1,2,4

return: -1 3 1 1 -1
```

#### Train of Thought

我们可以维护一个单调递减的stack，stack存的是原数组元素的index。

每当我们遇到一个比栈顶要大的数时，我们便弹出所有比这个新来的数要小的栈内元素，它们index的差值便是要走的步数。

对于每一个元素，出栈代表着它们遇到了右侧第一个比自己大的元素。

#### Solution

```c++
class Solution {
public:
  vector<int> nextExceed(vector<int> & input) {
		vector<int> result(input.size(), -1);
		stack<int> monoStack;
		for (int i = 0; i < input.size(); ++i) {
			while (!monoStack.empty() && input[monoStack.top()] < input[i]) {
				result[monoStack.top()] = i - monoStack.top();
				monoStack.pop();
			}
			monoStack.push(i);
		}
		return result;
	}
};
```



### [84\. Largest Rectangle in Histogram](https://leetcode.com/problems/largest-rectangle-in-histogram/)

Difficulty: **Hard**


Given _n_ non-negative integers representing the histogram's bar height where the width of each bar is 1, find the area of largest rectangle in the histogram.

![](https://assets.leetcode.com/uploads/2018/10/12/histogram.png)  
<small style="display: inline;">Above is a histogram where width of each bar is 1, given height = `[2,1,5,6,2,3]`.</small>

![](https://assets.leetcode.com/uploads/2018/10/12/histogram_area.png)  
<small style="display: inline;">The largest rectangle is shown in the shaded area, which has area = `10` unit.</small>

**Example:**

```
Input: [2,1,5,6,2,3]
Output: 10
```


#### Train of Thought

这里我们要维护一个单调递增栈，因为使得之前更矮的矩形可以和后面的矩形构成更大的面积（以最矮的高度为顶）。

出栈时，代表着遇到了一个比栈顶要矮的矩形。这时，无法以栈顶为顶构成矩形了，所以我们要把那些比新来元素高的矩形高度弹出。

为了保证所有的顶点都能算到，我们在input的最后加了一个`0`。使得所有高度都有机会成为顶部，我们能比较所有可能的面积。

#### Solution

Language: **C++**

```c++
#include <algorithm>
#include <iostream>
#include <vector>
#include <stack>

using namespace std;

class Solution {
public:
	int largestRectangleArea(vector<int>& heights) {
		int ret = 0;
		heights.push_back(0);
		stack<int> index;

		for (int i = 0; i < heights.size(); ++i)
		{
			while (!index.empty() && heights[index.top()] >= heights[i])
			{
				int h = heights[index.top()];
				index.pop();

				int leftIndex = index.empty() ? -1 : index.top();
				ret = max(ret, h * (i - 1 - leftIndex));
			}

			index.push(i);
		}

		return ret;
	}
};

int main()
{
	vector<int> heights{ 2,1,5,6,3 };
	Solution solution;
	cout << solution.largestRectangleArea(heights) << endl;
}

```


### [85\. Maximal Rectangle](https://leetcode.com/problems/maximal-rectangle/)

Difficulty: **Hard**


Given a 2D binary matrix filled with 0's and 1's, find the largest rectangle containing only 1's and return its area.

**Example:**

```
Input:
[
  ["1","0","1","0","0"],
  ["1","0","1","1","1"],
  ["1","1","1","1","1"],
  ["1","0","0","1","0"]
]
Output: 6
```

#### Train of Thought

类似上一题，我们可以一行行累加构建一组组height，如果最新一行（最下面）的某个元素是`0`，那么对应的一列的height就为0。

然后用上面的算法就可以求出最大面积了。

#### Solution

Language: **C++**

```c++
#include <algorithm>
#include <iostream>
#include <vector>
#include <stack>

using namespace std;

class Solution {
public:
	int largestRectangleArea(vector<int>& heights) {
		int ret = 0;
		heights.push_back(0);
		stack<int> index;

		for (int i = 0; i < heights.size(); ++i)
		{
			while (!index.empty() && heights[index.top()] >= heights[i])
			{
				int h = heights[index.top()];
				index.pop();

				int leftIndex = index.empty() ? -1 : index.top();
				ret = max(ret, h * (i - 1 - leftIndex));
			}

			index.push(i);
		}

		return ret;
	}
	int maximalRectangle(vector<vector<char>>& matrix) {
		if (matrix.size() == 0 || matrix[0].size() == 0)
			return 0;
		int maxArea = 0;
		vector<int> heights(matrix[0].size(), 0);

		for (int i = 0; i < matrix.size(); ++i)
		{
			for (int j = 0; j < matrix[0].size(); ++j)
			{
				if (matrix[i][j] == '0') heights[j] = 0;
				else ++heights[j];
			}

			maxArea = max(maxArea, largestRectangleArea(heights));
			heights.pop_back();
		}
		return maxArea;
	}
};

int main()
{
	vector<vector<char>> matrix{
		{1, 0, 1, 0, 0},
		{1, 0, 1, 1, 1},
		{1, 1, 1, 1, 1},
		{1, 0, 0, 1, 0} };
	Solution solution;
	cout << solution.maximalRectangle(matrix) << endl;
}

```



### Traffic Lights

十字路口前，有`n`辆车在等红灯。由于车的高度不尽相同，某些车会因前车的遮挡而无法看到红绿灯。

现已知红绿灯的高度为$$h$$，$$n$$辆车按距离红绿灯由近到远分别标号为$$1...n$$，第$$i$$辆车与红绿灯的距离为$$i$$，高度为$$a_i$$。为简化问题，我们以距红绿灯的距离为x轴，高度为y轴建立平面直角坐标系，则红绿灯可抽象为一点$$(0, h)$$，第$$i$$辆车可抽象为线段$$(i, 0) - (i, a_i)$$。我们称车$$j$$挡住了车$$i$$的红绿灯，当且仅当$$j<i$$，且车$$i$$看红绿灯的视线，即$$(i,a_i)$$与$$(0,h)$$的连线与代表车$$j$$的线段$$(j, 0) - (j, a_j)$$。

现在，我们需要你对每辆车计算谁挡住了它的红绿灯；即对于每一辆车$$i$$，求最大的$$j$$满足“车$$j$$挡住了车$$i$$的红绿灯”。

**Example:**

```
输入描述：
第一行包含两个非负整数n和h；
第二行包含n个非负整数$$a_1, a_2, ..., a_n$$。

输出描述：
输出共有n行，第i行包含对于车i的答案，若没有车挡住车i，则该行输出0。
```

#### Train of Thought

#### Solution

Language: **C++**

```c++
#include <iostream>
#include <vector>

using namespace std;

int n, h;

double getK(double hi, int i)
{
	return (hi - h) / (i + 1);
}

int main()
{
	cin >> n >> h;
	vector<int> nums;
	int tmp;
	for (int i = 0; i < n; ++i)
	{
		cin >> tmp;
		nums.push_back(tmp);
	}

	vector<int> is;
	vector<int> res;
	is.push_back(0);
	res.push_back(0);
	for (int i = 1; i < nums.size(); ++i)
	{
		double ki = getK(nums[i], i);
		while (!is.empty() && ki > getK(nums[is.back()], is.back()))
			is.pop_back();
		if (is.empty())
			res.push_back(0);
		else
			res.push_back(is.back() + 1);
		is.push_back(i);
	}

	for (int i : res)
		cout << i << endl;
}
```
