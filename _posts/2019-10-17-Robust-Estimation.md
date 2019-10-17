---
layout:     post
title:      "鲁棒估计"
date:       2019-10-17
author:     Tong
catalog: true
tags:
    - SLAM
---


### RANSAC (Random Sample Consensus)

#### 目标

一个模型与含有野值得数据集$$S$$的鲁棒拟合。（由最小集推测的模型根据在阈值距离内的点数来计分）

#### 算法

1. 随机地从$$S$$中选择$$s$$个数据点组成的一个样本作为模型的一个例示。
2. 确定在模型距离阈值$$t$$内的数据点集$$S_i$$，$$S_i$$称为采样的一致集并定义$$S$$的内点。
3. 如果$$S_i$$的大小（内点的数目）大于某个阈值$$T$$，用$$S_i$$的所有点重估计模型并结束。
4. 如果$$S_i$$的大小小于$$T$$，选择一个新的子集并重复上面的过程。
5. 经过N次实验选择最大一致集$$S_i$$，并用$$S_i$$的所有点重估计模型。

#### 问题

如果选取算法中的三个阈值$$t$$, $$N$$, $$T$$?

#### 距离阈值$$t$$

我们希望选择的距离阈值$$t$$使点为内点的概率是$$\alpha$$。该计算需要知道内点到模型的距离的概率分布。但是如果假定测量误差为零均值和标准方差$$\sigma$$的高斯分布，那么$$t$$的值可以算出。计算时还要考虑模型的余维度。通常$$\alpha$$取$$0.95$$。

#### 采样次数$$N$$

只要采样次数$$N$$足够大，以保证由$$s$$个点组成的随机样本中至少有一次没有野值的概率为$$p$$，通常$$p$$取为$$0.99$$。假设$$w$$是任意选择的数据点为内点的概率，那么$$\epsilon = 1 - w$$是其为野值的概率。则$$
\left(1-w^{s}\right)^{N}=1-p
$$。

$$
N=\log (1-p) / \log \left(1-(1-\epsilon)^{s}\right)
$$

根据上述公式，我们可以采用自适应的算法来确定野值概率$$\epsilon$$和采样次数$$N$$。
1. 初始化：$$\epsilon = 1, N = \inf, sample_count = 0$$
2. 当 N > sample_count 时重复
    - 选取一个样本并计算内点数
    - 令$$\epsilon = 1 - (内点数) / (总点数)$$
    - 取$$p = 0.99$$，并计算$$N$$
    - sample_count += 1
3. 终止


#### 一致集大小$$T$$

在给定野值的假定比例后，如果一致集大小接近期望属于该数据集的内点数时迭代就停止，即对$$n$$个数据，$$
T=(1-\epsilon) n
$$。

### 最小中值平方估计(Least Median of Squares, LMS)

#### 目标

根据数据中所有点的距离中值，选择具有最小中值的模型。

#### 优点

不需要阈值或误差方差的先验知识。

#### 缺点

如果多余一半的数据是野值，那么它要失败，因为距离的中值可能是一个野值。

解决办法：用野值的比例来确定它所选的距离。例如有50%野值时可取低于中值的阈值。


### [编程题：线性回归](https://www.nowcoder.com/questionTerminal/9d4d2ab1cca947ec88912fc7761e537b)

[参考资料](https://blog.csdn.net/LaplaceSmoothing/article/details/94581854)

拟合二维平面中的带噪音直线，
其中有不超过10%的样本点远离了直线，另外90%的样本点可能有高斯噪声的偏移
要求输出为
ax+by+c＝0的形式
其中a > 0 且 a^2 + b^2 = 1

输入描述:
第一个数n表示有多少个样本点  之后n*2个数 每次是每个点的x 和y


输出描述:
输出a,b,c三个数，至多可以到6位有效数字
示例1
输入
5
3 4
6 8
9 12
15 20
10 -10
输出
-0.800000 0.600000 0.000000
说明
本题共有10个测试点，每个点会根据选手输出的参数计算非噪音数据点的拟合误差E，并根据E来对每个数据点进行评分0-10分
输入数据的范围在0-10000

#### 实现

```c++
#include <cmath>		
#include <iostream>
#include <limits>
#include <random>
#include <vector>
using namespace std;

struct Point
{
	double x, y;
};

vector<double> ransac(vector<Point>& points, double outlier_prob = 0.1, double p = 0.99, double threshold = 10.0)
{
	default_random_engine generator;
	int n_sample = points.size();
	uniform_int_distribution<int> sampler(0, n_sample - 1);

	int s = 2;   // 拟合模型需要的最小数据量

	// 计算理论最大迭代次数
	double inlier_prob = 1 - outlier_prob;
	double sample_fail = 1 - inlier_prob * inlier_prob;
	int N = log(1 - p) / log(sample_fail);	// p: 取样N次，至少一次没有野值的概率

	double a_res = 0, b_res = 0, c_res = 0;
	double min_error = numeric_limits<double>::max();
	while (N--)
	{
		// 随机采样 s 个样本
		int idx1 = 0, idx2 = 0;
		while (idx1 == idx2)
		{
			idx1 = sampler(generator);
			idx2 = sampler(generator);
		}
		Point p1 = points[idx1], p2 = points[idx2];

		// 拟合模型：a*x1+b*y1+c=0 和 a*x2+b*y2+c=0
		// 两式相减：a*(x1-x2) = b*(y2-y1)
		//    解得：a=z*(y2-y1), b=z*(x1-x2)
		// 归一化时 z 会被约去，令 z=1，得 a=y2-y1, b=x1-x2
		// 把上述a,b代入 a*x1+b*y1+c=0 解得 c=x2*y1-y2*x1
		double a = p2.y - p1.y;
		double b = p1.x - p2.x;
		double c = (p2.x * p1.y) - (p2.y * p1.x);

		// 归一化到 a^2 + b^2 = 1
		double coef = sqrt(a * a + b * b);
		a /= coef;
		b /= coef;
		c /= coef;

		// 测试数据，计算可能的局内点
		double error = 0.0;
		int n_inlier = 0;
		for (int i = 0; i < n_sample; ++i)
		{
			double err_i = fabs(a * points[i].x + b * points[i].y + c);
			if (err_i < threshold)
			{   // 若低于阈值，则为内点
				++n_inlier;
				error += err_i;
			}
		}

		// 若有足够多的点被归类为局内点
		if (static_cast<double>(n_inlier) / static_cast<double>(n_sample) > 0.7)
			if (error < min_error)
			{   // 若新模型更好
				min_error = error;
				a_res = a;
				b_res = b;
				c_res = c;
			}
	}
	return { a_res, b_res, c_res };
}

int main()
{
	int N = 0;
	cin >> N;
	vector<Point> points(N, { 0,0 });
	for (int i = 0; i < N; ++i)
		cin >> points[i].x >> points[i].y;
	vector<double> res = ransac(points, 0.1, 0.99, 10);
	cout << res[0] << " " << res[1] << " " << res[2] << endl;
}
```
