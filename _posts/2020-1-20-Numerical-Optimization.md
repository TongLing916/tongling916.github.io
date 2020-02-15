---
layout:     post
title:      "Numerical Optimization"
date:       2020-1-20
author:     Tong
catalog: true
tags:
    - SLAM
---

> https://github.com/RainerKuemmerle/g2o

### Theory

#### 梯度下降法

#### 牛顿法

#### 高斯牛顿法

##### 简单例子

```c++
#include <iostream>
#include <chrono>
#include <cmath>
#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

// y = exp(ar * x * x + br * x + cr)
int main(int argc, char **argv)
{
    double ar = 1.0, br = 2.0, cr = 1.0;
    double ae = 2.0, be = -1.0, ce = 5.0;
    int N = 100;
    double w_sigma = 1.0;
    double inv_sigma = 1.0 / w_sigma;
    cv::RNG rng;

    vector<double> x_data, y_data;
    for (int i = 0; i < N; ++i)
    {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma));
    }

    int iterations = 100;
    double cost = 0, last_cost = 0;

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for (int iter = 0; iter < iterations; ++iter)
    {
        Matrix3d H = Matrix3d::Zero();
        Vector3d b = Vector3d::Zero();
        cost = 0;

        for (int i = 0; i < N; ++i)
        {
            double xi = x_data[i], yi = y_data[i];
            double error = yi - exp(ae * xi * xi + be * xi + ce);
            Vector3d J;
            J[0] = -xi * xi * exp(ae * xi * xi + be * xi + ce);
            J[1] = -xi * exp(ae * xi * xi + be * xi + ce);
            J[2] = -exp(ae * xi * xi + be * xi + ce);

            H += inv_sigma * inv_sigma * J * J.transpose();
            b += -inv_sigma * inv_sigma * error * J;

            cost += error * error;
        }

        Vector3d dx = H.ldlt().solve(b);
        if (isnan(dx[0]))
        {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= last_cost)
        {
            cout << "cost: " << cost << " >= last_cost: " << last_cost << ", break." << endl;
            break;
        }

        ae += dx[0];
        be += dx[1];
        ce += dx[2];

        last_cost = cost;
        cout << "total cost: " << cost << ", update: " << dx.transpose() << endl
             << "estimated params: " << ae << ", " << be << ", " << ce << endl
             << endl;
    }

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve time cost = " << time_used.count() << " seconds." << endl;

    cout << "estimated abc = " << ae << ", " << be << ", " << ce << endl;
    cout << "real abc = " << ar << ", " << br << ", " << cr << endl;
}
```

#### Levenberg-Marquard Algorithm

##### 缺点

当一次更新被拒绝后，修改后的信息矩阵需要被重新分解，而分解的过程是整个算法中最耗时的一个部分。

#### Dogleg

##### 思想

分别利用高斯牛顿法和梯度下降法计算更新量，然后将其有效地结合起来。如果更新被拒绝，更新的方向仍然是有效的，并且它们可以一种不同的方式重新结合起来，知道目标函数的值下降为止。因此，每次状态估计量的更新只涉及一个而非多个矩阵的分解。

##### 缺点

高斯牛顿法和Dogleg算法，都要测观测量雅可比矩阵是满秩的，以保证可逆性。当遇到欠约束的系统（没有足够的观测值），或者数值上病态的系统，就可以使用LM算法，尽管收敛速度可能受到影响。


### Implementation
