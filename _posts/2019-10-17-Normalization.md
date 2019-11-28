---
layout:     post
title:      "Normalization"
date:       2019-10-17
author:     Tong
catalog: true
tags:
    - SLAM
---

### 归一化 (Normalization)

#### 原理

一个只包括平移和缩放的相似变换$$T$$，使得新的点集的形心位于原点，并且它们到原点的平均距离是$$\sqrt{2}$$

#### 原因

在SLAM中，当使用DLT计算单应矩阵$$H$$或者基本矩阵$$F$$时，我们通常会将点先归一化，计算完后，再转换回来。

数据归一化不仅提高了结果的精度，还提供了第二个好处，即对初始化数据归一化的算法将对任何尺度缩放和坐标原点的选择不变。因而使DLT算法关于相似变换不变。

#### 实现

```c++
// ORB-SLAM2: https://github.com/raulmur/ORB_SLAM2
// To ensure that the centroid of the points is (0,0) and their average distance from the origin is square root 2.
void Initializer::Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T)
{
    float meanX = 0;
    float meanY = 0;
    const int N = vKeys.size();

    vNormalizedPoints.resize(N);

    for(int i=0; i<N; i++)
    {
        meanX += vKeys[i].pt.x;
        meanY += vKeys[i].pt.y;
    }

    meanX = meanX/N;
    meanY = meanY/N;

    float meanDevX = 0;
    float meanDevY = 0;

    for(int i=0; i<N; i++)
    {
        vNormalizedPoints[i].x = vKeys[i].pt.x - meanX;
        vNormalizedPoints[i].y = vKeys[i].pt.y - meanY;

        meanDevX += fabs(vNormalizedPoints[i].x);
        meanDevY += fabs(vNormalizedPoints[i].y);
    }

    meanDevX = meanDevX/N;
    meanDevY = meanDevY/N;

    float sX = 1.0/meanDevX;
    float sY = 1.0/meanDevY;

    for(int i=0; i<N; i++)
    {
        vNormalizedPoints[i].x = vNormalizedPoints[i].x * sX;
        vNormalizedPoints[i].y = vNormalizedPoints[i].y * sY;
    }

	// x_normalized = T * x
    T = cv::Mat::eye(3,3,CV_32F);
    T.at<float>(0,0) = sX;
    T.at<float>(1,1) = sY;
    T.at<float>(0,2) = -meanX*sX;
    T.at<float>(1,2) = -meanY*sY;
}
```
