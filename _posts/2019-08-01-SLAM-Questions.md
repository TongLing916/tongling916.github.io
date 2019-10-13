---
layout:     post
title:      "SLAM Questions"
date:       2019-8-1
author:     Tong
catalog: true
tags:
    - SLAM
---

### Algorithm

1. [请给出PnP的经典解法，任选ePnP，P3P，P1P来简要说明(100字以内)。](https://www.nowcoder.com/profile/968642742/test/25553522/168350#summary)

### Feature

1. [请简单汇总下ORB-BRISK-FREAK特征描述子的改进点，HARRIS角点/FAST的角点提取方式。列举一些表达图像的patch或者是描述子相似度的测度(2种即可)。 ](https://www.nowcoder.com/profile/968642742/test/25553522/168353#summary)

### Filter

1. [（本题为图像处理方向）详述卡尔曼滤波或粒子滤波的原理及其在目标跟踪中的应用 ](（本题为图像处理方向）详述卡尔曼滤波或粒子滤波的原理及其在目标跟踪中的应用 )

2. 编程一个卡尔曼滤波。

### Geometry

1. [（本题为图像处理方向）从变换矩阵和变换效果等方面阐述相似变换、仿射变换、投影变换的区别。 ](https://www.nowcoder.com/profile/968642742/test/25480638/167632#summary)

2. [（本题为图像处理方向）F矩阵，E矩阵，H矩阵有什么关联性。](https://www.nowcoder.com/profile/968642742/test/25480638/167635#summary)


### Optimization

1. [请写出L-M(Levenberg-Marquard)算法的流程并指出其相对于GN优化算法的优缺点。 ](https://www.nowcoder.com/profile/968642742/test/25553522/168352#summary)

2. [（本题为模式识别方向）列举至少三种损失函数，写出数学表达式并简述各自优点 ](https://www.nowcoder.com/profile/968642742/test/25480638/167642#summary)

3. 详细描述梯度下降法过程及原理；什么是梯度消失和梯度爆炸？解决梯度消失和梯度爆炸的方案都有那些？

### SLAM

1. [请写出MSCKF的状态空间，并简单说说MSCKF2.0版本的零空间投影的操作方式，以及其线性空间和其零空间的rank的关系。](https://www.nowcoder.com/profile/968642742/test/25553522/168354#summary)

2. [下图中l表示3D landmark，x表示6D pose，Pose-pose为里程计，pose-landmark的观察为相机投影。请写出Jacobian矩阵和Hessian矩阵的稀疏性结构(矩阵块结构即可)。其次请总结下你知道的landmark的参数化方式(3种即可)。 ](https://www.nowcoder.com/profile/968642742/test/25553522/168351#summary)

3. [（本题为图像处理方向）视觉slam中常用的边缘化处理的原理和方法。 ](https://www.nowcoder.com/profile/968642742/test/25480638/167636#summary)

4. [（本题为图像处理方向）单目视觉slam中尺寸漂移是怎么产生的？ ](https://www.nowcoder.com/profile/968642742/test/25480638/167637#summary)

5. 如何用李代数解决求导问题？

1）用李代数表示姿态，然后根据李代数加法对李代数求导。这个方法要求计算一个雅可比矩阵。
2）对李群左乘或者右乘微小扰动，然后对该扰动求导，称为左扰动或右扰动模型。这种方法不需要求雅可比矩阵。

6. ORB-SLAM2用的哪种模型？为什么不用计算雅可比矩阵？

用了是左乘扰动模型，省略了计算雅可比矩阵。

7. 数据怎么在ORB-SLAM2的三个线程间传递的？（从代码层面解释）
`main`里面启动三个线程（不考虑`Viewer`的话）-`mpTracker`, `mpLocalMapper`, `mpLoopCloser`。

对于每张输入的图片，`main`函数会调用`System::TrackMonocular()`，这个函数会使用`mpTracker->GrabImageMonocular()`函数进行tracking。通过最后一步判断是否为keyframe，来决定我们是否应该插入一个keyframe到`mpLocalMapper`的`std::list<KeyFrame*> mlNewKeyFrames`变量中。

`mpLocalMapper`一直处于`while(1)`循环中，调用`CheckNewKeyFrames()`函数。`LocalMapping::CheckNewKeyFrames()`会检查`std::list<KeyFrame*> mlNewKeyFrames`这个变量是否为空。转换成BoW，删除多余的地图点，创建新的地图点，试图找到更多的地图点，进行Local BA，删除多余的keyframe，以及最后把新的keyframe加入到`mpLoopCloser`的`std::list<KeyFrame*> mlpLoopKeyFrameQueue`变量中。


`mpLoopCloser`一直处于`while(1)`循环中，调用`CheckNewKeyFrames()`函数。`LoopClosing::CheckNewKeyFrames()`会检查`std::list<KeyFrame*> mlpLoopKeyFrameQueue`这个变量是否为空。检查是否是一个loop，计算sim3，进行loop fusion，以及Essential Graph Optimization（位姿图优化）。
