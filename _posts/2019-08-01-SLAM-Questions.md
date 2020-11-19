---
layout:     post
title:      "SLAM Questions"
date:       2019-8-1
author:     Tong
catalog: true
tags:
    - SLAM
---

> [视觉SLAM面试题汇总 - 第一部分](https://www.bilibili.com/read/cv7390089?share_source=weixin&share_medium=iphone&bbid=Z945F09203487802419BBC0EF65E97980843&ts=1605518957)

> [视觉SLAM面试题汇总 - 第二部分](https://www.bilibili.com/read/cv7452301?share_source=weixin&share_medium=iphone&bbid=Z945F09203487802419BBC0EF65E97980843&ts=1605518972)

1. SIFT和SUFT的区别。
2. 相似变换、仿射变换、射影变换的区别。
3. Homography、Essential和Fundamental Matrix的区别。
4. 视差与深度的关系。
5. 描述PnP算法。
6. 闭环检测常用方法。
7. 给一个二值图，求最大连通域。
8. 推导一下卡尔曼滤波、描述下粒子滤波。
9.  如何求解$Ax=b$的问题?
10. 什么是极线约束?
11. 单目视觉SLAM中尺寸漂移是怎么产生的?
12. 解释SLAM中的绑架问题。
13. 描述特征点法和直接法的优缺点。
14. EKF和BA的区别。
15. 边缘检测算子有哪些？
16. 简单实现`cv::Mat`
17. 10个相机同时看到100个路标点，问BA优化的雅克比矩阵多少维？
18. 介绍经典的视觉SLAM框架。
19. 介绍下熟悉的非线性优化库。
20. 室内SLAM与自动驾驶SLAM有什么区别？
21. 什么是紧耦合、松耦合？优缺点。
22. 地图点的构建方法有哪些？
23. 如果对于一个3D点，我们在连续帧之间形成了2D特征点之间的匹配，但是这个匹配中可能存在错误的匹配。请问你如何去构建3D点？
24. RANSAC在选择最佳模型的时候用的判断准则是什么?
25. 除了RANSAC之外，还有什么鲁棒估计的方法？
26. 3D地图点是怎么存储的？表达方式？
27. 给你$m$相机$n$个点的bundle adjustment。当我们在仿真的时候，在迭代的时候，相机的位姿会很快的接近真值。而地图点却不能很快的收敛这是为什么呢？
28. LM算法里面那个$\lambda$是如何变化的呢？
29. 说一下3D空间的位姿如何去表达?
30. 李群和李代数的关系。
31. 求导$\frac{\partial R_{1}R_{2}}{R_{1}}$ 
32. `cv::Mat`是如何访问元素的？先访问行还是先访问列？
33. 写出单目相机的投影模型，畸变模型。
34. 安装2D lidar的平台匀速旋转的时候，去激光数据畸变，写代码
35. 给两组已经匹配好的3D点，计算相对位姿变换，写代码
36. ORB-SLAM初始化的时候为什么要同时计算$H$矩阵和$F$矩阵？
37. 介绍Dog-Leg算法。
38. VINS-Mono里面什么是边缘化？First Estimate Jacobian？一致性？可观性？
39. 介绍VINS-Mono的优缺点
40. 推到VINS-Mono里面的预积分公式。
41. 给定一些有噪声的GPS信号的时候如何去精准的定位？
42. 如何标定IMU与相机之间的外参数？
43. 给你xx误差的GPS，给你xx误差的惯导你怎么得到一个cm级别的地图?
44. 计算$H$矩阵和$F$矩阵的时候有什么技巧呢？
45. 给一组点云，从中提取平面。
46. 给一张图片，知道相机与地面之间的相对关系，计算出图的俯视图。
47. 双线性插值如何去做，写公式。
48. RGB-D的SLAM和RGB的SLAM有什么区别？
49. 什么是ORB特征? ORB特征的旋转不变性是如何做的? BRIEF算子是怎么提取的?
50. ORB-SLAM中的特征是如何提取的？如何均匀化的？
51. [请写出MSCKF的状态空间，并简单说说MSCKF2.0版本的零空间投影的操作方式，以及其线性空间和其零空间的rank的关系。](https://www.nowcoder.com/profile/968642742/test/25553522/168354#summary)
52. [下图中l表示3D landmark，x表示6D pose，Pose-pose为里程计，pose-landmark的观察为相机投影。请写出Jacobian矩阵和Hessian矩阵的稀疏性结构(矩阵块结构即可)。其次请总结下你知道的landmark的参数化方式(3种即可)。 ](https://www.nowcoder.com/profile/968642742/test/25553522/168351#summary)
53. [（本题为图像处理方向）视觉slam中常用的边缘化处理的原理和方法。 ](https://www.nowcoder.com/profile/968642742/test/25480638/167636#summary)
54. [（本题为图像处理方向）单目视觉slam中尺寸漂移是怎么产生的？ ](https://www.nowcoder.com/profile/968642742/test/25480638/167637#summary)
55. 如何用李代数解决求导问题？
    - 用李代数表示姿态，然后根据李代数加法对李代数求导。这个方法要求计算一个雅可比矩阵。
    - 对李群左乘或者右乘微小扰动，然后对该扰动求导，称为左扰动或右扰动模型。这种方法不需要求雅可比矩阵。
56. ORB-SLAM2用的哪种模型？为什么不用计算雅可比矩阵？
    - 用了是左乘扰动模型，省略了计算雅可比矩阵。
57. 数据怎么在ORB-SLAM2的三个线程间传递的？（从代码层面解释）
    - `main`里面启动三个线程（不考虑`Viewer`的话）-`mpTracker`, `mpLocalMapper`, `mpLoopCloser`。
    - 对于每张输入的图片，`main`函数会调用`System::TrackMonocular()`，这个函数会使用`mpTracker->GrabImageMonocular()`函数进行tracking。通过最后一步判断是否为keyframe，来决定我们是否应该插入一个keyframe到`mpLocalMapper`的`std::list<KeyFrame*> mlNewKeyFrames`变量中。
    - `mpLocalMapper`一直处于`while(1)`循环中，调用`CheckNewKeyFrames()`函数。`LocalMapping::CheckNewKeyFrames()`会检查`std::list<KeyFrame*> mlNewKeyFrames`这个变量是否为空。转换成BoW，删除多余的地图点，创建新的地图点，试图找到更多的地图点，进行Local BA，删除多余的keyframe，以及最后把新的keyframe加入到`mpLoopCloser`的`std::list<KeyFrame*> mlpLoopKeyFrameQueue`变量中。
    - `mpLoopCloser`一直处于`while(1)`循环中，调用`CheckNewKeyFrames()`函数。`LoopClosing::CheckNewKeyFrames()`会检查`std::list<KeyFrame*> mlpLoopKeyFrameQueue`这个变量是否为空。检查是否是一个loop，计算sim3，进行loop fusion，以及Essential Graph Optimization（位姿图优化）。