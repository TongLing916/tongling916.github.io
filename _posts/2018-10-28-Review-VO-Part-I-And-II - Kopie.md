---
layout:     post
title:      "Review - Visual Odometry: Part I and II"
date:       2018-10-28
author:     Tong
catalog: true
tags:
    - SLAM
---

## [Part I: The First 30 Years and Fundamentals][paper-part-1]

$$\quad$$ Visual Odometry (VO) 是一个估测某个运动物体的过程，其中只会使用一个或几个摄像机。 VO通过检查图像上的不同，来持续地估计物体的位置。VO要想顺利工作，
必须保证环境中有充足的光照(illumination)，场景静态并且有足够的纹理（texture），这样才能让运动顺利地被提取出来。另外，必须要拍摄连续的帧（frame)来保证它们场景能有足够的重合。

$$\quad$$ 不像车轮里程计（wheel odometry）一样，VO不会被轮胎打滑或其他不好的路面情况所影响。

$$\quad$$ 本篇主要介绍VO历史和基础，在介绍完camera modeling和calibration后，会介绍单目和双目的the main motion-estimation pipelines，然后指出两者的优劣。

### 1. History of Visual Odometry

$$\quad$$ 在computer vision领域，通过一系列相机照片（校准过或没校准）来恢复相机的相对位置以及3D结构的问题，称为_structure from motion_ （SFM）。
VO是SFM中一种特殊的情况。SFM更加的宽泛，它解决的问题是通过有序或无序的图片集，来对结构以及相机位置进行3D重建。最后得到的结构以及位置还会通过offline优化（例如bundle adjustment）来改善。
与之相反，VO专注于实时并且连续估计相机的3D运动。Bundle adjustment可以用来改善轨迹的局部估计。

$$\quad$$ 在单目相机情况下，只有bearing （the orientation of a robot）信息能被获取到。缺点是运动只能回复到一个相对的状态，还需要确定一个scale factor。这个absolute scale可以
通过不同的手段获得，比如直接测量场景中某个物体的实际大小，运动限制（motion constraints） 或者是其他的传感器，例如IMU（Inertial Measurement Unit），气压以及距离传感器（range sensors）。
之所以有单目相机的方法是因为，双目VO在某些情况会退化成弹幕情况，例如当场景中的距离比双目的baseline（the distance betweeen the two cameras）的距离大得多。在这种情况下，双目视觉就会变得
无效，我们则必须使用单目方法。


#### 1.1 Stereo VO

#### 1.2 Monocular VO

#### 1.3 Reducing the Drift

#### 1.4 V-SLAM 

#### 1.5 VO Versus V-SLAM

### 2. Camera Modeling and Calibration

#### 2.1 Perspective Camera Model

#### 2.2 Omnidirectional Camera Model

#### 2.3 Spherical Model

#### 2.4 Camera Calibration

### 3. Motion Estimation

#### 3.1 2D-to-2D: Motion from Image Feature Correspondences

#### 3.2 3D-to-3D: Motion from 3-D Structure Correspondences

#### 3.3 3D-to-2D: Motion from 3-D Structure and Image Feature Correspondences

#### 3.4 Triangulation and Keyframe Selection

#### 3.5 Discussion

### 4. Conclusions


<br>
<br>
<br>

## [Part II: Matching, Robustness, and Applications][paper-part-2]

$$\quad$$ 本篇主要介绍feature matching, robustness和applications。它会回顾在VO中经常使用的point-feature detectors和不同的outlier-rejection schemes。重点还会讲random sample consensus (RANSAC)，以及一些用来提升它速度的独特的tricks。其他会提到的有error modeling, location recognition (or loop-closure detection)以及bundle adjustment.


[paper-part-1]: https://www.ifi.uzh.ch/dam/jcr:5759a719-55db-4930-8051-4cc534f812b1/VO_Part_I_Scaramuzza.pdf
[paper-part-2]: http://rpg.ifi.uzh.ch/docs/VO_Part_II_Scaramuzza.pdf