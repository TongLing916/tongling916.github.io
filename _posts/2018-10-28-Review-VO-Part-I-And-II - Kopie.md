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

$$\quad$$ Visual Odometry (VO) 是一个估测某个运动物体的过程，其中只会使用一个或几个摄像机。 VO通过检查图像上的不同，来持续地估计物体的位置。VO要想顺利工作，必须保证环境中有充足的光照(illumination)，场景静态并且有足够的纹理（texture），这样才能让运动顺利地被提取出来。另外，必须要拍摄连续的帧（frame)来保证它们场景能有足够的重合。

$$\quad$$ 不像车轮里程计（wheel odometry）一样，VO不会被轮胎打滑或其他不好的路面情况所影响。

$$\quad$$ 本篇主要介绍VO历史和基础，在介绍完camera modeling和calibration后，会介绍单目和双目的the main motion-estimation pipelines，然后指出两者的优劣。

### 1. History of Visual Odometry

$$\quad$$ 在

### 1.1 Stereo VO

### 1.2 Monocular VO

### 1.3 Reducing the Drift

### 1.4 V-SLAM 

### 1.5 VO Versus V-SLAM

### 2. Camera Modeling and Calibration

### 2.1 Perspective Camera Model

### 2.2 Omnidirectional Camera Model

### 2.3 Spherical Model

### 2.4 Camera Calibration

### 2.5 2D-to-2D: Motion from Image Feature Correspondences

### 2.6 3D-to-3D: Motion from 3-D Structure Correspondences

### 2.7 3D-to-2D: Motion from 3-D Structure and Image Feature Correspondences

### 2.8 Triangulation and Keyframe Selection

### 2.9 Discussion

### 3. Conclusions


<br>
<br>
<br>

## [Part II: Matching, Robustness, and Applications][paper-part-2]

$$\quad$$ 本篇主要介绍feature matching, robustnessm和applications。它会回顾在VO中经常使用的point-feature detectors和不同的outlier-rejection schemes。重点还会讲random sample consensus (RANSAC)，以及一些用来提升它速度的独特的tricks。其他会提到的有error modeling, location recognition (or loop-closure detection)以及bundle adjustment.


[paper-part-1]: https://www.ifi.uzh.ch/dam/jcr:5759a719-55db-4930-8051-4cc534f812b1/VO_Part_I_Scaramuzza.pdf
[paper-part-2]: http://rpg.ifi.uzh.ch/docs/VO_Part_II_Scaramuzza.pdf