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

> Keyframe selection is a very important step in VO and should always be done before updating the motion.

$$\quad$$ 很多双目VO都是对每一对双目图像采用三角测量法测量3D点，再把这个作为一个3D-3D point registration 
(the process of finding a spatial transformation that aligns two point sets)的问题算出相对运动。[Nister等][paper-nister]提出了一种不同的方法。首先，不像以前的工作，
他们并没有追踪帧之间的features，而是独立地检测所有帧中的features（[Harris corner][website-harris-corner]），并且只允许features之间配对。
这种方法有效地避免了cross-correlation-based tracking间的feature drift. 第二点，他们没用对待3D-to-3D point regestration问题的方法来计算相对运动，而是把这看作是一个
3D-to-2D camera-pose estimation问题。最后，他们还在motion estimation的一步中结合了RANSAC outlier rejection。

$$\quad$$ [Comport等][paper-comport]还提出了一种motion estimation的方案，他们用了quadrifocal tensor，使得可以利用2D-to-2D的图片对应来计算运动，
而不再需要用三角测量的方法计算任何3D的点。这种直接使用原生2D点的方法可以得到更加精确的运动。


#### 1.2 Monocular VO

$$\quad$$ 与双目方法不同，单目VO要求相对运动和3D结构从2D bearing数据计算获得。由于absolute scale不知道，所以初始两个相机的位置通常设置为1。当一张新的图像到来时，
相对尺寸和相机对于初始两帧的位置便能确定，要么用3D结构的知识或者用trifocal tensor。

$$\quad$$ 单目方法可以分为以下几种：feature-based methods, appearance-based methods以及hybrid methods。Feature-based method是基于追踪每一帧中显著并且重复的feature；
Appearance-based methods是利用一张图片或者图片中一个区域的所有像素的强度信息；Hyrbid methods将这两个方法联合了起来。

$$\quad$$ 在feature-based methods中，很多人采用了[five-point minimal sovler][website-ransac]来计算RANSAC中的运动假设。Five-point RANSAC可以用来出去outliers。
Appearance-based methods有一个很大的问题是它们对于极易被遮挡影响。

$$\quad$$ 所有上面提到的方法原本都是给无限制的运动（6 DOF）设计的。然而，也有一些VO的方法是专门给那些有限制的物体的设计的。这个好处是降低了计算时间以及提高了运动准确性.


#### 1.3 Reducing the Drift

$$\quad$$ 由于VO是累加地计算相机的路径(一个接一个的位置),所以每次帧到帧的误差会随着时间而累加.这就会造成估计的轨迹对于实际路径的一个漂移. 对一些应用来说,使这个漂移尽可能的小至关重要,我们可以
对最后的m个相机位置采取局部优化的方式.这种方法称为_sliding window bundle adjustment_或者_windowed bundle adjustment_.

#### 1.4 V-SLAM 

> VO is only concerned with the local consistency of the trajectory, whereas SLAM with the global consistency.

$$\quad$$ V-SLAM中有两种占主导的方法:
	- Filtering methods fuse the information from all the images with a probability distribution
	- Nonfiltering methods (also called _keyframe methods_) retain the optimization of global bundle adjustment to selected keyframes.

$$\quad$$ 两种方法的优劣可以参考这篇[论文][paper-why-filter].

#### 1.5 VO Versus V-SLAM

> In the motion estimation step, the camera motion between the current and the previous image is computed.

$$\quad$$ SLAM的目标通常是得到一个全局的,一致的机器人路径估计.这意味这需要保存整个地图上的路径轨迹(即使它本身不需要这个地图),因为它需要识别出它以前经过的地点(这也称作_loop closure_.
当一个回环被检测到时,这个信息可以用来减少地图和相机路径的漂移.Understanding wehn a loop closure occurs和effciently integrating this new constraint 
into current map时SLAM中两个主要的问题). 

$$\quad$$ 相反,VO的目的时持续地回复路径,一个位置接一个位置,以及还可能对最后n个位置进行优化(_window bundle adjustment_).这个sliding window optimization可以看作时SLAM中的
建立局部地图.然而,这里的思想是不同的:在VO中,我们只考虑轨迹的局部一致性,局部地图是用来获得对于局部轨迹更精确的一个估计;然而SLAM中考虑更多的是全局地图的一致性.

$$\quad$$ VO可以被用来当作SLAM中的一部分,用来恢复相机持续的运动.然而,要想完成一个SLAM,我们还得加回环检测以及可能全局优化以此来保证地图度量的一致性.

$$\quad$$ 一个V-SLAM方法可能会更精确,因为它对路径实施了更多的限制,但是可能不会更加鲁棒(比如说,回环检测中outlier能严重影响地图的一致性).另外,它更复杂而且计算量更大.

### 2. Formulation of the VO Problem

> Feature matching: Detecting features independently in all the images and then matching them based on some similarity metrics. <br>
> Featuer tracking: Finding features in one image and then tracking them in the next images using a local search technique, such as correlation.

$$\quad$$ VO中主要任务是从两个照片中计算relative transformation $$T_k$$,然后利用这算出来的变换矩阵来算出相机经过的整个轨迹.在算出一个位置后,可以使用一个迭代的优化方法来获取一个更加准确的
局部轨迹的估计.这个迭代的方法是减少最近10张图中重构的3D点的重投影误差的平方和(也称为 _windowed-bundle adjustment_ ).这些3D点是通过对图像点进行三角测量得到的.

$$\quad$$ 主要有两种方法来计算相对运动$$T_k$$: <br>
	- appearance-based (or global) methods,利用两幅输入图中所有像素强度信息  <br>
	- feature-based methods,只利用从图像中提取出的突出的,重复的特征.

$$\quad$$ Global methods相比feature-based不够准确,而且运算量太大.Feature-based methods需要能够稳定地匹配(或追踪)帧之间的特征,与global methods相比更精确,更快.

$$\quad$$ 一个VO工作的示意图如下图所示.对于每张新图片$$I_k$$,首先两步是检测和匹配其中与之前帧中对应的2D特征.从不同帧中3D特征通过重投影得到的2D特征称为_image correspondences_.第三步是
计算两帧之间的相对运动$$T_k$$。根据不同的对应类型，一共有三种不同的方法解决这个问题。最后，一个迭代的优化可以对最后m帧实施。

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/post-vo.PNG)

 

### 3. Camera Modeling and Calibration

#### 3.1 Perspective Camera Model

#### 3.2 Omnidirectional Camera Model

#### 3.3 Spherical Model

#### 3.4 Camera Calibration

### 4. Motion Estimation

#### 4.1 2D-to-2D: Motion from Image Feature Correspondences

#### 4.2 3D-to-3D: Motion from 3-D Structure Correspondences

#### 4.3 3D-to-2D: Motion from 3-D Structure and Image Feature Correspondences

#### 4.4 Triangulation and Keyframe Selection

#### 4.5 Discussion

### 5. Conclusions


<br>
<br>
<br>

## [Part II: Matching, Robustness, and Applications][paper-part-2]

$$\quad$$ 本篇主要介绍feature matching, robustness和applications。它会回顾在VO中经常使用的point-feature detectors和不同的outlier-rejection schemes。
重点还会讲random sample consensus ([RANSAC][website-ransac])，以及一些用来提升它速度的独特的tricks。其他会提到的有error modeling, location recognition (or loop-closure detection)以及bundle adjustment.


[paper-part-1]: https://www.ifi.uzh.ch/dam/jcr:5759a719-55db-4930-8051-4cc534f812b1/VO_Part_I_Scaramuzza.pdf
[paper-part-2]: http://rpg.ifi.uzh.ch/docs/VO_Part_II_Scaramuzza.pdf
[paper-nister]: https://www.computer.org/csdl/proceedings/cvpr/2004/2158/01/01315094.pdf
[website-harris-corner]: https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_feature2d/py_features_harris/py_features_harris.html
[paper-comport]: http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.331.9823&rep=rep1&type=pdf
[website-ransac]: http://lingtong.de/2018/10/29/RANSAC/
[paper-why-filter]: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=5509636