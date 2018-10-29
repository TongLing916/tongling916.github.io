---
layout:     post
title:      "Review - Visual Odometry: Part I and II"
date:       2018-10-29
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
之所以有单目相机的方法是因为，双目VO在某些情况会退化成弹幕情况，例如当场景中的距离比双目的baseline（the distance between the two cameras）的距离大得多。在这种情况下，双目视觉就会变得
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

$$\quad$$ 我们可以用透视投影相机（perspective camera model）以及全景相机（omnidirectional camera）来实现VO。

#### 3.1 Perspective Camera Model

$$\quad$$ 透视投影相机用的最多的是针孔投影模型（pinhole projection）:光束从物体通过透镜中心（the center of the lens, projection center）相交，形成图像。假设$$X = [x,y,z]^\intercal$$
是camera reference frame中的场景点，$$p = [u,v]^\intercal$$是它在成像平面的投影，单位是pixel。3D世界到2D图像的透视投影等式可以写为<br>
$$
\lambda \begin{bmatrix} u \\ v \\ 1\end{bmatrix} = KX = \begin{bmatrix} \alpha_u & 0& u_0 \\ 0 & \alpha_v & v_0 \\ 0 & 0 &1\end{bmatrix} \begin{bmatrix} x \\ y \\ z\end{bmatrix}
$$
<br>$$\quad$$ 其中$$\lambda$$是depth factor,$$\alpha_u$$和$$\alpha_v$$是focal length，$$u_0, v_0$$是image coordinates of the projection center。
这些参数称作_intrinsic parameters_。

#### 3.2 Omnidirectional Camera Model

$$\quad$$ 全景相机是具有广阔视角的相机（有时超过180°），可以使用鱼眼透镜或者将传统相机和镜子联合起来。

#### 3.3 Spherical Model

$$\quad$$ 相机必须具有一个单独的投影中心（a single projection center, also called _single effective viewpoint_）。球体模型不仅能用于全景相机也能用于透视相机。

#### 3.4 Camera Calibration

$$\quad$$ 标定的目的是准确地测量相机的intrinsic和extrinsic parameters。

### 4. Motion Estimation

$$\quad$$  这部分主要讲如何根据两个时刻$$k-1$$和$$k$$上的对应特征集$$f_{k-1}$$和$$f_k$$，计算两幅图片$$I_{k-1}$$和$$I_k$$之间的转换$$T_k$$。根据特征对应是在2D还是3D下确认的，
我们可以使用不同的方法。<br>
- 2D-to-2D: 这个情况下，$$f_{k-1}$$和$$f_k$$都是用2D图像坐标表示。
- 3D-to-3D: 这个情况下，$$f_{k-1}$$和$$f_k$$都是用3D坐标表示。要实现这个，必须每个时刻都对3D点进行三角测量，比如借助一个双目相机系统。
- 3D-to-2D: 这个情况下，$$f_{k-1}$$是3D下确定，$$f_k$$是图$$I_k$$上它们对应的2D重投影。如果用的是单目相机，那么3D结构需要从两个临近的相机视角（例如，$$I_{k-2}$$和$$I_{k-1}$$）通过
三角测量得到，然后再与第三个视角（例如$$I_{k}$$）的2D图像特征进行匹配。所以说，单目系统中，匹配必须要有三个视角。

$$\quad$$ 注意，特征可以是点也可以是线。通常，由于结构很乱的场景中缺少特征线，我们在VO中会找出特征点。这篇文章中只考虑特征点。

#### 4.1 2D-to-2D: Motion from Image Feature Correspondences

> __Algorithm 1. VO from 2D-to-2D correspondences__ <br>
> 1) Capture new frame $$I_k$$ <br>
> 2) Extract and match features between $$I_{k-1}$$ and $$I_k$$  <br>
> 3) Compute essential matrix for image pair $$I_{k-1}$$, $$I_k$$ <br>
> 4) Decompose essential matrix into $$R_k$$ and $$t_k$$, and form $$T_k$$ <br>
> 5) Compute relative sclae and rescale $$t_k$$ accordingly <br>
> 6) Concatenate transformation by computing $$C_k = C_{k-1} T_k$$ <br>
> 7) Repeat from 1)

#### 4.2 3D-to-3D: Motion from 3-D Structure Correspondences

> __Algorithm 2. VO from 3D-to-3D correspondences__ <br>
> 1) Capture two stereo image pairs $$I_{l,k-1}, I_{r,k-1}$$ and $$I_{l,k}, I_{r,k}$$ <br>
> 2) Extract and match features between $$I_{l, k-1}$$ and $$I_{l,k}$$  <br>
> 3) Triangulate matched features for each stereo pair$ <br>
> 4) Compute $$T_k$$ from 3D features $$X_{k-1}$$ and $$X_k$$ <br>
> 5) Concatenate transformation by computing $$C_k = C_{k-1} T_k$$ <br>
> 6) Repeat from 1)

#### 4.3 3D-to-2D: Motion from 3-D Structure and Image Feature 

> __Algorithm 3. VO from 3D-to-2D correspondences__ <br>
> 1) Do only once: <br>
> 1.1) Capture two frames $$I_{k-2},I_{k-1}$$<br>
> 1.2) Extract and match features between them <br>
> 1.3) Triangulate features from $$I_{k-2},I_{k-1}$$ <br>
> 2) Do at each iteration:
> 2.1) Capture new frame $$I_k$$ <br>
> 2.2) Extract features and match with previous frame $$I_{k-1}$$ <br>
> 2.3) Compute camera pose (PnP) from 3D-to-2D matches <br>
> 2.4) Triangulate all new feature matches between $$I_k$$ and $$I_{k-1}$$ <br>
> 2.5) Iterate from 2.1)

#### 4.4 Triangulation and Keyframe Selection

$$\quad$$ 通过把至少两帧的图像对应处的back-projected rays相交，我们可以得到三角测量的3D点。在完美的情况下，这些射线会相交于一点。然而，因为图片的噪声，相机模型和标定错误记忆特征匹配
的不确定性，它们从来都不会相交。因此，我们必须对这个3D点做出估计，使得它与其它相交射线具有最小二乘。这个3D的好坏可以通过该点与其他射线的距离的标准差来判断。带有很强不确定性的点会被抛弃。这种情况
尤其在相对场景点距离很近的两帧中出现。当这个发生时，这个3D点会具有很大的不确定性。为了避免这种情况发生，有一种方法是跳过一些帧，知道3D点的平均不确定性降低过一个阈值。这剩下所选的帧称为关键帧（_keyframes_）
关键帧的选区是VO中非常重要的一步，它永远都必须在更新运动前执行。

#### 4.5 Discussion

$$\quad$$ 根据[Nister][paper-nister]，在运动计算中用2D-to-2D和3D-to-2D比用3D-to-3D更好。因为三角测量得到的3D点通常在深度方向具有不确定性。如果3D-to-3D用在运动计算中，那么这种
误差就会对运动估计有着很坏的影响。

$$\quad$$ 在单目方法中，2D-to-2D的方法比3D-to-2D更受欢迎，因为它避免了三角测量。然而实际上，3D-to-2D用的更多。因为它可以更快的进行数据关联。

$$\quad$$ 对于运动计算来说，输入数据中不能包含outlier，这一点十分重要。Outlier rejection是需要非常小心处理的，同时，它的计算时间和要用来计算运动所需要最少的点的数量有关。 2D-to-2D的方法
需要最少[5对点][website-ransac],然而，3D-to-2D只需要[3对][paper-three-point]。由于更少的点数，我们能更快地估计出运动。

$$\quad$$ 双目和单目相比，其中一个优势是，3D特征可以直接根据absolute scale算出，除此之外，单目要求必须要有3个视角图，而双目只需要有2个。另外，由于3D结构在双目中可以直接从单幅双目图中求得，
而不需要像单目一样从临近的帧中求出，所以双目算法在短程运动中具有更少的漂移量。单目方法之所以存在，是因为双目VO会在一些情况退化成单目，例如，到场景的距离远比双目baseline（两个相机的距离）的距离大。
在这种情况下，我们必须使用单目方法。

$$\quad$$ 不管选择哪种方法， local bundle adjustment (over the last m frames)必须永远使用，这样才能对轨迹做出一个更加精确的估计。在bundle adjustment后，运动估计的影响
会被大大减轻。

### 5. Conclusions
	
<br>
<br>
<br>

## [Part II: Matching, Robustness, and Applications][paper-part-2]

$$\quad$$ 本篇主要介绍feature matching, robustness和applications。它会回顾在VO中经常使用的point-feature detectors和不同的outlier-rejection schemes。
重点还会讲random sample consensus ([RANSAC][website-ransac])，以及一些用来提升它速度的独特的tricks。其他会提到的有error modeling, location recognition (or loop-closure detection)以及bundle adjustment.

### 1. Introduction

### 2. Feature Selection and Matching

$$\quad$$ 要想找到特征点以及确定它们的对应点主要有两种方法。第一种是在某幅图中先找到一些特征，再在接下来的一些帧中利用局部搜索的方式跟踪它们，例如correlation。第二个方法是随意地检测所有图像中
的特征，然后基于它们descriptors的相似度比较来对它们进行匹配。第一种方法更适用于在临近视角拍的图像，后一种更适于远距离的运动以及视角的改变。

#### 2.1 Feaure Detection

$$\quad$$ 在特征检测这一步中，图像上那些突出的，容易与其他图像进行匹配的特征点会被搜寻出来。

$$\quad$$ 一个局部特征是指图像上某个与它临近部位明显不同的图像纹路，不同的地方可以是强度，颜色，质地。对于VO来说，检测特征点，例如拐角或者斑点，是非常重要的，因为通常能很精确地测量到它们在图像中的位置。

$$\quad$$ 一个拐角（corner）定义为处于两条或多条边相交的点。一个半点（blob）是指图像上在强度，颜色和质地方面与周围不同的图案。

$$\quad$$ 一个好的特征应该具有以下性质： 
	- localization accuracy (both in position and scale)
	- repeatablility (a large number of features should be re-detected in the next images)
	- computational efficiency
	- robustness (to noise, compression artifacts, blur)
	- distinctiveness (so that features can be matched accurately across different images)
	- invariance (to both photometric changes such as illumination and geometric changes such as rotation, sclae and perspective distortion)

$$\quad$$ 一个对于不同检测器的概览可以参考这篇[文献][book-introduction-robot]。角检测器通常运算很快，但是区分度不够高，相反的，斑点检测器区分度更高，但运算很慢。另外，角在图像中更容易被定位，但是
它们在尺寸变化后更难定位。这意味着，角不能像斑点一样，经过长时间的尺度变化或者视角变化，还能被重新检测出来。然而，斑点对于某些环境来说也不是最好的选择，例如，[SIFT][website-sift]会自动忽略拐角，然而，
在城市里面经常会有拐角的出现。基于这些理由，我们在选择检测器时，应该考虑computational constraints, real-time requirementss, environment type, and motion baseline (i.e.
how nearby images are taken).下图是对不同检测器的对比，另外还可以参考这篇[论文][paper-comparison-detector]。

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/post-comparison-detector.PNG)

$$\quad$$ 每个检测器都有两步。第一步时对整个图像使用一个 _feature-response_ function，例如Harris detector的corner response function或者是SIFT的difference-of-Gaussian
operator。第二步时确认所有的local minima (or maxima) of the feature-response function。Non-maxima suppression后的输出便是检测到的特征。要想使得检测器不被尺度变化所影响，
我们可以将检测器作用在同样图像的lower-sclae和upper-scale的版本上。要想不被透视变化（perspective change）所影响，我们得把透视畸变近似于仿射变换。

#### 2.2 Feature Descriptor 

$$\quad$$ 在特征描述这一步里，每个被检测到的特征周围的区域会被转换成一个紧密的描述器，这个描述器可以用来与其他描述器进行配对。

$$\quad$$ 最简单的的描述一个特征的方法是用它的外观，也就是说这个特征点那一块所有像素的强度值。在这个情况下，误差可以用[sum of squared differences (SSD)或者
normalized cross correlation (NCC)][book-digital-image-processing]来比较强度。相比SSD，NCC可以更好地抵消亮度变化。还有一种测量图片相似度
的方法-[Census transform][paper-census]。这个方法将每个图片块用一个binary vector来表示，其中的值表示邻近的点比中心像素高或低。两个图片块的相似度便可以通过汉明距离来表示。

$$\quad$$ 很多情况下，其实特征的局部外貌并不是一个好的描述器，因为它的外貌容易随着方向，尺度以及视点的改变而改变。实际上，NCC和SSD对这个变化都没有不变性，因此，它们只能作用于处于邻近位置的相片。
一种很受欢迎的描述特征点的描述器是[SIFT][website-sift]。除此之外，还出现了[ORB，BRIEF][website-keypoint-descriptor]。

#### 2.3 Feature Matching

$$\quad$$ 特征匹配这一步里，我们寻找其他图片中对应的特征。最简单的方法就是比较两幅图片所有的描述器，比较它们的相似度(similarity measure)。

1) _Mutual consistency:_ 在匹配两幅图的特征点的过程中，可能会在第二幅图中找到多个特征与第一幅图对应，要想找到合适的对应特征，我们要采取the _mutual consistency check_。
也就是说，我们要在图一中找到与图二对应的特征点。一对特征成功匹配的要求是，它们互相是对方最相似的特征点。

2) _Constrained matching:_ 之前匹配的缺点是太耗时间了，它随着特征数的数量平方而增长。一个更好的方式是使用an indexing structure, such as a multi-dimensional 
search tree or a hash table。一种更快的搜索方式是只找寻第二幅图中特征图应该在的一些区域。这些区域我们可以通过一个运动模型和3D特征点的位置（如果有）预测获得。<br>
如果只知道运动模型，但不知道3D特征点的位置，我们便可以采取 _epipolar matching_ 的方法。 <br>
在双目视觉中，我们不需要为每个特征算epipolar line，因为这些图像通常都已经被矫正了。Image rectification是一个将一对图进行重映射到一对新图上的过程，新的左右两图上的epipolar lines
都会水平并且互相对齐。这对特征匹配很有帮助。然而，如果运动也具有不确定性，那么epipolar search通常会称为一个与epipolar line有确定距离的矩形的区域。

#### 2.4 Feature Tracking

$$\quad$$ An alternative to independently finding features in all candidate images and then matching them is to detect features in the first image and,
then, search for their corresponding matches in the next images. This _detect-then-track_ approach is suitable for VO applications where 
images are taken at nearby locations, where the amount of motion and appearance deformation between adjancent frames is small. 

$$\quad$$ However, if features are tracked over long image sequences, their appearance can undergo larger changes. In this case, the solution 
is to apply an affine-distortion model to each feature. The resulting tracker is often called [KanadeLucasTomasi （KLT） tracker][paper-klt-tracker].

#### 2.5 Discussion

_1) SIFT matching:_ In many cases it might be beneficial to skip the ratio test and let RANSAC take care of the outliers.

_2) Lines and edgelets:_ Contrary to points, lines are more difficult to match because lines are more likely to be occluded than points. Furthermore,
the origin and end of a line segment of edgelet may not exist.

_3) Number of features and distribution:_ The distribution of the features in the image affect the VO results remarkably. In particular, more features
provide more stable motion-estimation results than with fewer features, but at the same time the keypoints should cover as evenly as possible the 
image . In order to do this, the image can be partitioned into a grid and the feature detector is applied to each cell by tuning the detection 
threshholds untile a minimum number of feature is found in each subimage. As a rule of the thumb, one thousand features is a good number for a 
640 x 480-pixel image.

_4) Dense and correspondence-free methods:_ An alternative to _sparse_-feature extraction is to use dense methods, such as optical flow. Another 
alternative is feature-less motion-estimation methods.

### 3. Outlier Removal

#### 3.1 RANSAC

#### 3.2 Minimal Model Parameterizations: 8, 7, 6, 5, 4, 2, and 1-point RANSAC

#### 3.3 Reducing the Iterations of RANSAC

#### 3.4 Is it really better to use a minimal set in RANSAC?

### 4. Error Propagation

### 5. Camera Pose Optimization 

#### 5.1 Pose-Graph Optimization

#### 5.2 Windowed (or Local) Bundle Adjustment

### 6. Applications

### 7. Conclusions 

[paper-part-1]: https://www.ifi.uzh.ch/dam/jcr:5759a719-55db-4930-8051-4cc534f812b1/VO_Part_I_Scaramuzza.pdf
[paper-part-2]: http://rpg.ifi.uzh.ch/docs/VO_Part_II_Scaramuzza.pdf
[paper-nister]: https://www.computer.org/csdl/proceedings/cvpr/2004/2158/01/01315094.pdf
[website-harris-corner]: https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_feature2d/py_features_harris/py_features_harris.html
[paper-comport]: http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.331.9823&rep=rep1&type=pdf
[website-ransac]: http://lingtong.de/2018/10/30/RANSAC/
[paper-why-filter]: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=5509636
[paper-three-point]: http://rpg.ifi.uzh.ch/docs/CVPR11_kneip.pdf
[book-introduction-robot]: https://mitpress.mit.edu/books/introduction-autonomous-mobile-robots-second-edition
[website-sift]: https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_feature2d/py_sift_intro/py_sift_intro.html
[paper-comparison-detector]: https://pdfs.semanticscholar.org/fdb0/f0abbb6ec7d7b9076adab6c69d8f40ad6e02.pdf
[book-digital-image-processing]: http://web.ipac.caltech.edu/staff/fmasci/home/astro_refs/Digital_Image_Processing_2ndEd.pdf
[paper-census]: http://www.cs.cornell.edu/~rdz/Papers/ZW-ECCV94.pdf
[website-keypoint-descriptor]: http://lingtong.de/2018/10/27/Keypoint-and-Descriptor/
[paper-klt-tracker]: http://www.ai.mit.edu/courses/6.891/handouts/shi94good.pdf