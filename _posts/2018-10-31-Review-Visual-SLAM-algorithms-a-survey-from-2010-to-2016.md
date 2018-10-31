---
layout:     post
title:      "Review - Visual SLAM algorithms: a survey from 2010 to 2016"
date:       2018-10-31
author:     Tong
catalog: true
tags:
   - Review
---

> The paper can be found on this [website][paper-visual-slam].

### 1. Introduction
$$\quad$$ Direct approach: to cope with texture-less or feature-less environments, vSLAM without detecting feature points and directly with 
a whole image for tracking and mapping.

### 2. Elements of vSLAM

#### 2.1 Basic modules

1. Initialization: define the global coordinate system for camera pose estimation and a part of the environment is reconstructed as an initial 
map in the global coordinate system.
 
2. Tracking: the reconstructed map is tracked in the image to estimate the camera pose of the image with respect to the map. Note that most of vSLAM 
algorithms assumes that intrinsic camera parameters are calibrated beforehand so that they are known. Therefore, a camera pose is normally 
equivalent to extrinsic camera parameters with translation and rotation of the camera in the global coordinate system.
 
3. Mapping: the map is expanded by computing the 3D structure of an environment when the camera observes unknown regions where the mapping is 
not performed before.

#### 2.2 Additional modules for stable and accurate vSLAM

1. Relocalization: the relocalization is required when the tracking is failed due to fast camera motion or some disturbances. In this case, it is 
necessary to compute the camera pose with respect tot the map again. (kidnapped robot problems)

2. Global map optimization: the map generally includes accumulative estimation error according to the distance of camera movement. In order to suppress 
that error, the global map optimization is normally performed. In this process, the map is refined by considering the consistency of the whole map 
information. When a map is revisisted such that a starting region is captured again after some camera movement, reference information that 
represents the accumulative error from the beginning to the present can be computed. Then, a loop constraint from the reference information is used 
as a constraint to suppress the error in the global optimization. <br> Loop closing is a technique to acquire the reference information. In the loop 
closing, a closed loop is first searched by matching a current image with previously acquired images. Note that the closed-loop detection procedure 
can be done by using the same techniques as relocalization. Basically, relocalization is done for recovering a camera pose and loop detection is 
done for obtaining geometrically consistent map.

$$\quad$$ Pose-graph optimization has widely been used to suppress the accumulated error by optimizing camera poses. More information can be found 
in this [paper][paper-graph-based] and [this][paper-g2o]. Bundle adjustment (BA) is also used to minimize the reprojection error of the map by 
optimizing both the map and the camera poses. More information can be found [here][paper-ba].

#### 2.3 Summary

$$\quad$$ TAM (tracking and mapping) was first used in Parallel Tracking and Mapping ([PTAM][paper-ptam]). Tracking is performed in every frame with 
one thread whereas mapping is performed at a certain timing with another thread.

### 3. Related technologies

#### 3.1 [Visual odometry][website-vo]

> vSLAM = VO + global map optimization 

$$\quad$$ The main difference between these two techniques is global map optimization in the mapping.

#### 3.2 Structure from motion

$$\quad$$ There is no definitive difference between vSLAM and real-time SfM.

### 4. Feature-based methods

#### 4.1 [MonoSLAM][paper-monoslam]

#### 4.2 [PTAM][paper-ptam]

#### 4.3 Comparison between MonoSLAM and PTAM

$$\quad$$ The difference between the EKF-based mapping in MonoSLAM and the BA-based mapping with the keyframes in PTAM was discussed in this 
[paper][paper-why-filter]. To improve an accuracy of vSLAM, it is important to increase the number of feature points in a map. From this point of 
view, the BA-based approach is better than the EKF-based approach because it can handle large number of points. 

#### 4.4 Techniques on global map optimization

$$\quad$$ Geometric consistency of the whole map is maintained by using BA for the keyframes. However, in general, BA suffers from a local minimum 
problem due to the numerous number of parameters including camera poses of the keyframes and points in the map.

$$\quad$$ Pose-graph optimization is a solution to avoid this problem in the loop closing. In the loop closing, camera poses are first optimized 
using the loop constraint. After optimizing the camera poses, BA is performed to optimize both 3D positions of feature points and the 
camera poses.

$$\quad$$ In monocular vSLAM cases, there is a sclae ambiguity and a sclae may change during camera movement if global BA is not performed. In this 
case, a scale drift problem occurs and the scale of the coordinate system at each frame may not be consistent. In order to correct the scale drift, 
camera poses should be optimized in 7 DoF. This [paper][paper-7-DoF] proposed a method for optimizing 7 DoF camera poses based on similarity 
transformation. 

$$\quad$$ As an extension of PTAM, ORB-SLAM includes BA, vision-based closed-loop detection, and 7 DoF pose-graph optimization.

#### 4.5 Summary


### 5. Direct methods

#### 5.1 DTAM

#### 5.2 LSD-SLAM

#### 5.3 SVO and DSO

#### 5.4 Summary

### 6. RGB-D vSLAM

#### 6.1 RGB-D vSLAM

#### 6.2 KinectFusion

#### 6.3 SLAM++

#### 6.4 Techniques on RGB-D VO and global map optimization

### 7. Open problems

#### 7.1 Pure rotation

#### 7.2 Map initialization

#### 7.3 Estimating intrinsic camera parameters

#### 7.4 Rolling shutter distortion

#### 7.5 Scale ambiguity

### 8. Benchmarking

### 9. Conclusions


[paper-visual-slam]: https://ipsjcva.springeropen.com/track/pdf/10.1186/s41074-017-0027-2
[paper-ptam]: https://www.robots.ox.ac.uk/~vgg/rg/papers/klein_murray__2007__ptam.pdf
[paper-monoslam]: https://www.doc.ic.ac.uk/~ajd/Publications/davison_etal_pami2007.pdf
[paper-graph-based]: http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti10titsmag.pdf
[paper-g2o]: https://europa.informatik.uni-freiburg.de/files/kuemmerle11icra.pdf
[paper-ba]: http://lingtong.de/2018/11/01/Bundle-Adjustment/
[website-vo]: http://lingtong.de/2018/10/29/Review-VO-Part-I-And-II/
[paper-why-filter]: https://www.doc.ic.ac.uk/~ajd/Publications/strasdat_etal_ivc2012.pdf
[paper-7-DoF]: http://roboticsproceedings.org/rss06/p10.pdf