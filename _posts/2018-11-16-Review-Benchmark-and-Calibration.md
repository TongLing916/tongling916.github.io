---
layout:     post
title:      "Review - Photometric Calibration"
date:       2018-11-16
author:     Tong
catalog: true
tags:
   - Review
---

## [A Photometrically Calibrated Benchmark For Monocular Visual Odometry][paper-benchmark]

### 0	Abstract
We present a dataset for evaluating the tracking accuracy of monocular visual odometry and SLAM methods. It contains 50 real-world sequences comprising more than 100 minutes of video, recorded across dozens of different environments -- ranging from narrow indoor corridors to wide outdoor scenes. All sequences contain mostly exploring camera motion, starting and ending at the same position. This allows to evaluate tracking accuracy via the accumulated drift from start to end, without requiring ground truth for the full sequence. In contrast to existing datasets, all sequences are photometrically calibrated. We provide exposure times for each frame as reported by the sensor, the camera response function, and dense lens attenuation factors. We also propose a novel, simple approach to non-parametric vignette calibration, which requires minimal set-up and is easy to reproduce. Finally, we thoroughly evaluate two existing methods (ORB-SLAM and DSO) on the dataset, including an analysis of the effect of image resolution, camera field of view, and the camera motion direction.

### 1	Introduction
1.	Sensor Intrinsics: many existing methods are designed to operate on, and are evaluated with, data captured by commodity cameras without take advantage of knowing the full image formation pipeline. 
2.	Specifically, methods are designed to be robust to (assumed unknown) automatic exposure changes, non-linear response functions (gamma correction), lens attenuation (vignetting), de-bayering artifacts, or even strong geometric distortions caused by a rolling shutter. This is true for modern keypoint detectors and descriptors, which are robust or invariant to arbitrary monotonic brightness changes. 
3.	Sensors – including cameras – can and will be designed to fit the needs of the algorithms processing their data. In turn, algorithms should take full advantage of the sensor’s capabilities and incorporate knowledge about the sensor design. 
4.	Simple examples are image exposure time and hardware gamma correction, which are intentionally built into the camera to produce better images. Instead of treating them as unknown noise factors and attempt tot correct for them afterwards, they can be treated as feature that can be modelled by and incorporated into the algorithm – rendering the obtained data more meaningful. 
5.	Benchmark Size: to obtain a meaningful comparison between different methods and to avoid manual overfitting to specific environments or motion patterns (except for cases where this is specifically desired), algorithms should be evaluated on large datasets in a wide variety of scenes. 
6.	However, existing datasets often contain only a limited number of environments. The major reason for this is that accurate ground truth acquisition is challenging, in particular if a wide range of different environments is to be covered: GPS/INS is limited in accuracy and only possible in outdoor environments with adequate GPS reception. External motion capture systems on the other hand are costly and time-consuming to set up and can only cover small (indoor) environments. 
7.	The dataset attempts to tackle these two issues. 
-	First, it contains frame-wise exposure times as reported by the sensor, as well as accurate calibrations for the sensor’s response function and lens vignetting, which enhances the performance particularly of direct approaches.
-	Second, it contains 50 sequences with a total duration of 105 minutes, captured in dozens of different environments. 
8.	Tracking accuracy is evaluated by measuring the accumulated drift that occurs after a large loop.

#### 1.1	Related Work: Datasets
1.	KITTI: 21 stereo sequences recorded form a driving car, motion patterns and environments are limited to forward-motion and street-scenes. Images are pre-rectified, raw sensor measurements or calibration datasets are not available. The benchmark contains GPS-INS ground truth poses for all frames. 
2.	EUROC MAV: 11 stereo-inertial sequences from a flying quadcopter in three different indoor environments. The benchmark contains ground truth poses for all frames, as well as the raw sensor data and respective calibration datasets. 
3.	TUM RGB-D: 89 sequences in different categories in various environments, recorded with a commodity RGB-D sensor. They contain strong motion blur and rolling-shutter artifacts, as well as degenerate (rotation-only) motion patterns that cannot be tracked well from monocular odometry alone. Sequences are pre-rectified, the raw sensor data is not available. The benchmark contains ground truth poses for all sequences. 
4.	ICL-NUIM: 8 ray-traced RGB-D sequences from 2 different environments. It provides a ground truth intrinsic calibration; a photometric calibration is not required, as the virtual exposure time is constant. Some of the sequences contain degenerate (rotation-only) motion patterns that cannot be tracked well from a monocular camera alone. 

#### 1.2 Related Work: Photometric Calibration

#### 1.3	Paper Outline

### 2	Calibration
#### 2.1	Hardware

#### 2.2	Geometric Intrinsic Calibration
1.	We use the pinhole camera model in combination with a FOV distortion mode, since it is well-suited for the used fisheye lenses.  
2.	A useful property of this model is the existence of a closed-form inverse: for a given point the image and depth, the corresponding 3D point can be computed by first converting it back to normalized image coordinates. 

#### 2.3	Photometric Calibration 
1.	The combined image formation model is I(x) = G(tV(x)B(x)).
-	I: the overserved value
-	G: the camera response function 
-	t: the exposure time
-	V: pixel-wise attenuation factor
-	B: the irradiance image (up to a scalar factor)
-	U: the inverse response function.

##### 2.3.1	Response Calibration 
1.	The resulting U may not be monotonic – which is a pre-requisite for invertibility. In this case it needs to be smoothed or perturbed.
2.	We do not employ a smoothness prior on U – instead, we use large amounts of data. This is done by recording a video of a static scene while slowly changing the camera’s exposure. 
3.	If only a small number of images or exposure times is available, a regularized approach will be required. 

##### 2.3.2	Non-parametric Vignette Calibration 
1.	We estimate a non-parametric (dense) vignetting map V from a sequence of images showing a planar scene.
2.	Apart from planarity, we only require the scene to have a bright color and to be fully Lambertian – in practice, a predominantly white walls serves well.
3.	We estimate the camera pose with respect to the planar surface using an AR marker.  

### 3	Evaluation
#### 3.1	Evaluation from Loop-Closure 
1.	To provide better comparability between the sequences, the ground truth scale is normalized such that the full trajectory has a length of approximately 100.
2.	The tracking accuracy of a VO method can then be evaluated in terms of the accumulated error (drift) over the full sequence. Note that this evaluation method is only valid if the VO/SLAM method does not perform loop-closure itself. To evaluate full SLAM systems like ORB-SLAM, loop-closure detection needs to be disabled. 
3.	Even for full SLAM methods, the amount of drift accumulated before closing the loop is a good indicator for the accuracy. In particular, it is strongly correlated with the long-term accuracy after loop-closure. 
 
#### 3.2	Error Metric
1.	First, we align the tracked trajectory with both the start- and end-segment independently.
2.	We further define a combined error measure, the alignment error, which equally takes into account the error caused by scale, rotation and translation drift over the full trajectory. 
3.	We choose this metric, since 
-	It can be applied to other SLAM / VO modes with different observability modes (like visual-inertial or stereo).
-	It is equally affected by scale, rotation, and translation drift, implicitly weighted by their effect on the tracked position, 
-	It can be applied for algorithms which compute only poses for a subset of frames (e.g. keyframes), as long as start- and end-segment contain sufficient frames for alignment, and 
-	It better reflects the overall accuracy of the algorithm than the translational drift or the joint RMSE.

### 4	Benchmark
1.	When evaluating accuracy of SLAM or VO methods, a common issue is that not all methods work on all sequences. 
2.	This is particularly true for monocular methods, as sequences with degenerate (rotation-only) motion or entirely texture-less scenes (white walls) cannot be tracked. 
3.	A common approach hence is to show only results on a hand-picked subset of sequences on which the compared methods do not fail (encouraging manual overfitting), or to show large tables with error values, which is not practicable for a dataset containing 50 sequences. 
4.	A better approach is to summarize tracking accuracy as cumulative distribution, visualizing on how many sequences the error is below a certain threshold – it shows both the accuracy on sequences where a method works well, as well as the method’s robustness, i.e., on how many sequences it does not fail.
5.	Algorithm Parameter Settings: Since both methods do not support the FOV camera model, we run the evaluation on pinhole-rectified images with VGA resolution. Further, we disable explicit loop-closure detection and re-localization to allow application of our metric, and reduce the threshold where ORB-SLAM decides it is lost to 10 inlier observations. 
6.	Data Variations: A good way to further to analyze the performance of an algorithm is to vary the sequence in a number of ways.
7.	Ground Truth Validation: Since for most sequences, the used loop-closure ground truth is computed with a SLAM-algorithm itself, it is not perfectly accurate. We can however validate it by looking at the RMSE when aligning the start- and end-segment.

#### 4.1	Dataset
1.	 Raw camera images of all 50 sequences, with frame-wise exposure times and computed ground truth alignment of start- an end-segment.
2.	Geometric (FOV distortion model) and photometric calibrations (vignetting and response function).
3.	Calibration datasets containing 
-	checkerboard-images for geometric calibration
-	several sequences suitable for the proposed vignette and response function calibration 
-	images of a uniformly lit white paper
4.	Minimal c++ code for reading, pinhole-rectifying, and photometrically undistorting the images, as well as for performing photometric calibration.
5.	Matlab scripts to compute the proposed error metrics, as well as the raw tracking data for all runs used to create the plots and figures.

#### 4.2	Known Issues
1.	Even though we use industry-grade cameras, the SDK provided by the manufacturer only allows to asynchronously query the current exposure time. Thus, in some rare cases the logged exposure time may be shifted by one frame. 




## [Online Photometric Calibration of Auto Exposure Video for Realtime Visual Odometry and SLAM][paper-calibration]


### 0	Abstract
Recent direct visual odometry and SLAM algorithms have demonstrated impressive levels of precision. However, they require a photometric camera calibration in order to achieve competitive results. Hence, the respective algorithm cannot be directly applied to an off-the-shelf-camera or to a video sequence acquired with an unknown camera. In this work we propose a method for online photometric calibration which enables to process auto exposure videos with visual odometry precisions that are on par with those of photometrically calibrated videos. Our algorithm recovers the exposure times of consecutive frames, the camera response function, and the attenuation factors of the sensor irradiance due to vignetting. Gain robust KLT feature tracks are used to obtain scene point correspondences as input to a nonlinear optimization framework. We show that our approach can reliably calibrate arbitrary video sequences by evaluating it on datasets for which full photometric ground truth is available. We further show that our calibration can improve the performance of a state-of-the-art direct visual odometry method that works solely on pixel intensities, calibrating for photometric parameters in an online fashion in realtime.

### 1	Introduction and Related Work
1.	DSO or LSD-SLAM work only on pixel intensities. 
2.	They all rely on the underlying assumption, that a scene point appears with constant brightness values across multiple images. 
3.	When taking images with auto exposure video cameras, this assumption typically does not hold.  
4.	The automatic adjustment of the exposure times, the photometric falloff of the pixel intensities to the sides of the image due to vignetting as well as an often nonlinear camera response function cause the observed pixel intensities to differ for the same scene point.
5.	Our algorithm builds on the work, applying their nonlinear estimation formulation to arbitrary video sequences using gain robust feature tracking, recovering response function, vignetting, exposure times and radiances of the tracked scene points. 
6.	We track features with large radial motion across multiple frames in order to recover the vignetting reliably.
7.	In the case of vignetted video, we do not require any exposure change to calibrate for the parameters, in contrast tot methods which only estimate for a response function. 
8.	Using our algorithm in parallel to a visual odometry or visual SLAM method can significantly enhance its performance when running on datasets with photometric disturbances. 

### 2	Photometric image formation process
1.	A scene point is illuminated by a light source and reflects the light back into space.
2.	The amount of light reflected is called the radiance L of the scene point.
3.	If the radiance received by a moving observer is independent of the observers viewing angle, the scene point is called to exhibit Lambertian reflectance behavior. 
4.	The total amount of energy received at sensor location x is called the Irradiance I(x).
5.	For most cameras a radiometric fall off of the pixel intensities can be observed towards the image borders. This so called vignetting effect can be either due to a partial blocking of light rays by the lens barrel or induced by the lends geometry, modeled by the Cosine-Fourth law.
6.	When taking an image, the sensor irradiance is integrated over a time window specified by the camera exposure time e. We assume the irradiance of a sensor element to be constant over this window.
7.	For our work, since the physical scale of the radiances cannot be recovered, the dynamic range of the camera is normalized to the unit interval [0, 1].

### 3	Tracking frontend
1.	In order to obtain a photometric calibration, a set of scene points P must be selected and their projections onto the images where they are visible must be estimated. 
2.	To do this, we use a pyramidal implementation of the KLT tracker to obtain point correspondences. 
3.	We optimize jointly for the tracking updates and a gain ratio between the two trackers when applied on frames with larger exposure change. 
4.	We extract Shi-Thomasi corners which well constrain the solution to the optical flow equation, constituting good candidate points for tracking. 
5.	In order to reliably recover the vignetting, it is important to sample features uniformly across the image to cover the entire radial range. Therefore, the image is divided into a number of grid cells and a total sum of N features is sampled from all the cells. This is also beneficial in order to cover the entire image intensity range which is required to constrain the response function well. 
6.	We use long feature tracks in order to increase radial movements, which are necessary for vignetting estimation since, if the radial movements of the tracked features are small, the changes of irradiance due to spatial photometric fall-off will not be captured. 
7.	In order to increase the number of measurements without having to extract further features from the image, all pixels from a small image patch around tracked feature locations are included in the optimization. 
8.	Extracting an image patch aims at obtaining low gradient correspondences, whose image intensity profiles will be less sensitive with respect to small tracking errors. 

### 4	Optimization backend
1.	To model the CRF (camera response function), we use the empiric model of response (EMoR) introduced by Grossberg and Nayar. 
2.	A principle component analysis (PCA) is applied to find the mean response and basis function which can be linearly combinated to form the overall response function by choosing parameters.  
3.	Since estimating a vignetting factor at every pixel location of the image is not feasible without a very large number of correspondences, we employ a flexible radian vignetting model, assuming that the attenuation factors are symmetric around the image center. Furthermore, we assume that the center of vignetting falls together with the center of the image. 
4.	The dependency structure of the optimization problem is exploited to decouple the estimation of the irradiances from the other parameters. 
5.	We distinguish between running our algorithm in an offline and an online fashion. 
6.	When calibrating offline, we assume to have access to the entire input sequence and no hard constraints on the running time. In the online setting images are received frame by frame and no information about future frames can be obtained. We aim at providing photometrically calibrated frames as input to a direct method running in parallel. 
7.	Algorithmic overview of the system running in online mode. Input frames are partially calibrated by removing disturbances due to vignetting and camera response and features are tracked forward from the previous input frame to the current one. An exposure time is estimated based on the tracks of the last M input frames. In the background, nonlinear optimization is performed to continuously update the vignette and response function estimates. 
![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/post-calibration-online-mode.png)
 
### 5	Evaluation

### 6	Conclusion
1.	We propose a novel system for providing realtime online calibration of auto exposure video for direct formulations of visual odometry and visual SLAM, estimating for the photometric response function, vignetting, and exposure times.  
2.	We show that our algorithm provides an accurate and robust photometric calibration for arbitrary video sequences and significantly enhances the quality of direct methods for visual odometry such as DSO.
3.	We plan to substitute the KLT tracking by integrating the proposed online optimization into existing direct methods where constancy assumption to jointly optimize for the photometric parameters, the camera poses, and depth values. 




[paper-benchmark]: https://vision.in.tum.de/research/vslam/photometric-calibration
[paper-calibration]: https://vision.in.tum.de/research/vslam/photometric-calibration
