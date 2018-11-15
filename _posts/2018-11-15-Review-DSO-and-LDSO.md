---
layout:     post
title:      "Review - DSO (Direct sparse odometry) and LDSO (DSO with loop closure)"
date:       2018-11-15
author:     Tong
catalog: true
tags:
   - Review
---

## [Direct Sparse Odometry][paper-dso]

### 0	Abstract
Direct Sparse Odometry (DSO) is a visual odometry method based on a novel, highly accurate sparse and direct structure and motion formulation. It combines a fully direct probabilistic model (minimizing a photometric error) with consistent, joint optimization of all model parameters, including geometry-represented as inverse depth in a reference frame-and camera motion. This is achieved in real time by omitting the smoothness prior used in other direct methods and instead sampling pixels evenly throughout the images. Since our method does not depend on keypoint detectors or descriptors, it can naturally sample pixels from across all image regions that have intensity gradient, including edges or smooth intensity variations on essentially featureless walls. The proposed model integrates a full photometric calibration, accounting for exposure time, lens vignetting, and non-linear response functions. We thoroughly evaluate our method on three different datasets comprising several hours of video. The experiments show that the presented approach significantly outperforms state-of-the-art direct and indirect methods in a variety of real-world settings, both in terms of tracking accuracy and robustness.

### 1	Introduction
1.	Indirect methods proceed in two steps. First, the raw sensor measurements are pre-processed to generate an intermediate representation, solving part of the overall problem, such as establishing correspondences. Second, the computed intermediate values are interpreted as noisy measurements Y in a probabilistic model to estimate geometry and camera motion. 
2.	Direct methods skip pre-processing step and directly use the actual sensor values – light received from a certain direction over a certain time period – as measurements Y in a probabilistic model. 
3.	In the case of a passive vison, the direct approach thus optimizes a photometric error, since the sensor provides photometric measurements. Indirect methods on the other hand optimize a geometric error, since the pre-computed values – point-positions or flow-vectors – are geometric quantities. 
4.	Sparse methods use and reconstruct only a selected set of independent points (traditionally corners), whereas dense methods attempt to use and reconstruct all pixels in the 2D image domain. Intermediate approaches (semi-dense) refrain from reconstructing the complete surface, but still aim at using and reconstructing a (largely connected and well-constrained) subset.
5.	A more fundamental and consequential difference between dense and sparse lies in the addition of a geometry prior. In the sparse formulation, there is no notion of neighborhood, and geometry parameters (keypoint positions) are conditionally independent given the camera poses & intrinsics. Dense (or semi-dense) approaches on the other hand exploit the connectedness of the used image region to formulate a geometry prior, typically favoring smoothness. 

#### 1.1	Motivation
1.	One of the main benefits of a direct formulation is that it does not require a point to be recognized by itself, thereby allowing for a more finely grained geometry representation (pixelwise inverse depth). 
2.	Furthermore, we can sample from across all available data – including edges and weak intensity variations – generating a more complete model and lending more robustness in sparsely textured environments. 
3.	The main drawback of adding a geometry prior is the introduction of correlations between geometry parameters, which render a statistically consistent, joint optimization in real time infeasible. In addition, the expressive complexity of today’s priors is limited: Priors can introduce a bias, and thereby reduce long-term, large-scale accuracy.

#### 1.2 Contribution and Outline
1.	DSO is the only fully direct method that jointly optimizes the full likelihood for all involved model parameters, including camera poses, camera intrinsics, and geometry parameters (inverse depth values).
2.	Optimization is performed in a sliding window, where old camera poses as well as points that leave the field of view of the camera are marginalized. 
3.	In contrast to existing approaches, our method further takes full advantage of photometric camera calibration, including lens attenuation, gamma correction, and known exposure times. This integrated photometric calibration further increases accuracy and robustness. 

### 2	Direct Sparse Model
1.	DSO is based on continuous optimization of the photometric error over a window of recent frames, taking into account a photometrically calibrated model for image formation. 
2.	In contrast to other direct methods, we jointly optimize for all involved parameters (camera intrinsics, camera extrinsics, and inverse depth values).
3.	We keep the geometry representation employed by other direct approaches, i.e., 3D points are represented as inverse depth in a reference frame (and thus have one degree of freedom).

#### 2.1	Calibration
1.	A geometric camera model comprises the function that projects a 3D point onto the 2D image. 
2.	It is beneficial to consider a photometric camera model, which comprises the function that maps real-world energy received by a pixel on the sensor (irradiance) to the respective intensity value. (But this is only for direct methods.)

##### 2.1.1	Geometric Camera Calibration
1.	Our approach can be extended to other (invertible) camera models.

##### 2.1.2	Photometric Camera Calibration

#### 2.2	Model Formulation
1.	We define the photometric error of a point in reference frame, observed in a target frame, as the weighted SSD (Sum of square differences) over a small neighborhood of pixels.
2.	Our experiments have shown that 8 pixels, arranged in a slightly spread pattern give a good trade-off between computations required for evaluation, robustness to motion blur, and providing sufficient information. Note that in terms of the contained information, evaluating the SSD over such a small neighborhood of pixels is similar to adding first- and second-order irradiance derivative constancy terms (in addition to irradiance constancy) for the central pixel.  
3.	In order to allow our method to operate on sequences without known exposure times, we include an additional affine brightness transfer function.
4.	In addition to using robust Huber penalties, we apply a gradient-dependent weighting. 
5.	This weighting function can be probabilistically interpreted as adding small, independent geometric noise on the projected point position, and immediately marginalizing – approximating small geometric error. 
6.	To summarize, the error depends on the following variables: (1) the point’s inverse depth, (2) the camera intrinsics, (3) the poses of the involved frames, and (4) their brightness transfer function parameters. 
7.	The only difference to the classical reprojection error is the additional dependency of each residual on the pose of the host frame, i.e., each term depends on two frames instead of only one. (While this adds off-diagonal entries to the pose-pose block of the Hessian, it does not affect the sparsity pattern after application of the Schur complement to marginalize point parameters. The resulting system can thus be solved analogously to the indirect formulation.)
8.	If exposure times are known, we further add a prior pulling the affine brightness transfer function to zero. 
9.	Point Dimensionality: In the proposed direct model, a point is parametrized by only one parameter (the inverse depth in the reference frame), in contrast to three unknowns as in the indirect model.
10.	Reason for point dimensionality: note that in both cases a 3D points is in fact an arbitrarily located discrete sample on a continuous, real-world 3D surface. The difference then lies in the way this 2D location on the surface is defined. In the indirect approach, it is implicitly defined as the point, which (projected into an image) generates a maximum in the used corner response function. This entails that both the surface, as well as the point’s location on the surface are unknowns, and need to be estimated. In our direct formulation, a point is simply defined as the point, where the source pixel’s ray hits the surface, thus only one unknown remains. In addition to a reduced number of parameters, this enables an inverse depth parameterization, which – in a Gaussian framework – is better suited to represent uncertainty from stereo-based depth estimation, in particular for far-away points. 
11.	Consistency: The proposed direct sparse model does not allow to use some observations (pixel values) multiple times, while others are not used at all. This is because we allow point observations to overlap, and thus depend on the same pixel values(s).

#### 2.3	Windowed Optimization 
1.	We optimize the total error in a sliding window using the Gauss-Newton algorithm, which gives a good trade-off between speed and flexibility. 
2.	Marginalizing a residual that depends on a parameter will fix the tangent space in which any future information (delta-updates) on that parameter is accumulated. 
3.	Marginalization: When the active set of variables becomes too large, old variables are removed by marginalization using the Schur complement. We drop any residual terms that would affect the sparsity pattern of H.

### 3	Visual Odometry Front-End
1.	It decides which points and frames are used, and in which frames a point is visible – in particular, this includes outlier removal and occlusion detection. 
2.	It provides initializations for new parameters. As a rule of thumb, a linearization of the image is only valid in a 1-2 pixel radius. 
3.	It decides when a point / frame should be marginalized. 

#### 3.1	Frame Management
1.	Our method always keeps a window of up to 7 active keyframes. Every new frame is initially tracked with respect to these reference frames (Step 1). It is then either discarded or used to create a new keyframe (Step 2). Once a new keyframe – and respective new points – are created, the total photometric error is optimized. Afterwards, we marginalize one or more frames (Step 3).
2.	(1) Initial frame tracking. (2) Keyframe creation. (3) Keyframe marginalization. 

#### 3.2	Point Management
1.	Most existing direct methods focus on utilizing as much image data as possible. To achieve this in real time, we heavily sub-sample data to allow processing it in real time in a joint optimization framework. 
2.	In contrast to indirect methods, our direct framework allows to sample from across all available data, including weakly textured or repetitive regions and edges, which does provide a real benefit.  
3.	(1) Candidate point selection. (2) Candidate point tracking. (3) Candidate point activation. 
4.	Outlier and Occlusion Detection: Since the available image data generally contains much more information than can be used in real time, we attempt to identify and remove potential outliers as early as possible. (1) When searching along the epipolar line during candidate tracking, points for which the minimum is not sufficiently distinct are permanently discarded, greatly reducing the number of false matches in repetitive areas. (2) Point observations for which the photometric error surpasses a threshold are removed. The threshold is continuously adapted with respect to the median residual in the respective frame. 

### 4	Results
1.	The TUM monoVO dataset: provides 50 photometrically calibrated sequences. Since the dataset only provides loop-closure-ground-truth (allowing to evaluate tracking accuracy via the accumulated drift after a large loop), we evaluate using the alignment error.
2.	The EuRoC MAV dataset: contains 11 stereo-inertial sequences. For this dataset, no photometric calibration or exposure times are available, hence we omit photometric image correction and set some parameters. We evaluate in terms of the absolute trajectory error. For this dataset we crop the beginning of each sequence since they contain very shaky motion meant to initialize the IMU biases – we only use the parts of the sequence where the MAV is in the air. 
3.	The ICL-NUIM dataset: contains 8 ray-traced sequences. For this dataset, photometric image correction is not required, and all exposure times can be set. Again, we evaluate in terms of the absolute trajectory error. 
4.	Methodology: We aim at an evaluation as comprehensive as possible given the available data, and thus run all sequences both forwards and backwards, 5 times each (to account for non-deterministic behavior). For the EuRoC MAV dataset we further run both the left and the right video separately. 

#### 4.1	Quantative Comparison 
1.	The direct, sparse approach outperforms ORB-SLAM in accuracy and robustness both on the TUM-monoVO dataset, as well as the synthetic ICL_NUIM dataset. 
2.	On the EuRoC MAV dataset, ORB-SLAM achieves a better accuracy (but lower robustness). This is due to two major reason: (1) there is no photometric calibration available, and (2) the sequences contain many small loops or segments where the quadcopter “back-tracks” the way it came, allowing ORB-SLAM’s local mapping component to implicitly close many small and some large loops, whereas our visual odometry formulation permanently marginalizes all points and frames that leave the field of view. 

#### 4.2	Parameter Studies
1.	Photometric calibration. 
2.	Amount of Data.
3.	Selection of Data.
4.	Number of keyframes.

#### 4.3	Geometric vs. Photometric Noise Study
#### 4.4	Qualitative Results

### 5	Conclusion 
1.	DSO combines the benefits of direct methods (seamless ability to use & reconstruct all points instead of only corners) with the flexibility of sparse approaches (efficient, joint optimization of all model parameters). 
2.	This is possible in real time by omitting the geometric prior used by other direct methods, and instead evaluating the photometric error for each point over a small neighborhood of pixels, to well-constrain the overall problem.
3.	We incorporate full photometric calibration, completing the intrinsic camera model that traditionally only reflects the geometric component of the image formation process.  



## [LDSO: Direct Sparse Odometry with Loop Closure][paper-ldso]


### 0	Abstract
In this paper we present an extension of Direct Sparse Odometry (DSO) to a monocular visual SLAM system with loop closure detection and pose-graph optimization (LDSO). As a direct technique, DSO can utilize any image pixel with sufficient intensity gradient, which makes it robust even in featureless areas. LDSO retains this robustness, while at the same time ensuring repeatability of some of these points by favoring corner features in the tracking frontend. This repeatability allows to reliably detect loop closure candidates with a conventional feature-based bag-of-words (BoW) approach. Loop closure candidates are verified geometrically and Sim(3) relative pose constraints are estimated by jointly minimizing 2D and 3D geometric error terms. These constraints are fused with a co-visibility graph of relative poses extracted from DSO's sliding window optimization. Our evaluation on publicly available datasets demonstrates that the modified point selection strategy retains the tracking accuracy and robustness, and the integrated pose-graph optimization significantly reduces the accumulated rotation-, translation- and scale-drift, resulting in an overall performance comparable to state-of-the-art feature-based systems, even without global bundle adjustment.

### 1	Introduction
1.	Visual SLAM has been very popular, in part because cameras are readily available in consumer products and passively acquire rich information about the environment. In particular, in this work we focus on the monocular case of tracking a single gray-scale camera.
2.	Typically, a visual SLAM system consists of a camera tracking frontend, and a backend that creates and maintains a map of keyframes, and reduces global drift by loop closure detection and map optimization.
3.	There are several open challenges in adapting a direct, sliding-window, marginalizing odometry system like DSO to reuse existing information from a map. 
-	In order to evaluate the photometric error, images of past keyframes would have to be kept in memory, and when incorporating measurements from previous keyframes, it is challenging to ensure estimator consistency, since information from these keyframes that is already contained in the marginalization prior should not be reused. 
4.	We adapt DSO as our SLAM frontend to estimate visual odometry with local consistency and correct its drift with loop closure detection and pose graph optimization in the backend. Note that DSO itself consists also of a camera-tracking frontend and a backend that optimizes keyframes and point depths. 
5.	VO approaches can be divided into two categories: indirect (feature-based) methods that minimize the reprojection error with fixed, previously estimated correspondences between repeatable discrete feature points, and direct methods that jointly estimate motion and correspondences by minimizing the photometric error in direct image alignment. 
6.	Recent advances in direct VO have shown better accuracy and robustness. The robustness in the direct approach comes from the joint estimation of motion and correspondences as well as the ability to also use non-corner pixels, corresponding to edges, or even smooth image regions (as long as there is sufficient image gradient).
7.	However, both indirect and direct VO suffers from the accumulated drift in the unobservable degrees-of-freedom, which are global translation, rotation and scale in the monocular case.
8.	We propose instead to gear point selection towards repeatable features and use geometric techniques to estimate constraints. Contributions are:
-	We adapt DSO’s point selection strategy to favor repeatable corner features, while retaining its robustness against feature- poor environments. The selected corner features are then used for loop closure detection with conventional BoW.
-	We utilize the depth estimates of matched feature points to compute Sim(3) pose constraints with a combination of pose-only bundle adjustment and point cloud alignment, and – in parallel to the odometry frontend – fuse them with a co-visibility graph of relative poses extracted from DSO’s sliding window optimization. 
-	The point selection retains the tracking frontend’s accuracy and robustness, and the pose graph optimization significantly reduces the odometry’s drift and results in overall performance comparable state-of-the-art feature-based methods, even without global bundle adjustment. 

### 2	Related Work
1.	ScaViSLAM suggested to mix local bundle adjustment with Sim(3) pose-graph optimization. 
2.	ORB-SLAM features multiple levels of map-optimization, starting from local bundle-adjustment after keyframe insertion, global pose-graph optimization after loop closures detected with BoW, and finally (expensive) global bundle adjustment.
3.	While loop-closure detection and pose graph optimization are similar in LDSO, we only need to compute feature descriptors for keyframes. 

### 3	Loop Closing in DSO

#### 3.1	Framework
1.	As a new frame arrives, DSO estimates its initial pose using direct image alignment by projecting all the active 3D points in the current window into this frame. If required, this frame thereafter will be added into the local windowed bundle adjustment. 
2.	The sliding window naturally forms a co-visibility graph like in ORB-SLAM, but the co-visible information is never used outside the local window, as old or redundant keyframes and points are marginalized out. 
3.	A global optimization pipeline is needed in order to close long-term loops for DSO. Ideally global bundle adjustment using photometric error should be used, which nicely would match the original formulation of DSO. However, in that case all the images would need to be saved, since the photometric error is computed on images. Moreover, nowadays it is still impractical to perform global photometric bundle adjustment for the amount of points selected by DSO. 
4.	To avoid these problems, we turn to the idea of using pose graph optimization, which leaves us several other challenges: 
(i)	How to combine the result of global pose graph optimization with that of the windowed optimization? On step further, how to set up the pose graph constraints using the information in the sliding window, considering that pose graph optimization minimize Sim(3) geometry error between keyframes while in the sliding window we minimize the photometric error?
(ii)	How to propose loop candidates? While the mainstream of loop detection is based on image descriptors, shall we simply add another thread to perform those feature related computations?
(iii)	Once loop candidates are proposed, we need to compute their relative Sim(3) transformation. In a direct image alignment approach, we need to set an initial guess on the relative pose to start the Gauss-Newton or the Levenberg-Marquardt iterations, which is challenging in this case as the relative motion may be far away from identity.
5.	We design our loop closing module as in the following figure.
![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/post-framework-of-ldso.png) 
6.	Alongside the DSO window, we add a global pose graph to maintain the connectivity between keyframes. 
7.	DSO’s sliding window naturally forms a co-visibility graph where we can take the relative 3D pose transformations between the keyframes as the pairwise pose measurements. 
8.	For loop detection and validation, we rely on BoW and propose as novel way to combine ORB features with the original sampled points of DSO. In this way, if a loop candidate is proposed and validated, its Sim(3) constraint with respect to the current keyframe is calculated and added to the global pose graph, which is thereafter optimized to obtain a more accurate long-term camera pose estimation.

#### 3.2	Point Selection with Repeatable Features
1.	In DSO, point selection is still needed; one difference from indirect methods is that the repeatability of those points is not required by direct methods. 
2.	DSO uses a dynamic grid search to pick enough pixels even in weakly textured environments. We modify this strategy to make it more sensitive to corners. More specifically, we still pick a given number of pixels (by default 2000 in DSO), in which part of them are corners (detected by using the easy-to-compute Shi-Tomasi score), while the others are still selected using the method proposed for DSO. 
3.	Keeping the number for corners small, we compute their ORB descriptors and pack them into BoW. 
4.	The VO frontend uses both the corners and the non-corners for camera tracking, keeping therefore the extra overhead for feature extraction of the loop closing thread to a minimum.
5.	Pixels picked by DSO have little repeatability and therefore it is hard to seek image matchings using those points for loop closure. In LDSO we use both corners and other pixels with high gradients, where the corners are used both for building BoW models and for tracking, while the non-corners are only used for tracking. 

#### 3.3	Loop Candidates Proposal and Checking
1.	As we compute ORB descriptors for each keyframe, a BoW database is built using DBoW3. 
2.	Loop candidates are proposed for the current keyframe by querying the database and we only pick those that are outside the current window (i.e., marginalized keyframes).
3.	For each candidate we try to match its ORB features to those of the current keyframe, and then perform RANSAC PnP to compute an initial guess of the SE(3) transformation. Afterwards we optimize a Sim(3) transformation using Gauss-Newton method by minimizing the 3D and 2D geometric constraints. 
4.	In practice the scale can only be estimated by the 3D part, but without the 2D reprojection error, the rotation and translation estimate will be inaccurate if the estimated depth values are noisy. 

#### 3.4	Sliding Window and Sim(3) Pose Graph
1.	In this section we explain how to fuse the estimations of the sliding window and the global pose graph. 
2.	Since our loop closing approach computes relative pose constraints between the loop candidate and the current frame, we also approximate the constraints inside the marginalization window with pairwise relative pose observations. Specifically, we compute those observations from the frontend’s current global pose estimates. 
3.	Since we do not want to disturb the local windowed optimization (it contains absolute pose information), in pose graph optimization we will fix the current frame’s pose estimation. Therefore, the pose graph optimization will tend to modify the global poses of the old part of the trajectory. 
4.	Besides, the global poses of the keyframes in the current window are not updated after the pose graph optimization, to further make sure that the local windowed bundle adjustment is not influenced by the global optimization. Our implementation is based on g2o. 

### 4	Evaluation
#### 4.1	The TUM-Mono Dataset 
1.	The camera always returns to the starting point in all sequences, making this dataset very suitable for evaluating accumulated drifts of VO systems. For this reason, we disable the loop closure functionality of our method on this dataset, to first evaluate the accuracy of our method with the modified point selection strategy.
2.	We evaluate three different point selection strategies: (1) random point selection; (2) the original method of DSO and (3) our method. 
3.	For each strategy we run 10 times forward and 10 times backward on each sequence to account for the nondeterministic behavior. 
4.	We compute the accumulated translational, rotational and scale drifts in the keyframe trajectories.
5.	We see that our integration of corner features into DSO does not reduce the VO accuracy of the original system. 
6.	Although random picking makes the tracking fail more frequently, it seems it does not increase the errors on those successfully tracked sequences like s01 to s10. 

#### 4.2	The EuRoC MAV Dataset
1.	We compare LDSO with DSO and ORB on this dataset by evaluating their root-mean-square error (RMSE) using their monocular settings. 
2.	ORB-SLAM2 performs quite well on this dataset and it only fails consistently on sequence V2-03 when running forward. 
3.	DSO and LDSO both fail on sequences V2-03, but on most of the other sequences LDSO significantly improves the camera tracking accuracy. 
4.	From the plot we can see that ORB-SLAM2 is more accurate, whereas LDSO is more robust on this dataset. 

#### 4.3	The KITTI Odometry Dataset
1.	DSO and ORB-SLAM (the VO component only) suffer severe accumulated drift which makes them not usable for such large-scale scenarios. While the natural way to resolve this problem is to integrate other sensors like IMU or to use stereo cameras which has been proved quite successful. 
2.	We compare LDSO with DSO and ORB-SLAM2 and show the Absolute Trajectory Errors (ATEs). 
3.	The ATEs are computed by performing Sim(3) alignment to the groundtruth. 
4.	Not surprisingly on sequences with loops (seq. 00, 05, 07), LDSO improves the performance of DSO a lot. 
5.	Besides, our method achieves comparable accuracy to ORB-SLAM2, which has a global bundle adjustment in the loop closing thread and we only use pose graph optimization. 

#### 4.4	Runtime Evaluation
1.	Loop closure only occurs very occasionally, and the pose graph is running in a single thread, thus they do not affect much the computation time of the main thread.
2.	What we change in the main thread is adding an extra feature extraction and descriptor computation step. But unlike the feature-based approaches, they are not performed for every frame but only for keyframes. 
3.	The point selection in LDSO takes slightly more time than that in DSO due to the feature and descriptor extraction. It is worth noting that the values are calculated over keyframes, thus the runtime impact will be further moderated when averaging over all frames. 

### 5	Conclusion 
1.	In this paper we propose an approach to integrate loop closure and global map optimization into the fully direct VO system DSO. 
2.	DSO’s original point selection is adapted to include repeatable features. For those we compute ORB descriptors and build BoW models for loop closure detection. 
3.	Advices:
-	A photometric bundle adjustment layer might increase the global map accuracy. 
-	In order to ensure long-term operation, map maintenance strategies such as keyframe culling and removal of redundant 3D points may be employed. 
-	Combining the information from 3D points of neighboring keyframes after loop closure may help to further increase the accuracy of the reconstructed geometry. 



[paper-dso]: https://vision.in.tum.de/research/vslam/dso
[paper-ldso]: https://vision.in.tum.de/research/vslam/ldso
