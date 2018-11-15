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



## [Part II][paper-part-2]

$$\quad$$ Focus on three key areas: computational complexity, data association, and environment representation.

### 1. Computational Complexity

$$\quad$$ The direct approach to reducing computational complexity involves exploting the structure of the SLAM problem in re-formulating 
the essential time- and observation-update equations to limit required computation. 

$$\quad$$ The time-update computation can be limited using _state-augmentation_ methods. 

$$\quad$$ The observation-update computation can be limited using a _partitioned form_ of the update equations. 

$$\quad$$ Re-formulation of the standard state-space SLAM representation into information form allows _sparsification_ of the resulting information 
matrix to be exploited in reducing computation. 

$$\quad$$ _Submapping_ methods exploit the idea that a map can be broken up into regions with local coordinate systems and arranged in a hierarchical 
manner. Updates can occur in a local frame with periodic interframe updates.


#### 1.1 State Augmentation

#### 1.2 Partitioned Updates

$$\quad$$ The submap method has a number of advantages. First, the number of landmarks that must be updated at any on time is limitted to only 
those that are described in the local submap coordinate frame. Thus, the observation-rate update is independent of local estimates. The full update, 
and the propagation of local estimates, can be carried out as a background task at a much lower update rate while still permitting observation-rate 
global localization. A second advantage is that there is lower uncertainty in a locally referenced frame, so approximations due to linearization 
are reduced. Finally, submap registration can use batch-validation gating, thereby improving association robustness. 

#### 1.3 Sparsification

$$\quad$$ The key to exact sparsification of the information form of the SLAM problem is to notice that state augmentation is a sparse operation.

#### 1.4 Global Submaps

#### 1.5 Relative Submaps 

### 2. Data Association

#### 2.1. Batch Validation

#### 2.2. Appearance Signatures

#### 2.3 Multihypothesis Data Association

### 3. Environment Representation

#### 3.1 Partial Observability and Delayed Mapping

#### 3.2 Nongeometric Landmarks

#### 3.3 3D SLAM

#### 3.4 Trajectory-Oriented SLAM

#### 3.5 Embedded Auxiliary Information

#### 3.6 Dynamic Environments

### 4. SLAM: Where to Next? 

[paper-dso]: https://vision.in.tum.de/research/vslam/dso
[paper-ldso]: https://vision.in.tum.de/research/vslam/ldso
