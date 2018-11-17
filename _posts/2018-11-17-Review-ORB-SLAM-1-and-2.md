---
layout:     post
title:      "Review - ORB-SLAM 1 and 2"
date:       2018-11-17
author:     Tong
catalog: true
tags:
   - Review
---

## [ORB-SLAM: a Versatile and Accurate Monocular SLAM System][paper-orb-slam-1]

### 0	Abstract
This paper presents ORB-SLAM, a feature-based monocular simultaneous localization and mapping (SLAM) system that operates in real time, in small and large indoor and outdoor environments. The system is robust to severe motion clutter, allows wide baseline loop closing and relocalization, and includes full automatic initialization. Building on excellent algorithms of recent years, we designed from scratch a novel system that uses the same features for all SLAM tasks: tracking, mapping, relocalization, and loop closing. A survival of the fittest strategy that selects the points and keyframes of the reconstruction leads to excellent robustness and generates a compact and trackable map that only grows if the scene content changes, allowing lifelong operation. We present an exhaustive evaluation in 27 sequences from the most popular datasets. ORB-SLAM achieves unprecedented performance with respect to other state-of-the-art monocular SLAM approaches. For the benefit of the community, we make the source code public.

### 1	Introduction
1.	Bundle adjustment (BA) is known to provide accurate estimates of camera localizations as well as a sparse geometrical reconstruction, given that a strong network of matches and good initial guesses are provided.
2.	To achieve accurate results at non-prohibitive computational cost, a real time SLAM algorithm has to provide BA with:
-	Corresponding observations of scene features (map points) among a subset of selected frames (keyframes).
-	As complexity grows with the number of keyframes, their selection should avoid unnecessary redundancy.
-	A strong network configuration of keyframes and points to produce accurate results, that is, a well spread set of keyframes observing point with significant parallax and with plenty of loop closure matches. 
-	An initial estimation of the keyframe poses and point locations for the non-linear optimization. 
-	A local map in exploration where optimization is focused to achieve scalability.
-	The ability to perform fast global optimizations (e.g. pose graph) to close loops in real-time.
3.	We build on the main ideas of PTAM, the place recognition work, the scale-aware loop closing, and the use of Covisibility information for large scale operation. 
4.	Main contributions are:
-	Use the same features for all tasks: tracking, mapping, relocalization and loop closing. This makes our system more efficient, simple and reliable. We use ORB features which allow real-time performance without GPUs, providing good invariance to changes in viewpoint and illumination. 
-	Real time operation in large environments. Thanks to the use of a covisibility graph, tracking and mapping is focused in a local covisible area, independent of global map size.
-	Real time loop closing based on the optimization of a pose graph that we call the Essential Graph. It is built from a spanning tree maintained by the system, loop closure links and strong edges from the covisibility graph. 
-	Real time camera relocalization with significant invariance to viewpoint and illumination. This allows recovery from tracking failure and also enhances map reuse. 
-	A new automatic and robust initialization procedure based on model selection that permits to create an initial map of planar and non-planar scenes. 
-	A survival of the fittest approach to map point and keyframe selection that is generous in the spawning but very restrictive in the culling. This policy improves tracking robustness, and enhances lifelong operation because redundant keyframes are discarded. 
5.	In contrast to a preliminary version, we add the initialization methods, the Essential Graph, and perfect all methods involved. 
 
### 2	Related Work

#### 2.1	Place Recognition
1.	We proposed a bag of words place recognizer built on DBoW2 with ORB. ORB are binary features invariant to rotation and scale (in a certain range), resulting in a very fast recognizer with good invariance to viewpoint.
2.	In this work we use an improved version of that place recognizer, using covisibility information and returning several hypotheses when querying the database instead of just the best match. 

#### 2.2	Map initialization 
1.	Monocular SLAM requires a procedure to create an initial map because depth cannot be recovered from a single image.
2.	We present a new automatic approach based on model selection between a homography for planar scenes and a fundamental matrix for non-planar scenes. 
3.	Under a similar rationale we have developed a heuristic initialization algorithm that takes into account the risk of selecting a fundamental matrix in close to degenerate cases (i.e. planar, nearly planar, and low parallax), favoring the selection of the homography. 
4.	In the planar case, for the sake of safe operation, we refrain from initializing if the solution has a twofold ambiguity, as a corrupted solution could be selected. We delay the initialization until the method produces a unique solution with significant parallax. 

#### 2.3	Monocular SLAM
1.	Our survival of the fittest strategy achieves unprecedented robustness in difficult scenarios by inserting keyframes as quickly as possible, and removing later the redundant ones, to avoid the extra cost.

### 3	System Overview
1.	ORB-SLAM. 
![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/post-orb-slam-1.png)
 
#### 3.1	Feature Choice
1.	One of the main ideas in our system is that the same features used by the mapping and tracking are used for place recognition to perform frame-rate relocalization and loop detection. This makes our system efficient and avoids the need to interpolate the depth of the recognition features from near SLAM features. 
2.	We chose ORB, which are oriented multi-scale FAST corners with a 256 bits descriptor associated. They are extremely fast to compute and match, while they have good invariance to viewpoint. This allows to match them from wide baselines, boosting the accuracy of BA. 
 
#### 3.2	Three Threads: Tracking, Local Mapping and Loop Closing 
1.	The tracking is in charge of localizing the camera with every frame and deciding when to insert a new keyframe. We perform first an initial feature matching with the previous frame and optimize the pose using motion-only BA. If the tracking is lost (e.g. due to occlusions or abrupt movements), the place recognition module is used to perform a global relocalization. Once there is an initial estimation of the camera pose and feature matchings, a local visible map is retrieved using the covisibility graph of keyframes that is maintained by the system. Then matches with the local map points are searched by reprojection, and camera pose is optimized again with all matches. Finally, the tracking thread decides if a new keyframe is inserted. 
2.	The local mapping processes new keyframes and performs local BA to achieve an optimal reconstruction in the surroundings of the camera pose. New correspondences for unmatched ORB in the new keyframe are searched in connected keyframes in the covisibility graph to triangulate new points. Some time after creation, based on the information gathered during the tracking, an exigent point culling policy is applied in order to retain only high quality point. The local mapping is also in charge of culling redundant keyframes. 
3.	The loop closing searches for loops with every new keyframe. If a loop is detected, we compute a similarity transformation that informs about the drift accumulated in the loop. Then both sides of the loop are aligned, and duplicated points are fused. Finally, a pose graph optimization over similarity constraints is performed to achieve global consistency. The main novelty is that we perform the optimization over the Essential Graph. 
4.	We use the Levenberg-Marquardt algorithm implemented in g2o to carry out all optimizations. 

#### 3.3	Map Points, Key Frames and their Selection
1.	Each map point p stores:
-	Its 3D position X in the world coordinate system.
-	The viewing direction n, which is the mean unit vector of all its viewing directions (the rays that join the point with optical center of the keyframes that observe it).
-	A representative ORB descriptor D, which is the associated ORB descriptor whose hamming distance is minimum with respect to all other associate descriptors in the keyframes in which the point is observed. 
-	The maximum dmax and minimum distances dmin at which the point can be observed, according the scale invariance limits of the ORB features. 
2.	Each keyframe K stores:
-	The camera pose T, which is a rigid body transformation that transforms points from the world to the camera coordinate system. 
-	The camera intrinsics, including focal length and principal point. 
-	All the ORB features extracted in the frame, associated or not to a map point, whose coordinates are undistorted if a distortion model is provided. 
3.	Map points and keyframes are created with a generous policy, while a later very exigent culling mechanism is in charge of detecting redundant keyframes and wrongly matched or not trackable map points. This permits a flexible map expansion during exploration, which is boost tracking robustness under hard conditions (e.g. rotations, fast movements), while its size is bounded in continual revisits to the same environment, i.e. lifelong operation. 

#### 3.4	Covisibility Graph and Essential Graph
1.	Covisibility information between keyframes is very useful in several tasks of our system, and is represented as an undirected weighted graph. Each node is a keyframe and an edge between two keyframes exists if they share observations of the same map points (at leas 15), being the weight of the edge the number of common map points.
2.	In order to correct a loop, we perform a pose graph optimization that distributes the loop closing error along the graph. In order not to include all the edges provided by covisibility graph, which can be very dense, we propose to build an Essential Graph that retains all the nodes (keyframes), but less edges. 
3.	The system builds incrementally a spanning tree from the initial keyframe, which provided a connected subgraph of the covisibility graph with minimal number of edges. When a new keyframe is inserted, it is included in the tree linked to the keyframe which shares most point observations, and when a new keyframe is erased by the culling policy, the system updates the links affected by the keyframe.
4.	The Essential Graph contains the spanning tree, the subset of edges from the covisibility graph with high covisibility, and the loop closure edges, resulting in a strong network of cameras. 

#### 3.5	Bags of Words Place Recognition
1.	The system has embedded a bag of words place recognition module, based on DBoW2, to perform loop detection and relocalization. 
2.	Visual words are just a discretization of the descriptor space, which is known as visual vocabulary. The vocabulary is created offline with the ORB descriptors extracted from a large set of images. If the images are general enough, the same vocabulary can be used for different environments.
3.	The system builds incrementally a database that contain an invert index, in which keyframes it has been seen, so that querying the database can be done very efficiently. The database is also updated when a keyframe is deleted by the culling procedure. 
4.	Because there exists visual overlap between keyframes, when querying the database there will not exist a unique keyframe with a high score. The original DBoW2 took this overlapping into account, adding up the score of images that are close in time. This has the limitation of not including keyframes viewing the same place but inserted at a different time. Instead We group those keyframes that are connected in the covisibility graph. In addition, our dataset returns all keyframe matches whose scores are higher than the 75% of the best score. 
5.	When we want to compute the correspondences between two sets of ORB features, we can constraint the brute force matching only to those features that belong to the same node in the vocabulary tree at a certain level (we select the second out of six), speeding up the search. We use this trick when searching matches for triangulating new points, and at loop detection and relocalization.
6.	We also refine the correspondences with an orientation consistency test that discards outliers ensuring a coherent rotation for all correspondences. 

### 4	Automatic Map Initialization 
1.	The goal of the map initialization is to compute the relative pose between two frames to triangulate an initial set of map points. 
2.	We propose to compute in parallel two geometrical models, a homography assuming a planar scene and a fundamental matrix assuming a non-planar scene. We then use a heuristic to select a model and try to recover the relative pose with a specific method for the selected model. 
3.	Our method only initializes when it is certain that the two-view configuration is safe, detecting low-parallax cases and the well-known twofold planar ambiguity, avoiding initializing a corrupted map. 
4.	The steps of our algorithm are:
1)	Find initial correspondences
2)	Parallel computation of the two models:
3)	Model selection
4)	Motion and Structure from Motion recovery
5)	Bundle adjustment 

### 5	Tracking 
1.	The camera pose optimizations consist in motion-only BA.
#### 5.1 ORB Extraction
1.	We extract FAST corners at 8 scale levels with a scale factor of 1.2.
2.	For image resolutions from 512 x 384 to 752 x 480 pixels, we extract 1000 corners.
3.	For 1241 x 376 in the KITTI dataset, we extract 2000 corners.
4.	In order to ensure a homogeneous distribution, we divide each scale level in a grid, trying to extract at least 5 corners per cell. Then we detect corners in each cell, adapting the detector threshold if not enough corners are found. The amount of corners retained per cell is also adapted if some cells contain no corners (texture-less or low contrast). The orientation and ORB descriptors are then computed on the retained FAST corners. 

#### 5.2 Initial Pose Estimation from Previous Frame
1.	If tracking was successful for last frame, we use a constant velocity motion model to predict the camera pose and perform a guided search of the map points observed in the last frame. If not enough matches were found (i.e. motion model is clearly violated), we use a wider search of the map points around their position in the last frame. The pose is then optimized with the found correspondences.

#### 5.3 Initial Pose Estimation from via Global Relocalization
1.	If the tracking is lost, we convert the frame into bag of words and query the recognition database for keyframe candidates for global relocalization. 
2.	We compute correspondences with ORB associated to map points in each keyframe. We then perform alternatively RANSAC iterations of each keyframe and try to find a camera pose using the PnP algorithm. If we find a camera pose with enough inliers, we optimize the pose and perform a guided search of more matches with the map points of the candidate keyframe. Finally, the camera pose is again optimized, and if supported with enough inliers, tracking procedure continues. 

#### 5.4 Track Local Map
1.	Once we have an estimation of the camera pose and an initial set of feature matches, we can project the map into the frame and search more map point correspondences. 
2.	To bound the complexity in large maps, we only project a local map. This local map contains the set of keyframe K1, that share map points with the current frame, and a set K2 with neighbors to the keyframe K1 in the in the covisibility graph. 
3.	The local map also has a reference frame Kref which shares most map points with current frame. 
4.	Each map point seen in K1 and K2 is searched in the current frame as follows:
1)	Compute the map point project x in the current frame. Discard it if it lays out of the image bounds. 
2)	Compute the angle between the current viewing ray v and the map point mean viewing direction n. Discard if vn < cos(60°).
3)	Compute the distance d from map point to camera center. Discard it if it is out of the scale invariance region of the map point [dmin, dmax].
4)	Compute the scale in the frame by the ratio d/dmin.
5)	Compare the representative descriptor D of the map point with the still unmatched ORB features in the frame, at the predicted scale, and near x, and associate the map point with the best match. 
5.	The camera pose is finally optimized with all the map points found in the frame.  

#### 5.5 New Keyframe Decision
1.	As there is a mechanism in the local mapping to cull redundant keyframes, we will try to insert keyframes as fast as possible, because that makes the tracking more robust to challenging camera movements, typically rotations. 
2.	To insert a new keyframe, all the following conditions must be met:
-	More than 20 frames must have passed from the last global relocalization.
-	Local mapping is idle, or more than 20 frames have passed from last keyframe insertion.
-	Current frame tracks at least 50 points. 
-	Current frame tracks less than 90% points than Kref. 
3.	We impose a minimum visual change (condition 4). Condition 1 ensures a good relocalization and condition 3 a good tracking. If a keyframe is inserted when the local mapping is busy, a signal is sent to stop local bundle adjustment, so that I can process as soon as possible the new keyframe.

### 6	Local Mapping

#### 6.1 Keyframe Insertion
1.	At first, we update the covisibility graph, adding a new node for Ki and updating the edges resulting from the shared map points with other keyframes. 
2.	We then update the spanning tree linking Ki with the keyframe with most points in common. 
3.	We then compute the bag of words representation of the keyframe, that will help in the data association for triangulating new points.  

#### 6.2 Recent Map Points Culling
1.	A point must fulfill these two conditions:
-	The tracking must find the point in more than the 25% of the frames in which it is predicted to be visible.
-	If more than one keyframe has passed from map point creation, it must be observed from at least three keyframes. 
2.	Once a map point has passed the test, it can be only removed if at any time it is observed from less than three keyframes. This can happen when keyframes are culled and when local bundle adjustment discards outlier observations. This policy makes our map contain very few outliers. 

#### 6.3 New Map Point Creation 
1.	New map points are created by triangulating ORB from connected keyframe Kc in the covisibility graph. 
2.	For each unmatched ORB in Ki, we search a match with other unmatched point in other keyframe. 

#### 6.4 Local Bundle Adjustment
1.	The local BA optimizes the currently processed keyframe Ki, all the keyframe connected to it in the covisibility graph Kc, and all the map points seen by those keyframes. 
2.	All other keyframes that see those points but are not connected to the currently processed keyframe are included in the optimization but remain fixed. 
3.	Observations that are marked as outliers are discarded at the middle and at the end of the optimization. 

#### 6.5 Local Keyframe Culling
1.	In order to maintain a compact reconstruction, the local mapping tries to detect redundant keyframes and delete them. 
2.	We discard all the keyframe Kc whose 90% of the map points have been seen in at least other three keyframes in the same or finer scale. 
3.	The scale condition ensures that map points maintain keyframes from which they are measured with most accuracy. 

### 7	Loop Closing
1.	The loop closing thread takes the last keyframe Ki processed by the local mapping, and tries to detect and close loops.

#### 7.1 Loop Candidates Detection
1.	At first, we compute the similarity between the bag of words vector of Ki and all its neighbors in the covisibility graph and retain the lowest score smin. 
2.	Then we query the recognition database and discard all those keyframes whose score is lower than smin. 
3.	In addition, all those keyframes directly connected to Ki are discarded from the results. 
4.	To accept a loop candidate, we must detect consecutively three loop candidates that are consistent (keyframes connected in the covisibility graph). 

#### 7.2 Compute the Similarity Transformation
1.	In monocular SLAM, there are 7 degrees of freedom in which the map can drift, three translations, three rotations and a scale factor. 
2.	Therefore, to close a loop we need to compute a similarity transformation from the current frame Ki to the loop keyframe Kl that informs us about the error accumulated in the loop. The computation of this similarity will serve also as geometrical validation of the loop.

#### 7.3 Loop Fusion
1.	The first step in the loop correctio is to fuse duplicated map points and insert new edges in the covisibility graph that will attach the loop closure. 
2.	At first the current keyframe pose T is corrected with the similarity transformation S and this correction is propagated to all the neighbors of K, concatenating transformations, so that both sides of the loop get aligned. 
3.	All map points seen by the loop keyframe and its neighbors are projected into K and its neighbors and matches are searched in a narrow area around the projection. 
4.	All those map points matched and those that were inliers in the computation of S are fused. 
5.	All keyframes involved in the fusion will update their edges in the covisibility graph effectively creating edges that attach the loop closure. 

#### 7.4 Essential Graph Optimization
1.	To effectively close the loop, we perform a pose graph optimization over the Essential Graph, that distributes the loop closing error along the graph.
2.	The optimization is performed over similarity transformations to correct the scale drift.  
3.	After the optimization, each map point is transformed according to the correction of one of the keyframes that observes it.

### 8	Experiments
#### 8.1 System Performance in the NewCollege Dataset

#### 8.2 Localization Accuracy in the TUM RGB-D Benchmark

#### 8.3 Relocalization in the TUM RGB-D Benchmark

#### 8.4 Lifelong Experiment in the TUM RGB-D Benchmark

#### 8.5 Large Scale and Large Loop Closing in the KITTI Dataset

### 9	Conclusions and Discussion
#### 9.1 Conclusions
1.	Our novel policy to spawn and cull keyframes, permits to create keyframes every few frames, which are eventually removed when considered redundant. 
2.	The flexible map expansion is really useful in poorly conditioned exploration trajectories, i.e., close to pure rotations or fast movements, the map only grows if the visual content of the scene changes, storing a history of its different visual appearance. 

#### 9.2 Sparse/Feature-based vs. Dense/Direct Methods
1.	Direct approaches do not need feature extraction and thus avoid the corresponding artifacts. They are more robust to blur, low-texture environments and high-frequency texture like asphalt. 
2.	Direct methods have their own limitations:
-	These methods assume a surface reflectance model that in real scenes produces its own artifacts. 
-	The photometric consistency limits the baseline of the matches, typically narrower than those that features allow. This has a great impact in reconstruction accuracy, which requires wide baseline observations to reduce depth uncertainty. 
-	Direct methods, if not correctly modeled, are quite affected by rolling-shutter, auto-gain and auto-exposure artifacts. 
3.	Feature-based methods are able to match features with a wide baseline, thanks to their good invariance to viewpoint and illumination changes.
4.	Bundle adjustment jointly optimizes camera poses and points over sensor measurements. 

#### 9.3 Future Work
1.	The accuracy of our system can still be improved incorporating points at infinity in the tracking. These points, which are not seen with sufficient parallax and our system does not include in the map, are very informative of the rotation of the camera. 
2.	Another open way is to upgrade the sparse map if our system to a denser and more useful reconstruction. 



## [ORB-SLAM2: An Open-Source SLAM System for Monocular, Stereo, and RGB-D Cameras][paper-orb-slam-2]

### 0	Abstract
We present ORB-SLAM2, a complete simultaneous localization and mapping (SLAM) system for monocular, stereo and RGB-D cameras, including map reuse, loop closing, and relocalization capabilities. The system works in real time on standard central processing units in a wide variety of environments from small hand-held indoors sequences, to drones flying in industrial environments and cars driving around a city. Our back-end, based on bundle adjustment with monocular and stereo observations, allows for accurate trajectory estimation with metric scale. Our system includes a lightweight localization mode that leverages visual odometry tracks for unmapped regions and matches with map points that allow for zero-drift localization. The evaluation on 29 popular public sequences shows that our method achieves state-of-the-art accuracy, being in most cases the most accurate SLAM solution. We publish the source code, not only for the benefit of the SLAM community, but with the aim of being an out-of-the-box SLAM solution for researchers in other fields.

### 1	Introduction
1.	Problems of monocular SLAM:
-	As depth is not observable from just one camera, the scale of the map and estimated trajectory is unknown. 
-	The system bootstrapping requires multiview or filtering techniques to produce an initial map as it cannot be triangulated from the very first frame. 
-	Monocular SLAM suffers from scale drift and may fail if performing pure rotations in exploration. 
2.	 Contributions:
-	The first open-source SLAM system for monocular, stereo, and RGB-D cameras, including loop closing, relocalization, and map reuse;
-	Our RGB-D results show that by using bundle adjustment (BA), we achieve more accuracy than state-of-the-art methods based on iterative closet point (ICP) or photometric and depth error minimization;
-	By using close and far stereo points and monocular observations, our stereo results are more accurate than the state-of-the-art direct stereo SLAM;
-	A lightweight localization mode that can effectively reuse the map with mapping disables.

### 2	Related Work

#### 2.1	Stereo SLAM

#### 2.2	RGB-D SLAM 

### 3	ORB-SLAM2
1.	ORB-SLAM2. 
![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/post-orb-slam-2.png) 

2.	Threads:
1)	The tracking to localize the camera with every frame by finding feature matches to the local map and minimizing the reprojection error applying motion-only BA;
2)	The local mapping to manage the local map and optimize it, performing local BA;
3)	The loop closing to detect large loops and correct the accumulated drift by performing a pose-graph optimization. This thread launches a fourth thread to perform full BA after pose -graph optimization, to compute the optimal structure and motion solution. 

#### 3.1	Monocular, Close Stereo, and Far Stereo Keypoints

#### 3.2	System Bootstrapping

#### 3.3	Bundle Adjustment with Monocular and Stereo Constraints
1.	Our System performs BA to optimize the camera pose in the tracking thread (motion-only BA), to optimize a local window of keyframes and points in the local mapping thread (local BA), and after a loop closure to optimize all keyframes and points (full BA). We use the Levenberg-Marquardt method implemented in g2o.
2.	Motion-only BA optimizes the camera orientation R and position t, minimizing the reprojection error between matched 3-D points in world coordinates and keypoints. 
3.	Local BA optimizes a set of covisible keyframes and all points seen in those keyframes. All other keyframes observing points contribute to the cost function but remain fixed in the optimization.
4.	Full BA is the specific case of local BA, where all keyframes and points in the map are optimized, except the origin keyframe that is fixed to eliminate the gauge freedom.  

#### 3.4	Loop Closing and Full BA
1.	Loop closing is performed in two steps, first, a loop has to be detected and validated, and second, the loop is corrected optimizing a pose graph. 
2.	In ORB-SLAM2, we have incorporated a full BA optimization after the pose graph to achieve the optimal solution. This optimization might be very costly, and therefore, we perform it in a separate thread, allowing the system to continue creating map and detecting loops. However, this brings the further challenge of merging the bundle adjustment output with the current state of the map.
3.	If a new loop is detected while the optimization is running, we abort the optimization and proceed to close the loop, which will launch the full BA optimization again. 
4.	When the full BA finishes, we need to merge the updated subset of keyframes and points optimized by the full BA, with the nonupdated keyframes and points that where inserted while the optimization was running. This is done by propagating the correction of updated keyframes (i.e., the transformation from the nonoptimized to the optimized pose) to nonupdated keyframes through the spanning tree. Nonupdated points are transformed according to the correction applied to their reference keyframe. 

#### 3.5	Keyframe Insertion 

#### 3.6	Localization Mode
1.	We incorporate a localization mode, which can be useful for lightweight long-term localization in well-mapped areas, as long as there are not significant changes in the environment. 
2.	In this mode, the local mapping and loop closing threads are deactivated and the camera is continuously localized by the tracking using relocalization if needed. 
3.	In this mode, the tracking leverage visual odometry matches between ORB in the current frame and 3-D points created in the previous frame from the stereo/depth information. These matches make the localization robust to unmapped regions, but drift can be accumulated. 
4.	Map point matches ensure drift-free localization to the existing map. 

### 4	Evaluation 
#### 4.1	KITTI Dataset
#### 4.2	EuRoC Dataset
#### 4.3	TUM RGB-D Dataset
#### 4.4	Timing Results

### 5	Conclusion 
1.	The proposed localization mode with the relocalization capability of the system yields a very robust, zero drift, and lightweight localization method for known environments. This mode can be useful for certain applications, such as tracking the user viewpoint in virtual reality in a well-mapped space.
2.	Future extensions might include, to name some examples, nonoverlapping multicamera, fisheye, or omnidirectional cameras support, large-scale dense function, cooperative mapping, or increased motion blur robustness. 




[paper-orb-slam-1]: http://webdiis.unizar.es/~raulmur/orbslam/
[paper-orb-slam-2]: http://webdiis.unizar.es/~raulmur/orbslam/
