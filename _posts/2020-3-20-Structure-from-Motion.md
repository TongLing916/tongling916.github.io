---
layout:     post
title:      "Structure from Motion"
date:       2020-3-20
author:     Tong
catalog: true
tags:
    - Reconstruction
---

### Very Large-Scale Global SfM by Distributed Motion Averaging [^Zhu18]

#### Abstract

### A survey of structure from motion [^Ozyesil17]

#### Abstract

### GSLAM: Initialization-Robust Monocular Visual SLAM via Global Structure-from-Motion [^Tang17]

#### Abstract

### Global Structure-from-Motion by Similarity Averaging [^Cui15]

#### Abstract

### Global fusion of relative motions for robust, accurate and scalable structure from motion [^Moulon13]

#### Abstract

Multi-view structure from motion (SfM) estimates the position and orientation of pictures in a common 3D coordinate frame. When views are treated incrementally, this external calibration can be subject to drift, contrary to global methods that distribute residual errors evenly. We propose a new global calibration approach based on the fusion of relative motions between image pairs. We improve an existing method for robustly computing global rotations. We present an efficient a contrario trifocal tensor estimation method, from which stable and precise translation directions can be extracted. We also define an efficient translation registration method that recovers accurate camera positions. These components are combined into an original SfM pipeline. Our experiments show that, on most datasets, it outperforms in accuracy other existing incremental and global pipelines. It also achieves strikingly good running times: it is about 20 times faster than the other global method we could compare to, and as fast as the best incremental method. More importantly, it features better scalability properties.

### Adaptive structure from motion with a contrario model estimation [^Moulon2012a]

#### Abstract

Structure from Motion (SfM) algorithms take as input multi-view stereo images (along with internal calibration information) and yield a 3D point cloud and camera orientations / poses in a common 3D coordinate system. In the case of an incremental SfM pipeline, the process requires repeated model estimations based on detected feature points: homography, fundamental and essential matrices, as well as camera poses. These estimations have a crucial impact on the quality of 3D reconstruction. We propose to improve these estimations using the _a contrario_ methodology. While SfM pipelines usually have globally-fixed thresholds to the input data and for each model estimation. Our experiments show that adaptive thresholds reach a significantly better precision. Additionally, the user is free from having to guess thresholds or to optimistically rely on default values. There are also cases where a globally-fixed threshold policy, whatever the threshold value is, cannot provide the best accuracy, contrary to an adaptive threhold policy.

### Discrete-continuous optimization for large-scale structure from motion [^Crandall11]

#### Abstract

### [Building Rome in a day](http://grail.cs.washington.edu/rome/) [^Agarwal11]

#### Abstract

We present a system that can match and reconstruct 3D scenes from extremely large collections of photographs such as those found by searching for a given city (e.g., Rome) on Internet photo sharing sites. Our system uses a collection of novel parallel distributed matching and reconstruction algorithms, designed to maximize parallelism at each stage in the pipeline and minimize serialization bottlenecks. It is designed to scale gracefully with both the size of the problem and the amount of available computation. We have experimented with a variety of alternative algorithms at each stage of the pipeline and report on which ones work best in a parallel computing environment. Our experimental results demonstrate that it is now possible to reconstruct cities consisting of 150 K images in less than a day on a cluster with 500 compute cores.

### Literature

[^Crandall11]: Crandall, David J., et al. "Discrete-continuous optimization for large-scale structure from motion." computer vision and pattern recognition (2011): 3001-3008.

[^Moulon2012a]: Moulon, Pierre, Pascal Monasse, and Renaud Marlet. "Adaptive structure from motion with a contrario model estimation." Asian Conference on Computer Vision. Springer, Berlin, Heidelberg, 2012.

[^Moulon13]: Moulon, Pierre, Pascal Monasse, and Renaud Marlet. "Global fusion of relative motions for robust, accurate and scalable structure from motion." Proceedings of the IEEE International Conference on Computer Vision. 2013.

[^Cui15]: Cui, Zhaopeng, and Ping Tan. "Global Structure-from-Motion by Similarity Averaging." international conference on computer vision (2015): 864-872.

[^Tang17]: Tang, Chengzhou, Oliver Wang, and Ping Tan. "GSLAM: Initialization-Robust Monocular Visual SLAM via Global Structure-from-Motion." international conference on 3d vision (2017): 155-164.

[^Ozyesil17]: Ozyesil, Onur, et al. "A survey of structure from motion." Acta Numerica (2017): 305-364.

[^Zhu18]: Zhu, Siyu, et al. "Very Large-Scale Global SfM by Distributed Motion Averaging." computer vision and pattern recognition (2018): 4568-4577.

[^Agarwal11]: Agarwal, Sameer, et al. "Building rome in a day." Communications of the ACM 54.10 (2011): 105-112.