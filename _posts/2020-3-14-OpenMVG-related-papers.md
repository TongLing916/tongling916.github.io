---
layout:     post
title:      "OpenMVG"
date:       2020-3-14
author:     Tong
catalog: true
tags:
    - SLAM
---

> [OpenMVG](https://github.com/openMVG/openMVG) [^Moulon2016]

### Adaptive structure from motion with a contrario model estimation [^Moulon2012]

#### Abstract

Structure from Motion (SfM) algorithms take as input multi-view stereo images (along with internal calibration information) and yield a 3D point cloud and camera orientations / poses in a common 3D coordinate system. In the case of an incremental SfM pipeline, the process requires repeated model estimations based on detected feature points: homography, fundamental and essential matrices, as well as camera poses. These estimations have a crucial impact on the quality of 3D reconstruction. We propose to improve these estimations using the _a contrario_ methodology. While SfM pipelines usually have globally-fixed thresholds to the input data and for each model estimation. Our experiments show that adaptive thresholds reach a significantly better precision. Additionally, the user is free from having to guess thresholds or to optimistically rely on default values. There are also cases where a globally-fixed threshold policy, whatever the threshold value is, cannot provide the best accuracy, contrary to an adaptive threhold policy.

### Unordered feature tracking made fast and easy [^Moulon2012]

#### Abstract

We present an efficient algorithm to fuse two-view correspondences into multi-view consistent tracks. The proposed method relies on the Union-Find [^Galler1964] algorithm to solve the fusion problem. It is very simple and has a lower computational complexity than other available methods. Our experiments show that it is faster and computes more tracks.

### Automatic homographic registration of a pair of images, with a contrario elimination of outliers [^Moisan2012]

> [Demo](http://www.ipol.im/pub/art/2012/mmm-oh/?utm_source=doi)

#### Abstract

The RANSAC [^Fischler1981] algorithm (RANdom SAmple Consensus) is a robust method to estimate parameters of a model fitting the data, in presence of outliers among the data. Its random nature is due only to complexity considerations. It iteratively extracts a random sample out of all data, of minimal size sufficient to estimate the parameters. At each such trial, the number of inliers (data that fits the model within an acceptable error threshold) is counted. In the end, the set of parameters maximizing the number of inliers is accepted.

The variant proposed by Moisan and Stival [^Moisan2004] consists in introducing an a contrario [^Desolneux2000] criterion to avoid the hard thresholds for inlier/outlier discrimination. It has three consequences:
    1. The threshold for inlier/outlier discrimination is adaptive, it does not need to be fixed.
    2. It gives a decision on the adequacy of the final model: it does not provide a wrong set of parameters if it does not have enough confidence.
    3. The procedure to draw a new sample can be amended as soon as one set of parameters is deemed meaningful: the new sample can be drawn among the _inliers_ of this model.

In this particular instantiation, we apply it to the estimation of the homography registering two images of the same scene. The homography is an 8-parameter model arising in two situations when using a pinhole camera: the scene is planar (a painting, a facade, etc.) or the viewpoint location is fixed (pure rotation around the optical center). When the homography is found, it is used to stitch the images in the coordinate frame of the second image and build a panorama. The point correspondences between images are computed by the SIFT [^Lowe2004] algorithm.

### Global fusion of relative motions for robust, accurate and scalable structure from motion [^Moulon2013]

#### Abstract

Multi-view structure from motion (SfM) estimates the position and orientation of pictures in a common 3D coordinate frame. When views are treated incrementally, this external calibration can be subject to drift, contrary to global methods that distribute residual errors evenly. We propose a new global calibration approach based on the fusion of relative motions between image pairs. We improve an existing method for robustly computing global rotations. We present an efficient a contrario trifocal tensor estimation method, from which stable and precise translation directions can be extracted. We also define an efficient translation registration method that recovers accurate camera positions. These components are combined into an original SfM pipeline. Our experiments show that, on most datasets, it outperforms in accuracy other existing incremental and global pipelines. It also achieves strikingly good running times: it is about 20 times faster than the other global method we could compare to, and as fast as the best incremental method. More importantly, it features better scalability properties.


### Literature

[^Moulon2016]: Moulon, Pierre, et al. "Openmvg: Open multiple view geometry." International Workshop on Reproducible Research in Pattern Recognition. Springer, Cham, 2016.

[^Moulon2012]: Moulon, Pierre, Pascal Monasse, and Renaud Marlet. "Adaptive structure from motion with a contrario model estimation." Asian Conference on Computer Vision. Springer, Berlin, Heidelberg, 2012.

[^Moulon2012]: Moulon, Pierre, and Pascal Monasse. "Unordered feature tracking made fast and easy." 2012.

[^Moisan2012]: Moisan, Lionel, Pierre Moulon, and Pascal Monasse. "Automatic homographic registration of a pair of images, with a contrario elimination of outliers." Image Processing On Line 2 (2012): 56-73.

[^Moulon2013]: Moulon, Pierre, Pascal Monasse, and Renaud Marlet. "Global fusion of relative motions for robust, accurate and scalable structure from motion." Proceedings of the IEEE International Conference on Computer Vision. 2013.

[^Galler1964]: Galler, Bernard A., and Michael J. Fisher. "An improved equivalence algorithm." Communications of the ACM 7.5 (1964): 301-303.

[^Fischler1981]: M.A. Fischler and R.C. Bolles. Random sample consensus: a paradigm for model fitting with applications to image analysis and automated cartography. Communications of the ACM, 24(6):381–395, 1981. http://dx.doi.org/10.1145/358669.358692.

[^Desolneux2000]: A. Desolneux, L. Moisan, and J.M. Morel. Meaningful alignments. International Journal of Computer Vision, 40(1):7–23, 2000. http://dx.doi.org/10.1023/A:1026593302236

[^Moisan2004]: L. Moisan and B. Stival. A probabilistic criterion to detect rigid point matches between two images and estimate the fundamental matrix. International Journal of Computer Vision, 57(3):201–218, 2004. http://dx.doi.org/10.1023/B:VISI.0000013094.38752.54.

[^Lowe2004]: D.G. Lowe. Distinctive image features from scale-invariant keypoints. International journal of computer vision, 60(2):91–110, 2004. http://dx.doi.org/10.1023/B:VISI.0000029664.99615.94.
