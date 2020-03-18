---
layout:     post
title:      "Correspondence"
date:       2020-3-20
author:     Tong
catalog: true
tags:
    - Technique
---

### State of the art in high density image matching [^Remondino14]

#### Abstract

### Memory efficient semi-global matching [^Hirschmueller2012]

#### Abstract

Semi-Global Matching (SGM) is a robust stereo method that has proven its usefulness in various applications ranging from aerial image matching to driver assistance systems. It supports pixelwise matching for maintaining sharp object boundaries and fine structures and can be implemented efficiently on different computation hardware. Furthermore, the method is not sensitive to the choice of parameters. The structure of the matching algorithm is well suited to be processed by highly paralleling hardware e.g. FPGAs and GPUs. The drawback of SGM is the temporary memory requirement that depends on the number of pixels and the disparity range. On the one hand this results in long idle times due to the bandwidth limitations of the external memory and on the other hand the capacity bounds are quickly reached. A full HD image with a size of 1920 × 1080 pixels and a disparity range of 512 pixels requires already 1 billion elements, which is at least several GB of RAM, depending on the element size, wich are not available at standard FPGA- and GPUboards. The novel memory efficient (eSGM) method is an advancement in which the amount of temporary memory only depends on the number of pixels and not on the disparity range. This permits matching of huge images in one piece and reduces the requirements of the memory bandwidth for real-time mobile robotics. The feature comes at the cost of 50% more compute operations as compared to SGM. This overhead is compensated by the previously idle compute logic within the FPGA and the GPU and therefore results in an overall performance increase. We show that eSGM produces the same high quality disparity images as SGM and demonstrate its performance both on an aerial image pair with 142 MPixel and within a real-time mobile robotic application. We have implemented the new method on the CPU, GPU and FPGA. We conclude that eSGM is advantageous for a GPU implementation and essential for an implementation on our FPGA.


### Unordered feature tracking made fast and easy [^Moulon2012b]

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

### PatchMatch: A Randomized Correspondence Algorithm for Structural Image Editing [^Barnes2009]

#### Abstract

This paper presents interactive image editing tools using a new randomized algorithm for quickly finding approximate nearest-neighbor matches between image patches. Previous research in graphics and vision has leveraged such nearest-neighbor searches to provide a variety of high-level digital image editing tools. However, the cost of computing a field of such matches for an entire image has eluded previous efforts to provide interactive performance. Our algorithm offers substantial performance improvements over the previous state of the art (20-100x), enabling its use in interactive editing tools. The key insights driving the algorithm are that some good patch matches can be found via random sampling, and that natural coherence in the imagery allows us to propagate such matches quickly to surrounding areas. We offer theoretical analysis of the convergence properties of the algorithm, as well as empirical and practical evidence for its high quality and performance. This one simple algorithm forms the basis for a variety of tools – image retargeting, completion and reshuffling – that can be used together
in the context of a high-level image editing application. Finally, we propose additional intuitive constraints on the synthesis process that offer the user a level of control unavailable in previous methods.

#### 1. Introduction

- The core element of nonparametric patch sampling methods is a repeated search of all patches in one image region for the most similar patch in another image region.
    - In other words, given images or regions A and B, find for every patch in A the nearest neighbor in B under a patch distance metric such $$L_p$$. We call this mapping the _Nearest-Neighbor Field_ (NNF).

- Our new algorithm relies on three key observations about the problem:
    - __Dimensionality of offset space.__ Our algorithm searches in the 2D space of possible patch offsets, achieving greater speed and memory efficiency.
    - __Natural structure of images.__ We can improve efficiency by performing searches for adjacent pixels in an interdependent manner.
    - __The law of large numbers.__ As this field grows larger, the chance that no patch will have a correct offset becomes vanishingly small.

- We offer a randomized algorithm for computing approximate NNFs using incremental updates.
    - The algorithm begins with an initial guess, which may be derived from prior information or may simply be a random field.
    - The iterative process consists of two phases:
        - _propagation_, in which coherence is used to disseminate good solutions to adjacent pixels in the field.
        - _random search_, in which the current offset vector is perturbed by multiple scales of random offsets.

- The contributions of our work include a fast randomized approximation algorithm for computing the nearest-neighbor field between two disjoint image regions; an application of this algorithm within a structural image editing framework that enables high-quality interactive image retargeting, image completion, and image reshuffling; and a set of intuitive interactive controls used to constrain the optimization process to obtain desired creative results.

#### 3. Approximate nearest-neighbor algorithm

The algorithm has three main components.
    - Initially, the nearest-neighbor field is filled with either random offsets or some prior information.
    - Next, an iterative update process is applied to the NNF, in which good patch offsets are propagated to adjacent pixels,
    - followed by random search in the neighborhood of the best offset found so far.

### Literature

[^Barnes2009]: Barnes, Connelly, et al. "PatchMatch: A randomized correspondence algorithm for structural image editing." ACM Transactions on Graphics (ToG). Vol. 28. No. 3. ACM, 2009.

[^Hirschmueller2012]: Hirschmüller, Heiko, Maximilian Buder, and Ines Ernst. "Memory efficient semi-global matching." ISPRS Annals of the Photogrammetry, Remote Sensing and Spatial Information Sciences 3 (2012): 371-376.

[^Moulon2012b]: Moulon, Pierre, and Pascal Monasse. "Unordered feature tracking made fast and easy." 2012.

[^Galler1964]: Galler, Bernard A., and Michael J. Fisher. "An improved equivalence algorithm." Communications of the ACM 7.5 (1964): 301-303.

[^Moisan2012]: Moisan, Lionel, Pierre Moulon, and Pascal Monasse. "Automatic homographic registration of a pair of images, with a contrario elimination of outliers." Image Processing On Line 2 (2012): 56-73.

[^Fischler1981]: M.A. Fischler and R.C. Bolles. Random sample consensus: a paradigm for model fitting with applications to image analysis and automated cartography. Communications of the ACM, 24(6):381–395, 1981. http://dx.doi.org/10.1145/358669.358692.

[^Desolneux2000]: A. Desolneux, L. Moisan, and J.M. Morel. Meaningful alignments. International Journal of Computer Vision, 40(1):7–23, 2000. http://dx.doi.org/10.1023/A:1026593302236

[^Moisan2004]: L. Moisan and B. Stival. A probabilistic criterion to detect rigid point matches between two images and estimate the fundamental matrix. International Journal of Computer Vision, 57(3):201–218, 2004. http://dx.doi.org/10.1023/B:VISI.0000013094.38752.54.

[^Lowe2004]: D.G. Lowe. Distinctive image features from scale-invariant keypoints. International journal of computer vision, 60(2):91–110, 2004. http://dx.doi.org/10.1023/B:VISI.0000029664.99615.94.

[^Remondino14]: Remondino, Fabio, et al. "State of the art in high density image matching." The photogrammetric record 29.146 (2014): 144-166.