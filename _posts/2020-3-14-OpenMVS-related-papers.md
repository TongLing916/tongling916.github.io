---
layout:     post
title:      "OpenMVS"
date:       2020-3-14
author:     Tong
catalog: true
tags:
    - SLAM
---

> [OpenMVS](https://github.com/cdcseacave/openMVS/wiki/Modules)

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

### Memory efficient semi-global matching [^Hirschmüller2012]

#### Abstract

Semi-Global Matching (SGM) is a robust stereo method that has proven its usefulness in various applications ranging from aerial image matching to driver assistance systems. It supports pixelwise matching for maintaining sharp object boundaries and fine structures and can be implemented efficiently on different computation hardware. Furthermore, the method is not sensitive to the choice of parameters. The structure of the matching algorithm is well suited to be processed by highly paralleling hardware e.g. FPGAs and GPUs. The drawback of SGM is the temporary memory requirement that depends on the number of pixels and the disparity range. On the one hand this results in long idle times due to the bandwidth limitations of the external memory and on the other hand the capacity bounds are quickly reached. A full HD image with a size of 1920 × 1080 pixels and a disparity range of 512 pixels requires already 1 billion elements, which is at least several GB of RAM, depending on the element size, wich are not available at standard FPGA- and GPUboards. The novel memory efficient (eSGM) method is an advancement in which the amount of temporary memory only depends on the number of pixels and not on the disparity range. This permits matching of huge images in one piece and reduces the requirements of the memory bandwidth for real-time mobile robotics. The feature comes at the cost of 50% more compute operations as compared to SGM. This overhead is compensated by the previously idle compute logic within the FPGA and the GPU and therefore results in an overall performance increase. We show that eSGM produces the same high quality disparity images as SGM and demonstrate its performance both on an aerial image pair with 142 MPixel and within a real-time mobile robotic application. We have implemented the new method on the CPU, GPU and FPGA. We conclude that eSGM is advantageous for a GPU implementation and essential for an implementation on our FPGA.

### Exploiting visibility information in surface reconstruction to preserve weakly supported surfaces [^Jancosek2014]

#### Abstract

We present a novel method for 3D surface reconstruction from an input cloud of 3D points augmented with visibility information. We observe that it is possible to reconstruct surfaces that do not contain input points. Instead of modeling the surface from input points, we model free space from visibility information of the input points. The complement of the modeled free space is considered full space. The surface occurs at interface between the free and the full space. We show that under certain conditions a part of the full space surrounded by the free space must contain a real object also when the real object does not contain any input points; that is, an occluder reveals itself through occlusion. Our key contribution is the proposal of a new interface classifier that can also detect the occluder interface just from the visibility of input points. We use the interface classifier to modify the state-of-the-art surface reconstruction method so that it gains the ability to reconstruct weakly supported surfaces. We evaluate proposed method on datasets augmented with different levels of noise, undersampling, and amount of outliers. We show that the proposed method outperforms other methods in accuracy and ability to reconstruct weakly supported surfaces.


### High accuracy and visibility-consistent dense multiview stereo [^Vu2011]

#### Abstract

Since the initial comparison of Seitz et al. [^Seitz2006], the accuracy of dense multiview stereovision methods has been increasing steadily. A number of limitations, however, make most of these methods not suitable to outdoor scenes taken under uncontrolled imaging conditions. The present work consists of a complete dense multiview stereo pipeline which circumvents these limitations, being able to handle large-scale scenes without sacrificing accuracy. Highly detailed reconstructions are produced within very reasonable time thanks to two key stages in our pipeline: a minimum s-t cut optimization over an adaptive domain that robustly and efficiently filters a quasidense point cloud from outliers and reconstructs an initial surface by integrating visibility constraints, followed by a mesh-based variational refinement that captures small details, smartly handling photo-consistency, regularization, and adaptive resolution. The pipeline has been tested over a wide range of scenes: from classic compact objects taken in a laboratory setting, to outdoor architectural scenes, landscapes, and cultural heritage sites. The accuracy of its reconstructions has also been measured on the dense multiview benchmark proposed by Strecha et al. [^Strecha2008], showing the results to compare more than favorably with the current state-of-the-art methods.

### Let there be color! Large-scale texturing of 3D reconstructions [^Waechter2014]

#### Abstract

3D reconstruction pipelines using structure-from-motion and multi-view stereo techniques are today able to reconstruct impressive, large-scale geometry models from images but do not yield textured results. Current texture creation methods are unable to handle the complexity and scale of these models. We therefore present the first comprehensive texturing framework for large-scale, real-world 3D reconstructions. Our method addresses most challenges occurring in such reconstructions: the large number of input images, their drastically varying properties such as image scale, (out-of-focus) blur, exposure variation, and occluders (e.g., moving plants or pedestrians). Using the proposed technique, we are able to texture datasets that are several orders of magnitude larger and far more challenging than shown in related work.


### Literature

[^Barnes2009]: Barnes, Connelly, et al. "PatchMatch: A randomized correspondence algorithm for structural image editing." ACM Transactions on Graphics (ToG). Vol. 28. No. 3. ACM, 2009.

[^Hirschmüller2012]: Hirschmüller, Heiko, Maximilian Buder, and Ines Ernst. "Memory efficient semi-global matching." ISPRS Annals of the Photogrammetry, Remote Sensing and Spatial Information Sciences 3 (2012): 371-376.

[^Jancosek2014]: Jancosek, Michal, and Tomas Pajdla. "Exploiting visibility information in surface reconstruction to preserve weakly supported surfaces." International scholarly research notices 2014 (2014).

[^Vu2011]: Vu, Hoang-Hiep, et al. "High accuracy and visibility-consistent dense multiview stereo." IEEE transactions on pattern analysis and machine intelligence 34.5 (2011): 889-901.

[^Waechter2014]: Waechter, Michael, Nils Moehrle, and Michael Goesele. "Let there be color! Large-scale texturing of 3D reconstructions." European conference on computer vision. Springer, Cham, 2014.

[^Seitz2006]: Seitz, Steven M., et al. "A comparison and evaluation of multi-view stereo reconstruction algorithms." 2006 IEEE computer society conference on computer vision and pattern recognition (CVPR'06). Vol. 1. IEEE, 2006.

[^Strecha2008]: Strecha, Christoph, et al. "On benchmarking camera calibration and multi-view stereo for high resolution imagery." 2008 IEEE Conference on Computer Vision and Pattern Recognition. Ieee, 2008.
