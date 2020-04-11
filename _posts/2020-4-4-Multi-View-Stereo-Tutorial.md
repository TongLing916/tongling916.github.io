---
layout:     post
title:      "Multi-View Stereo: A Tutorial"
date:       2020-4-4
author:     Tong
catalog: true
tags:
    - Reconstruction
---

### Abstract [^Furukawa15]

This tutorial presents a hands-on view of the field of multi-view stereo with a focus on practical algorithms. Multi-view stereo algorithms are able to construct highly detailed 3D models from images alone. They take a possibly very large set of images and construct a 3D plausible geometry that explains the images under some reasonable assumptions, the most important being scene rigidity. The tutorial frames the multiview stereo problem as an image/geometry consistency optimization problem. It describes in detail its main two ingredients: robust implementations of photometric consistency measures, and efficient optimization algorithms. It then presents how these main ingredients are used by some of the most successful algorithms, applied into real applications, and deployed as products in the industry. Finally it describes more advanced approaches exploiting domain-specific knowledge such as structural priors, and gives an overview of the remaining challenges and future research directions.

### 1 Introduction

1. All the MVS algorithms described in the following chapters assume the same input: 
   1. a set of images.
   2. their corresponding camera paramters (pose and intrinsic parameters).
2. An MVS algorithm is only as good as the quality of the input images and camera parameters.
3. __MVS pipeline__
   1. Collect images.
   2. Compute camera parameters for each image.
   3. Reconstruct the 3D geometry of the scene from the set of images and corresponding camera parameters.
   4. Optinally reconstruct the materials of the scene.
4. SfM (Structure from Motion) pipeline:
   1. Detect 2D features in every input image.
   2. Match 2D features between images.
   3. Construct 2D tracks from the matches.
   4. Solve for the SfM model from the 2D tracks.
   5. Refine the SfM model using bundle adjustment.
5. The robustness of MVS to camera reprojection error depends mainly on how tolerant the matching criterion (_photo-consistency_) is to to misalignments.
   1. The larger the domain of the photo-consistency measure, the more robust the measure is.
   2. Large domains also tend to produce over-smoothed geometry.
6. Because reprojection error is measured in pixels, one can downsample the input images and rescale the camera parameters until the reprojection error drops below a certain threshold.


### Literature

[^Furukawa15]: Furukawa, Yasutaka, and Carlos Hernández. "Multi-view stereo: A tutorial." Foundations and Trends® in Computer Graphics and Vision 9.1-2 (2015): 1-148.