---
layout:     post
title:      "Plane Sweeping"
date:       2020-3-20
author:     Tong
catalog: true
tags:
    - Technique
---

### Real-time plane-sweeping stereo with multiple sweeping directions [^Gallup07]

#### Abstract

- Recent research has focused on systems for obtaining automatic 3D reconstructions of urban environments from video acquired at street level. These systems record enormous amounts of video; therefore a key component is a stereo matcher which can process this data at speeds comparable to the recording frame rate. Furthermore, urban environments are unique in that they exhibit mostly planar surfaces. These surfaces, which are often imaged at oblique angles, pose a challenge for many window-based stereo matchers which suffer in the presence of slanted surfaces. We present a multi-view plane-sweep-based stereo algorithm which correctly handles slanted surfaces and runs in real-time using the graphics processing unit (GPU). Our algorithm consists of 
    1. identifying the scene’s principle plane orientations, 
    2. estimating depth by performing a plane-sweep for each direction, 
    3. combining the results of each sweep. 

- The latter can optionally be performed using graph cuts. Additionally, by incorporating priors on the locations of planes in the scene, we can increase the quality of the reconstruction and reduce computation time, especially for uniform textureless surfaces. We demonstrate our algorithm on a variety of scenes and show the improved accuracy obtained by accounting for slanted surfaces.

#### Introduction

- In our approach, we perform multiple plane-sweeps, where each plane-sweep is intented to reconstruct planar surfaces having a particular normal. Therefore, our algorithm consists of three steps.
    1. First, we identify the surface normals of the ground and facade planes by analysis of 3D points obtained through sparse struction from motion.
    2. Second, we perform a plane-sweep for each surface normal, resulting in multiple depth candidates for each pixel in the final depth map.
    3. Third, we select the best depth/normal combination or each pixel using a simple best-cost approach or, optionally, a more advanced three-label graph cut which takes smoothness and integrability into account.

- Additionally, we incorporate priors obtained from sparse point correspondences into our depth estimation. This aids in areas with little texture and produces a smoother result.

- We can also significantly reduce computation time by sweeping planes only in those regions with high prior probability according to the sparse data.

#### Plane-sweeping Stereo

1. Extracting the depth map from the cost
    - The first step is to select the best plane at each pixel in the reference view. This may simply be the plane of minimum cost, also called best-cost or winner-takes-all.
    - For each pixel we compute the cost for each plane using the left and right subset of the cameras and select the minimum. This scheme is very effective against occlusions, since typically the visibility of a pixel changes at most once in a sequence of images.
2. Implications of cost aggregation

#### Multiple Sweeping Directions

1. Identifying sweeping directions
2. Plane selection
3. Incorporating plane priors
4. Selecting depths from multiple sweeps

### Multi-resolution real-time stereo on commodity graphics hardware [^Yang03]

#### Abstract

In this paper a stereo algorithm suitable for implementation on commodity graphics hardware is presented. This is important since it allows to free up the main processor for other tasks including high-level interpretation of the stereo results. Our algorithm relies on the traditional sum-of-square-differences (SSD) dissimilarity measure between correlation windows. To achieve good results close to depth discontinuities as well as on low texture areas a multiresolution approach is used. The approach efficiently combines SSD measurements for windows of different sizes. Our implementation running on an NVIDIA GeForce4 graphics card achieves 50-70M disparity evaluations per second including all the overhead to download images and read-back the disparity map, which is equivalent to the fastest commercial CPU implementations available. An important advantage of our approach is that rectification is not necessary so that correspondences can just as easily be obtained for images that contain the epipoles. Another advantage is that this approach can easily be extended to multi-baseline stereo.

#### Introduction

At the heart of our method is a multi-resolution approach to achieve good results close to depth discontinuities as well as on low texture areas. We combine the sum-of-square differences (SSD) dissimilarity measures for windows of different sizes. This is, in fact, equivalent to using a large weighted correlation kernel with a pyramid shape. By utilizing the mipmap functionality [^Williams83] on the graphics hardware, we can compute this dissimilarity measure very efficiently.


### A space-sweep approach to true multi-image matching [^Collins96]

#### Abstract

The problem of determining feature correspondences across multiple views is considered. The term "true multi-image" matching is introduced to describe techniques that make full and efficient use of the geometric relationships between multiple iamges and the scene. A true multi-image technique must generalize to any number of images, be of linear algorithmic complexity in the number of images, and use all the images in an equal manner. A new space-sweep approach to true multi-image matching is presented that simultaneously determines 2D feature correspondences and the 3D positions of feature points in the scene. The method is illustrated on a seven-image matchign example from the aerial image domain.

### Literature

[^Collins96]: Collins, Robert T. "A space-sweep approach to true multi-image matching." Proceedings CVPR IEEE Computer Society Conference on Computer Vision and Pattern Recognition. IEEE, 1996.

[^Gallup07]: Gallup, David, et al. "Real-time plane-sweeping stereo with multiple sweeping directions." 2007 IEEE Conference on Computer Vision and Pattern Recognition. IEEE, 2007.

[^Merrell]: Merrell, Paul, et al. "Real-time visibility-based fusion of depth maps." 2007 IEEE 11th International Conference on Computer Vision. IEEE, 2007.

[^Yang03]: Yang, Ruigang, and Marc Pollefeys. "Multi-resolution real-time stereo on commodity graphics hardware." 2003 IEEE Computer Society Conference on Computer Vision and Pattern Recognition, 2003. Proceedings.. Vol. 1. IEEE, 2003.

[^Williams83]: Lance Williams. Pyramidal Parametrics. In Computer Graphics (SIGGRAPH 1983 Proceedings), volume 17, pages 1–11, July 1983.

