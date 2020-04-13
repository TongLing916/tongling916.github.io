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

This tutorial presents a hands-on view of the field of multi-view stereo with a focus on practical algorithms. Multi-view stereo algorithms are able to construct highly detailed 3D models from images alone. They take a possibly very large set of images and construct a 3D plausible geometry that explains the images under some reasonable assumptions, the most important being scene rigidity. The tutorial frames the multi-view stereo problem as an image/geometry consistency optimization problem. It describes in detail its main two ingredients: robust implementations of photometric consistency measures, and efficient optimization algorithms. It then presents how these main ingredients are used by some of the most successful algorithms, applied into real applications, and deployed as products in the industry. Finally it describes more advanced approaches exploiting domain-specific knowledge such as structural priors, and gives an overview of the remaining challenges and future research directions.

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


### 2 Multi-view Photo-consistency

- A __crucial requirement__ for photo-consistency measures is to compute photo-consistency on a set of images that see the __same__ 3D geometry.

#### 2.1 Photo-consistency measures

- Summary of different __similarity measures__

|Measure|require support domain?|invariance|
| --- | --- | --- |
|Sum of Squared Differences (SSD) |no| none |
|Sum of Absolute Differences (SAD)|no| none|
|Normalized Cross Correlation (NCC)|yes|bias / gain|
|Census [^Zabih94]|yes|bias / gain|
|Rank [^Zabih94]|yes|bias / gain / rotation|
|Mutual Information (MI)|yes| any bijection |

- The __size of a support domain__ needs to be proportional to the image resolution and the viewpoint separation, and inversely proportional to the distance to the scene.
- A related concept to the domain that is very common in stereo algorithms is __photo-consistency aggregation__ [^Scharstein02], which consists in spatially aggregating the photo-consistency measure to increase its robustness.
- Given __color images__, different strategies exist to deal with different channels:
  - Convert the color image into gray scale _before_ computing the photo-consistency
  - Compute the photo-consistency per color channel independently, and return the average.
  - Concatenate the vectors from all the color channels into a single larger vector.
- The handling of __color images__ in __NCC__ is to compute and substract the average intensity per color channel independently, but concatenate all the color channels together as a single vector when computing its variance.
- A __normalized variance of SSD__ is equivalent to NCC.
- The __main advantage__ of __Census__ vs NCC is around depth boundaries, where Census is more robust than NCC, because the assumption (the appearance in the domain is intrinsic to the object and, to some extent, invariant to illumination and viewpoint changes.) __breaks__ at depth discontinuities because the domain contains foreground and background objects.
- The main characteristic of MI (mutual information) is that it is highly invariant, in particular to any transformation that does not change the amount of information of a signal, e.g., bijective mappings. However, it is not a common measure since its invariance comes at the cost of degraded accuracy.
- __Normalization__ is important for parameter tuning as well as combining photo-consistency meausres together.
  - Typical transforms: _exponential_, _linear truncated_, _smooth step_.
  - The shape of the normalization function is expected to be sigmoid-like, with two plateaus separated by a relatively steep ramp.
  - The rationale behind it is that the steep slope acts as the optimal threshold separating the inlier/outlier model.
    - For SAD, inlier errors < 5 levels (out of 255), while outlier values are > 10 levels.
    - NCC values below $$\frac{1}{\sqrt{2}}$$ are typically considered not accurate enough and discarded.
- Photo-consistency aggregation
  - box filter
  - anisotropic fitlers
    - bilateral filter [^Tomasi98]
    - guided filter [^He10]
    - Weighted median filter [^Ma13]

#### 2.2 Visibility estimation in state-of-the-art algorithms

- Space-carving for visibility estimation [^Seitz97] [^Kutulakos00]
  - Given a 3D volume partitioned into a 3D grid of voxels, the volume is iteratively carved out by removing voxels that are not photo-consistent.
  - The __main contribution__ of the work was the proposal of a geometric constraint on the camera centers such that there exists an __ordinal visibility constraint__ on all the 3D voxels in the scene. That is, starting from an initial bounding volume, one can visit all the voxels in an order that guarantees that any potential occluder voxel is always visited before its potential occluded voxel.
- Visibility estimate for large image collections typically happen in two phases
  - In the first phase, visibility is estimated _coarsely_ by clustering the initial set of images and reducing the large-scale MVS problem into a sequence of small sub-problems. [^Yasutaka10a] [^Yasutaka10b] [^Hernandez04] [^Goesele07]
  - In the second phase, more fine-scale visibility estimation is conducted per 3D point basis.
    - One popular approach is to use the current reconstructed geometry to compute occlusions (e.g., a z-buffer testing), select which views see which parts of the geometry, and iterate visibility estimation and reconstruction.
    - Another popular solutions is to rely on robust photo-consistency statistics without explicitly estimating occlusion.

### 3 Algorithms: From Photo-Consistency to 3D Reconstruction

||View-dependent texture-mapping|Free-viewpoint rendering|Ease of manipulation (e.g., merging/splitting)|
|-|-|-|-|
|Depthmaps|Good|Fair (point-based rendering)|Fair|
|Point-cloud|Poor|Fair (point-based rendering)|Good|
|Voxels|No|No|Good|
|Mesh|Poor|Good|Bad|

- Evaluations
  - [Middlebury](http://vision.middlebury.edu/mview/data/)
  - [ETH3D](https://www.eth3d.net/overview)
  - [KITTI](www.cvlibs.net/datasets/kitti/)
  
> [Robust Vision Challenge](http://robustvision.net/submit.php) 

#### 3.1 Depthmap Reconstruction

- __Winner-Takes-All__ Depthmaps
  - Evaluate photo-consistency values throughout the depth range, and pick the depth value with the highest photo-consistency score for each pixel independently.
  - In addition to the depth value with the highest photo-consistency, the algorithm often evaluates a __confidence measure__ so that low-confidence depth values can be ignored or down-weighted in the later model mergin step [^Hu12].
- __Robust__ Photo-Consistency Depthmaps
  - Robust photo-consistency function [^Vogiatzis07]: a sum of kernel function (e.g., Gaussian) at major local maxima.
  - Another effective approach is to ignore photo-consistency scores that are below a certain threshold. [^Goesele06]
- __Markov Random Field (MRF) Depthmaps__
  - In the presence of severe occlusions, there may not exist a corresponding match in most other images. A standard solution is to enforce __spatial consistency__, under the assumption that neighboring pixels have similar depth values.
  - The MRF depth map formulations [^Kolmogorov01] can be seen as a combinatorial optimization problem, where an input depth range is discretized into a finite set of depth values.
  - The cost function is comprised of two parts
    - Unary potential: photo-consistency information
    - Pairwise interaction potentials: spatial regularization and proportional to the amount of depth discrepancy at neighboring pixels.
- __Mutiple Hypothesis MRF Depthmaps__
  - Instead of using blindly discretized depth values as possible label set for an entire image, their algorithm extracts local maxima from photo-consistency curves for each pixel, then the MRF formulation is used to assigned the depth of one of such local maxima to each pixel. [^Campbell08]
  - They also allow the "unknown" label to indicate that no correct depth can be estimated in certain cases.
  - The process consists of two phases
    1. Extraction of depth labels;
    2. MRF optimization to assign extracted depth labels.
- More Depthmap Reconstruction Algorithms
  - Real-time __plane sweeping__ depthmap reconstruction [^Gallup07]
    - It sweeps a family of parallel planes in a scene, projects images onto a plane via planar homographies, then evaluates photo-consistency values on each plane.
  - __Second order smoothness__ [^Woodford08]
    - MRF formulation, but a second order smoothness prior over triple cliques (i.e., three pixels) that enforces piece-wise planar surfaces

#### 3.2 Point-cloud Reconstruction

#### 3.3 Volumetric data fusion

#### 3.4 MVS Mesh Refinement

### Literature

[^Furukawa15]: Furukawa, Yasutaka, and Carlos Hernández. "Multi-view stereo: A tutorial." Foundations and Trends® in Computer Graphics and Vision 9.1-2 (2015): 1-148.

[^Yasutaka10a]: Yasutaka Furukawa and Jean Ponce. Accurate, dense, and robust multi-view stereopsis. IEEE Transactions on Pattern Analysis and Machine Intelligence, 32(8):1362–1376, August 2010.

[^Yasutaka10b]: Yasutaka Furukawa, Brian Curless, Steven M. Seitz, and Richard Szeliski. Towards Internet-scale multiview stereo. In IEEE Conference on Computer Vision and Pattern Recognition, 2010.

[^Scharstein02]: D. Scharstein and R. Szeliski. A taxonomy and evaluation of dense two-frame stereo correspondence algorithms. International Journal of Computer Vision, 47(1/2/3) 7–42, 2002.

[^Zabih94]: Ramin Zabih and John Woodfill. Non-parametric local transforms for computing visual correspondence. In European Conference on Computer Vision, pages 151–158, Stockhold, Sweden, 1994.

[^Tomasi98]: C. Tomasi and R. Manduchi. Bilateral filtering for gray and color images. In IEEE International Conference on Computer Vision, pages 839–846, 1998.

[^He10]: Kaiming He, Jian Sun, and Xiaoou Tang. Guided image filtering. In European Conference on Computer Vision, ECCV’10, pages 1–14, Berlin, Heidelberg, 2010.

[^Ma13]: Ziyang Ma, Kaiming He, Yichen Wei, Jian Sun, and Enhua Wu. Constant time weighted median filtering for stereo matching and beyond. In IEEE International Conference on Computer Vision, 2013.

[^Seitz97]: Steven M. Seitz and Charles R. Dyer. Photorealistic scene reconstruction by voxel coloring. In IEEE Conference on Computer Vision and Pattern Recognition, pages 1067–, Washington, DC, USA, 1997. IEEE Computer Society.

[^Kutulakos00]: K.N. Kutulakos and S. M. Seitz. A theory of shape by space carving. International Journal of Computer Vision, 38(3):199–218, 2000.

[^Hernandez04]: Carlos Hernández and Francis Schmitt. Silhouette and stereo fusion for 3d object modeling. Computer Vision and Image Understanding, 96(3):367–392, 2004.

[^Goesele07]: M. Goesele, N. Snavely, B. Curless, H. Hoppe, and S.M. Seitz. Multi-view stereo for community photo collections. In IEEE International Conference on Computer Vision, pages 1–8, 2007.

[^Goesele06]: Michael Goesele, Brian Curless, and Steven M. Seitz. Multi-view stereo revisited. In IEEE Conference on Computer Vision and Pattern Recognition, pages 2402–2409, 2006.

[^Hu12]: Xiaoyan Hu and P. Mordohai. A quantitative evaluation of confidence measures for stereo vision. IEEE Transactions on Pattern Analysis and Machine Intelligence, 34(11):2121–2133, Nov 2012.

[^Vogiatzis07]: G. Vogiatzis, C. Hernández, P. H S Torr, and R. Cipolla. Multi-view stereo via volumetric graph-cuts and occlusion robust photo-consistency. IEEE Transactions on Pattern Analysis and Machine Intelligence, 29(12):2241–2246, 2007.

[^Kolmogorov01]: V. Kolmogorov and R. Zabih. Computing visual correspondence with occlusions using graph cuts. In IEEE International Conference on Computer Vision, volume 2, pages 508–515 vol.2, 2001.

[^Campbell08]: Neill D.F. Campbell, George Vogiatzis, Carlos Hernández, and Roberto Cipolla. Using multiple hypotheses to improve depth-maps for multi-view stereo. In 10th European Conference on Computer Vision, volume 5302 of LNCS, pages 766–779, 2008.

[^Gallup07]: David Gallup, Jan-Michael Frahm, Philippos Mordohai, Qingxiong Yang, and Marc Pollefeys. Real-time plane-sweeping stereo with multiple sweeping directions. In IEEE Conference on Computer Vision and Pattern Recognition, 2007.

[^Woodford08]: Woodford, O. J., Torr, P. H. S., Reid, I. D., & Fitzgibbon, A. W. (2008). Global stereo reconstruction under second order smoothness priors. 2008 IEEE Conference on Computer Vision and Pattern Recognition. 