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
  - In the first phase, visibility is estimated _coarsely_ by clustering the initial set of images and reducing the large-scale MVS problem into a sequence of small sub-problems. [^Furukawa10a] [^Furukawa10b] [^Hernandez04] [^Goesele07]
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
  - [DenseMVS](https://icwww.epfl.ch/multiview/denseMVS.html)
  
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

- A 3D point with a __surface normal__ estimation or a __local region__ support is referred to as an oriented point or a patch.

- A common characteristic of point-cloud reconstruction algorithms is that they make use of an __spatial consistency assumption__ and __grows or expand__ the point-cloud on the surface of the scene during the reconstruction process. 

- This article focuses on the work [^Furukawa10a]. 
  - The algorithm also follows a __greedy expansion__ approach, 
  - but one __key difference__ is that it iterates between the expansion and the filtering steps after reconstructing an initial seed of patches via feature matching. 
  - The __filtering__ step analyzes consistency of patches across all the views and removes falsely reconstructed ones.

- __Key Elements__
  - __Patch Model__: A patch $$p$$ is essentially a local tangent plane approximation of a surface, whose geometry is determined by its center $$\mathbf{c}(p)$$ and unit normal vector $$\mathbf{n}(p)$$. 
    - The __robust__ photo-consistency function is simply evaluated by using the patch as a proxy geometry to sample pixel colors.
    - In practice, the perpendicular direction is fixed before and throughout the optimization, where one parameter for position and two paramters for normal are optimized via a standard non-linear least squares technique.
  - __Image-based Data Structure__
    - Image projections of reconstructed patches in the visible images are used to help __searching or accessing neighboring patches__, then __enforcing regularization__.

- __Algorithm__
  - __Initial Feature Matching__
    - The purpose is to generate a spares set of patches
    - Difference-of-Gaussian (DoG) + Harris operators
    - Search matches within two pixels from the corresponding epipolar line
    - Triangulation to initialize a patch
    - Patch optimization
  - __Expansion__
    - The __goal__ is to reconstruct at least one patch in every image cell, where they repeat taking existing patches and generating new ones in nearby empty spaces. 
    - The process repeats until the expansion process is performed from every patch that has been reconstructed.
  - __Filtering__
    - Two filters are used to remove erroneous patches.
      - The first filter relies on visibility consistency.
      - The second filter enforces a weak form of regularization.

#### 3.3 Volumetric data fusion

- __Volumetric surface extraction__ is flexible and the input 3D information can come from many different sources such as photo-consistency volumes, depthmaps, MVS point clouds, laser scanned 3D points, or any combination of those. It is a challenging task to fuse such diverse set of 3D measurements into a single clean mesh model with the right topology.
- A standard yet powerful solution is to accumulate evidence in a 3D voxel grid and extract a surface model via Marching Cube algorithm.
  - One formulation is to compute a __signed distance function__ field over the voxel grid, then pose a surface reconstruction as zero iso-surface extraction. [^Curless96] [^Zach07]
  - The other formulation is to pose as a 3D binary segmentation problem, where the boundary of the two segments can be extracted as a surface model. [^Zach07] [^Furukawa09] [^Hernandez07] [^Sinha07] [^Vogiatzis05]
- __Volumetric Graph-Cuts on a Voxel Grid__
  - Given a bounding box that contains the solution surface, the space is discretized with a voxel grid. 
  - The objective is to find the __optimal label assignment__ ("interior" or "exterior") to all the voxels that minimizes the following cost function: $$E\left(\left\{k_{v}\right\}\right)=\sum_{v} \Phi\left(k_{v}\right)+\sum_{(v, w) \in \mathcal{N}} \Psi\left(k_{v}, k_{w}\right)$$
  - The first term (_unary term_) is the summation of per voxel cost over the entire domain, where $$\Phi\left(k_{v}\right)$$ encodes the cost of assigning a label to voxel $$v$$.
  - The second term (pairwise _interaction term_) is the summation over all the pairs of adjacent voxels denoted as $$\mathcal{N}$$.
  - Possibility: a constant _ballooning term_ as the unary term
  - The optimization can be solved exactly and efficiently with a graph-cuts algorithm, as long as each pairwise term is submodular. [^Kolmogorov04]
  - One __limitation__ of the use of a voxel grid is that the memory allocation quickly becomes very expensive. 
    - One effective solution is an __octree__ data structure.
  - Due to the __shrinkage bias__, the reconstruction with the constant balloning term failes in reconstructing deep concavities in various parts of the model.
- A __better alternative__ than uniform voxel grid is to first reconstruct a sparse 3D point cloud of scene, then use the 3D points as nodes of the space discretization grid.
  - 3D Delaunay triangulation [^Labatut07]
  - __Explicit regularization__ term to penalize __label change__ 
    - It is usually __necessary__ at every pair of adjacent cells for a uniform voxel grid structure.
    - In the case of a 3D Delaunay triangulation, explicit regularization in the pairwise term is __not crucial__.
  - If the 3D evidence is weak, this technique will miss thin structures due to the MRF regularization.
    - Solution: add a step to reinforce the evidence of structure by analyzing the gradient of exterior evidence. [^Jancosek11]

#### 3.4 MVS Mesh Refinement

- Mesh refinement algorithms move the location of each vertex $$v_i$$ one by one or simultaneously, while minimizing an objective function defined over the mesh model.
- The object function $$E(\{v_i\})$$ typically consists of 
  - a data term $$E_p$$, which is based on a photometric consistency measure
  - a regularization term $$E_r$$, which measures the smoothness of the mesh.
  - Optionally, when image silhouettes are given as input, silhouette consistency measure $$E_s$$ to enforce that the mesh is consistent with the image silhouettes
- Gradient descent is usually the method of optimization.
  - In practice, the gradients of $$E_p$$ and $$E_s$$ are usually projected along the surface normal.
  - The regularization term $$E_r$$ is used to prefer uniform vertex distribution, and the direct derivative is calculated with respect to $$x_i$$, $$y_i$$, and $$z_i$$.
- Photometric consistency term
  - Three examples: [^Furukawa10a] [^Hernandez04] [^Vu12]
  - Different from many methods that use tangent planes or front-parallel surface approximations, the use of __triangulated mesh__ model for photometric consistency evaluation has several advantages:
    - First, the core texture reprojection (i.e., morphing) operations can be effectively and easily performed over an entire mesh by GPUs.
    - Second, it can handle sharp edge structure such as staircases property.
- Regularization term
  -  __Mesh Laplacian__ and __Mesh Bi-Laplacian__ are often used to define the regularization force. [^Botsch08]
  -  Laplacian becomes zero when the surface is planar.
  -  Bi-Laplician becomes zero when the surface has constant or uniform curvature.
- Sihouette consistency term
  - Silhouettes often delineate a foreground object from background. 
  - Example: [^Hernandez04]

### 4 Multi-view Stereo and Structure Priors

#### 4.1 Departure from Depthmap to Planemap

#### 4.2 Departure from Planes to Geometric Primitives

#### 4.3 Image Classification for Structure Priors


### Literature

[^Furukawa15]: Furukawa, Yasutaka, and Carlos Hernández. "Multi-view stereo: A tutorial." Foundations and Trends® in Computer Graphics and Vision 9.1-2 (2015): 1-148.

[^Furukawa09]: Yasutaka Furukawa, Brian Curless, Steven M. Seitz, and Richard Szeliski. Reconstructing building interiors from images. In IEEE International Conference on Computer Vision, 2009.

[^Furukawa10a]: Yasutaka Furukawa and Jean Ponce. Accurate, dense, and robust multi-view stereopsis. IEEE Transactions on Pattern Analysis and Machine Intelligence, 32(8):1362–1376, August 2010.

[^Furukawa10b]: Yasutaka Furukawa, Brian Curless, Steven M. Seitz, and Richard Szeliski. Towards Internet-scale multiview stereo. In IEEE Conference on Computer Vision and Pattern Recognition, 2010.

[^Scharstein02]: D. Scharstein and R. Szeliski. A taxonomy and evaluation of dense two-frame stereo correspondence algorithms. International Journal of Computer Vision, 47(1/2/3) 7–42, 2002.

[^Zabih94]: Ramin Zabih and John Woodfill. Non-parametric local transforms for computing visual correspondence. In European Conference on Computer Vision, pages 151–158, Stockhold, Sweden, 1994.

[^Tomasi98]: C. Tomasi and R. Manduchi. Bilateral filtering for gray and color images. In IEEE International Conference on Computer Vision, pages 839–846, 1998.

[^He10]: Kaiming He, Jian Sun, and Xiaoou Tang. Guided image filtering. In European Conference on Computer Vision, ECCV’10, pages 1–14, Berlin, Heidelberg, 2010.

[^Ma13]: Ziyang Ma, Kaiming He, Yichen Wei, Jian Sun, and Enhua Wu. Constant time weighted median filtering for stereo matching and beyond. In IEEE International Conference on Computer Vision, 2013.

[^Seitz97]: Steven M. Seitz and Charles R. Dyer. Photorealistic scene reconstruction by voxel coloring. In IEEE Conference on Computer Vision and Pattern Recognition, pages 1067–, Washington, DC, USA, 1997. IEEE Computer Society.

[^Kutulakos00]: K.N. Kutulakos and S. M. Seitz. A theory of shape by space carving. International Journal of Computer Vision, 38(3):199–218, 2000.

[^Hernandez04]: Carlos Hernández and Francis Schmitt. Silhouette and stereo fusion for 3d object modeling. Computer Vision and Image Understanding, 96(3):367–392, 2004.

[^Hernandez07]: Carlos Hernández, George Vogiatzis, and Roberto Cipolla. Probabilistic visibility for multi-view stereo. In IEEE Conference on Computer Vision and Pattern Recognition, 2007.

[^Goesele06]: Michael Goesele, Brian Curless, and Steven M. Seitz. Multi-view stereo revisited. In IEEE Conference on Computer Vision and Pattern Recognition, pages 2402–2409, 2006.

[^Goesele07]: M. Goesele, N. Snavely, B. Curless, H. Hoppe, and S.M. Seitz. Multi-view stereo for community photo collections. In IEEE International Conference on Computer Vision, pages 1–8, 2007.

[^Hu12]: Xiaoyan Hu and P. Mordohai. A quantitative evaluation of confidence measures for stereo vision. IEEE Transactions on Pattern Analysis and Machine Intelligence, 34(11):2121–2133, Nov 2012.

[^Vogiatzis05]: George Vogiatzis, P.H.S. Torr, and Roberto Cipolla. Multi-view stereo via volumetric graph-cuts. In IEEE Conference on Computer Vision and Pattern Recognition, 2005.

[^Vogiatzis07]: G. Vogiatzis, C. Hernández, P. H S Torr, and R. Cipolla. Multi-view stereo via volumetric graph-cuts and occlusion robust photo-consistency. IEEE Transactions on Pattern Analysis and Machine Intelligence, 29(12):2241–2246, 2007.

[^Kolmogorov01]: V. Kolmogorov and R. Zabih. Computing visual correspondence with occlusions using graph cuts. In IEEE International Conference on Computer Vision, volume 2, pages 508–515 vol.2, 2001.

[^Kolmogorov04]: V. Kolmogorov and R. Zabih. What energy functions can be minimized via graph cuts? IEEE Transactions on Pattern Analysis and Machine Intelligence, 26(2):147–159, 2004.

[^Campbell08]: Neill D.F. Campbell, George Vogiatzis, Carlos Hernández, and Roberto Cipolla. Using multiple hypotheses to improve depth-maps for multi-view stereo. In 10th European Conference on Computer Vision, volume 5302 of LNCS, pages 766–779, 2008.

[^Gallup07]: David Gallup, Jan-Michael Frahm, Philippos Mordohai, Qingxiong Yang, and Marc Pollefeys. Real-time plane-sweeping stereo with multiple sweeping directions. In IEEE Conference on Computer Vision and Pattern Recognition, 2007.

[^Woodford08]: Woodford, O. J., Torr, P. H. S., Reid, I. D., & Fitzgibbon, A. W. (2008). Global stereo reconstruction under second order smoothness priors. 2008 IEEE Conference on Computer Vision and Pattern Recognition. 

[^Curless96]: Brian Curless and Marc Levoy. A volumetric method for building complex models from range images. In ACM SIGGRAPH, 1996.

[^Zach07]: C. Zach, T. Pock, and H. Bischof. A globally optimal algorithm for robust tv-l 1 range image integration. In IEEE International Conference on Computer Vision, 2007.

[^Sinha07]: S.N. Sinha, P. Mordohai, and M. Pollefeys. Multi-view stereo via graph cuts on the dual of an adaptive tetrahedral mesh. In IEEE International Conference on Computer Vision, pages 1–8, 2007.

[^Labatut07]: Patrick Labatut, Jean-Philippe Pons, and Renaud Keriven. Efficient multi-view reconstruction of large-scale scenes using interest points, delaunay triangulation and graph cuts. In IEEE International Conference on Computer Vision, 2007.

[^Jancosek11]: M. Jancosek and T. Pajdla. Multi-view reconstruction preserving weakly-supported surfaces. In IEEE Conference on Computer Vision and Pattern Recognition, 2011.

[^Vu12]: H-H. Vu, P. Labatut, R. Keriven, and J.-P Pons. High accuracy and visibility-consistent dense multi-view stereo. IEEE Transactions on Pattern Analysis and Machine Intelligence, 34(5):889–901, May 2012.

[^Botsch08]: Mario Botsch and Olga Sorkine. On linear variational surface deformation methods. IEEE Transactions on Visualization and Computer Graphics, 14(1):213–230, 2008.