---
layout:     post
title:      "Monocular Reconstruction"
date:       2020-3-6
author:     Tong
catalog: true
tags:
    - SLAM
---

### Superpixel Soup: Monocular Dense 3D Reconstruction of a Complex Dynamic Scene[^Kumar2019]

#### Abstract

This work addresses the task of dense 3D reconstruction of a complex dynamic scene from images. The prevailing idea to solve this task is composed of a sequence of steps and is dependent on the success of several pipelines in its execution [^Ranftl2016]. To overcome such limitations with the existing algorithm, we propose a unified approach to solve this problem. We assume that a dynamic scene can be approximated by numerous piecewise planar surfaces, where each planar surface enjoys its own rigid motion, and the global change in the scene between two frames is as-rigid-as-possible (ARAP). Consequently, our model of a dynamic scene reduces to a _soup_ of planar structures and rigid motion of these local planar structures. Using planar over-segmentation of the scene, we reduce this task to solving a “3D jigsaw puzzle” problem. Hence, the task boils down to correctly assemble each rigid piece to construct a 3D shape that complies with the geometry of the scene under the ARAP assumption. Further, we show that our approach provides an effective solution to the inherent scale-ambiguity in structure-from-motion under perspective projection. We provide extensive experimental results and evaluation on several benchmark datasets. Quantitative comparison with competing approaches shows state-of-the-art performance.

#### Contributions

- Our method, _SuperPixelSoup_ algorithm, is built upon two basic assumptions about the scene.
    - The dynamic scene can be approximated by a collection of _piecewise planar surfaces_ each having its own _rigid motion_.
    - The deformation of the scene between two frames is _locally-rigid_, but _globally as-rigid-as-possible_.

- The main contributions of this work are:
    - A framework which disentangles object-level motion segmentation for dense 3D reconstruction of a complex dynamic scene.
    - A common framework for dense two-frame 3D reconstruction of a complex dynamic scene (including deformable objects), which achieves state-of-the-art performance.
    - A new idea to resolve the inherent relative scale ambiguity problem in monocular 3D reconstruction by exploiting the as-rigid-as-possible (ARAP) constraint [^Sorkine2007].

#### Algorithm: SuperPixel Soup

__Input__: Two consecutive image frames of a dynamic scene and dense optical flow correspondences between them.

__Output__: 3D reconstruction of both images.

1. Divide the image into $$N$$ superpixel[^Achanta2012] and construct a K-NN graph to represent the entire scene as a graph $$G(V, E)$$ defined over superpixels.
2. Employ the two-view epipolar geometry to recover the rigid motion and shape for each 3D superpixel.
3. Optimize the proposed energy function to assemble (or glue) and align all the  reconstructed superpixels (“3D Superpixel Jigsaw Puzzle”)

#### Solution

- Build a K-NN graph
    - We identify a 3D superpixel by its anchor point. The distance between two 3D superpixels is defined as the Euclidean distance between their anchor points in 3D space. By connecting K nearest neighbors, we build a K-NN graph G(V,E). The graph vertices are anchor points, connecting with each other via graph edges.

- As-Rigid-As-Possible (ARAP) Energy Term.

- Planar Re-projection Energy Term.

- 3D Continuity Energy Term.

- Combined Energy Function.

#### Implementation

We partition the reference image using SLIC superpixels [^Achanta2012]. We used the current  state-of-the-art optical flow algorithm to compute dense optical flow [^Bailer2015]. To initialize the motion and geometry variables, we used the the procedure discussed in §4.5.1. Interior point algorithm [^Benson2002][^Benson2014] and TRW-S [^Kolmogorov2006] were employed to solve the proposed optimization.

#### Limitations

- The success of our method depends on the effectiveness of the piece-wise planar and as rigid as possible assumption. As a result, our method may fail if the piece-wise smooth model is no longer a valid approximation for the dynamic scene.
    - For example, very fine or very small structures which are considerably far from the camera are difficult to recover under the piecewise planar assumption.
    - Further, what about as rigid as possible assumption, _when as rigid as possible assumption may fail?_ When the motion of the dynamic objects between consecutive frame is significantly large such that most of its neighboring relations in the reference frame get violated in the next frame.
    - Additionally, if the non-rigid shape shrinks or expands over frames such as a _deflating or inflating balloon_, ARAP model fails.


### REMODE: Probabilistic, Monocular Dense Reconstruction in Real Time[^Pizzoli2014]

> https://github.com/uzh-rpg/rpg_open_remode

#### Abstract

In this paper, we solve the problem of estimating dense and accurate depth maps from a single moving camera.
A probabilistic depth measurement is carried out in real time on a per-pixel basis and the computed uncertainty is used to reject erroneous estimations and provide live feedback on the reconstruction progress. Our contribution is a novel approach to depth map computation that combines Bayesian estimation and recent development on convex optimization for image processing. We demonstrate that our method outperforms state of-the-art techniques in terms of accuracy, while exhibiting high efficiency in memory usage and computing power. We call our approach REMODE (REgularized MOnocular Depth Estimation) and the CUDA-based implementation runs at 30Hz on a laptop computer.


#### Contributions

- A probabilistic depth map, in which the Bayesian scheme in [^Vogiatzis2011] is integrated in a monocular SLAM algorithm to estimate per-pixel depths based on the live camera stream.

- A fast smoothing method that takes into account the measurement uncertainty to provide spatial regularity and mitigates the effect of noisy camera localization.

#### Smoothing depth map

$$
\min _{F} \int_{\Omega}\left\{G(\mathbf{u})\|\nabla F(\mathbf{u})\|_{\epsilon}+\lambda\|F(\mathbf{u})-D(\mathbf{u})\|_{1}\right\} d \mathbf{u},
$$

where $$D(\mathbf{u})$$ is the depth map to smooth, $$F(\mathbf{u})$$ is the de-noised depth map, $$G(\mathbf{u})$$ is a weighting function related to the "G-Weighted Total Variation", introduced in [^Bresson2007] in the context of image segmentation.

### Live Dense Reconstruction with a Single Moving Camera[^Newcombe2010]

#### Abstract

We present a method which enables rapid and dense reconstruction of scenes browsed by a single live camera. We take point-based real-time structure from motion (SFM) as our starting point, generating accurate 3D camera pose estimates and a sparse point cloud. Our main novel contribution is to use an approximate but smooth base mesh generated from the SFM to predict the view at a bundle of poses around automatically selected reference frames spanning the scene, and then warp the base mesh into highly accurate depth maps based on view-predictive optical flow and a constrained scene flow update. The quality of the resulting depth maps means that a convincing global scene model can be obtained simply by placing them side by side and removing overlapping regions. We show that a cluttered indoor environment can be reconstructed from a live hand-held camera in a few seconds, with all processing performed by current desktop hardware. Real-time monocular dense reconstruction opens up many application areas, and we demonstrate both real-time novel view synthesis and advanced augmented reality where augmentations interact physically with the 3D scene and are correctly clipped by occlusions.


#### Contribution

- In the approach we propose, real-time SFM is first used to estimate live camera pose and provide a 3D feature point cloud to which we fit an initial base surface approximating the real scene.

- The key to dense reconstruction is precise dense correspondence between images offering sufficient baseline for triangulation, and the main novelty of our approach is the manner in which this is enabled. In a similar manner to [^Vogiatzis2008] we utilise a coarse base surface model as the initial starting point for dense reconstruction. Our base surface permits view predictions to be made between the cameras in a local bundle, and these predictions are then compared with the true images using GPU-based variational optical flow. This permits dense correspondence fields of sub-pixel accuracy to be obtained, even in areas of very low image texture, and this correspondence information is then used to update the base surface prior into highly accurate local depth maps using constrained scene flow optimisation. Depth map creation is pipelined, and multiple depth maps are straightforwardly fused to create complete scene reconstructions.

#### Overview

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/live_dense_reconstruction_pipeline_overview.png?token=AEVZO3PH2J2VTSC2OIM4OPS6NRNDM)

#### Structure from Motion

- We make a rough initial surface normal estimate for each point by averaging the optic axis directions of the key-frames in which it is visible.

#### Base Surface Construction

- A reconstructed mesh is extracted using the method of [^Bloomenthal1994].

- We use a state of the art multi-scale compactly supported radial basis function (MSCSRBF) technique for 3D scattered data interpolation [^Ohtake2003].

#### Constrained Scene Flow Dense Reconstruction

##### Scene Motion and Image Motion

- A small 3D motion of surface point $$\Delta \mathbf{x}_{j}$$ leads to a new point position $$\mathbf{x}_{j}^{\prime}$$ with corresponding projection $$\mathbf{u}_{j}^{\prime i}$$. If is sufficiently small, then image displacement $$\Delta \mathbf{u}_{j}^{i}=\mathbf{u}_{j}^{\prime i}-\mathbf{u}_{j}^{i}$$ can be approximated by the first order linearisation of $$\mathbf{P}^{i}$$ around $$\mathbf{x}_{j}$$. In this case, $$\Delta \mathbf{x}_{j}$$ is called scene flow[^Vedula1999]:
$$
\Delta \mathbf{u}_{j}^{i}=\mathbf{J}_{\mathbf{x}_{j}}^{i} \Delta \mathbf{x}_{j}
$$

$$
\left.\left.\mathbf{J}_{\mathrm{x}_{j}}^{i} \equiv \frac{\partial \mathbf{P}^{i}}{\partial \mathbf{x}}\right|_{\mathbf{x}_{j}} \equiv\left[\begin{array}{ccc}
{\frac{\partial \mathbf{P}_{u}^{i}}{\partial x}} & {\frac{\partial \mathbf{P}_{u}^{i}}{\partial y}} & {\frac{\partial \mathbf{P}_{u}^{i}}{\partial z}} \\
{\frac{\partial \mathbf{P}_{v}^{i}}{\partial x}} & {\frac{\partial \mathbf{P}_{v}^{i}}{\partial y}} & {\frac{\partial \mathbf{P}_{v}^{i}}{\partial z}}
\end{array}\right]\right|_{\mathbf{x}_{j}}
$$

##### Optimising the Base Mesh

- We wish to use image correspondences information to obtain the vertex updates $$\Delta \mathbf{x}_{j}$$ required to deform each vertex of the current base model $$\mathbf{x}_{j}$$ into a new estimate $$\mathbf{x}_{j}^{\prime}$$ accurately representing the true scene.

##### Triangulating the Depth Map

##### Surface Errors

- $$E^{s}$$: The per vertex mean reprojection error

- $$E^{v}$$: A measure of the visibility of the surface element in the reference view

$$
E^{s}=\left\|\mathbf{K}_{j} \lambda_{j}-\Delta \mathbf{u}_{j}\right\|_{2}, E^{v}=\left|\mathbf{r}_{j}^{i} \cdot \mathbf{n}_{j}^{i}\right|
$$

#### High Quality Dense Correspondence

- Our image pairs, each consisting of the reference image and a comparison view, are relatively short baseline. We therefore make use of high accuracy, variational optimisation based optical algorithm to obtain dense, sub-pixel quality correspondences.

##### View-Predictive Optical Flow Correspondence

- Our dense base mesh provides not only a prediction of the position and normal of a visible element at a given frame, but also of every pixel’s intensity. We back-project the reference frame image onto the base model surface and project the textured model into the comparison camera frame to synthesize an image prediction in that camera. This is performed efficiently on GPU hardware using projective texturing.

##### Minimally Destructive View Prediction  

##### Iterating Model Prediction

#### Local Model Integration

- Two classes of algorithms to enable the fusion of multiple depth maps
    - Make use of the computed depth maps, dense oriented point samples, or volumetric signed distance functions computed from the depth maps to fit a globally consistent function that is then polygonised.
    - Work directly on the partial mesh reconstructions.

- Approach in this paper: Given a newly triangulated mesh obtained at reference $$P^{r e f}$$, we render a depth map of the currently integrated dense reconstruction into $$P^{r e f}$$ and remove the vertices in the new model where the distance to the current vertex is within $$\epsilon_{d i s t}$$ of the new depth value. We also remove less visible vertices with high solution error in the new mesh where  $$E^{v}<0.9$$and $$E^{s}>1 e^{-3}$$.

#### Camera Bundle Selection

### Dense Monocular Depth Estimation in Complex Dynamic Scenes[^Ranftl2016]

#### Abstract

We present an approach to dense depth estimation from a single monocular camera that is moving through a dynamic scene. The approach produces a dense depth map from two consecutive frames. Moving objects are reconstructed along with the surrounding environment. We provide a novel motion segmentation algorithm that segments the optical flow field into a set of motion models, each with its own epipolar geometry. We then show that the scene can be reconstructed based on these motion models by optimizing a convex program. The optimization jointly reasons about the scales of different objects and assembles the scene in a common coordinate frame, determined up to a global scale. Experimental results demonstrate that the presented approach outperforms prior methods for monocular depth estimation in dynamic scenes.

#### Overview

- Our approach comprises two stages.
    - The first stage performs motion segmentation. This stage segments the dynamic scene into a set of motion models, each described by its own epipolar geometry. This enables reconstruction of each component of the scene up to an unknown scale. We propose a novel motion segmentation algorithm that is based on a convex relaxation of Potts model[^Chambolle2012]. Our algorithm supports dense segmentation of complex dynamic scenes into possibly dozens of independently moving components.
    - The second stage assembles the scene in a common metric frame by jointly reasoning about the scales of different components and their location relative to the camera. The main insight is that moving objects do not exist in a vacuum, but fulfill intrinsic occluder-occludee relationships with respect to each other and the static environment. This can be used to reason about the placement of different objects in the scene. We formulate this reconstruction problem as continuous optimization over scales and depths and introduce ordering and connectivity constraints to assemble the scene. The result is a reconstruction of the dynamic scene from only two frames, determined up to a single global scale.

- The proposed pipeline consists of two major stages.
    - First, the scene is segmented into a set of epipolar motion models. The segmentation is performed on optical flow and is formulated as a variational labeling problem.
    - The second stage performs triangulation and joint reconstruction of all objects. The key assumption in the second stage is that the scene consists of objects that are connected in space. In particular, we assume that dynamic objects are connected to the surrounding environment.

#### Motion Segmentation

- The task of the motion segmentation stage is to decompose the dynamic scene into a set of independent rigid motions, each described by a fundamental matrix, together with a per-pixel assignment to these motion models.

- Note that this approach automatically oversegments non-rigid objects into approximately rigid parts. We estimate the number of independent rigid motions as part of the global optimization to ensure that non-rigid motions are approximated well.

- We propose a new approach that efficiently handles dense correspondence fields, as produced by dense optical flow estimation. Our approach is most closely related to the discrete energy based multiple model fitting approach of Isack and Boykov [^Isack2012], but operates on soft assignments and models the data association as a continuous convex problem. This allows us to leverage recent advances in convex optimization [^Chambolle2015] and enables an efficient GPU-based implementation.

- We formulate this as a joint labeling and estimation problem, where we additionally exploit the fact that nearby pixels are likely to belong the same motion model.
$$
\begin{aligned}\left(u_{l}^{*}, F_{l}^{*}\right)=\arg \min _{u_{l}, F_{l}} & \sum_{l=1}^{L+1} u_{l} \cdot g\left(F_{l}\right)+\left\|W_{l} \nabla u_{l}\right\|_{2,1}(2) \\ \text { subject to } & \sum_{l=1}^{L+1} u_{l}^{i}=1, u_{l}^{i} \geq 0  \quad (\mathrm{SPX})\\ & \forall l \cdot \operatorname{rank}\left(F_{l}\right)=2 \quad (\mathrm{EPI}) \end{aligned}
$$

- We compute the symmetric distance to the epipolar lines for each model $$l \in\{1 \ldots L\}$$:
$$
g^{i}\left(F_{l}\right)=d\left(x_{1}^{i}, F_{l} x_{2}^{i}\right)^{2}+d\left(x_{2}^{i}, F_{l}^{\top} x_{1}^{i}\right)^{2}
$$

- We use a diagonal weighting matrix $$W_{l}$$ to enable edge-preserving regularization based on the reference image $$I_{1}$$.
$$
W_{l}=\operatorname{diag}\left(\exp \left(-\beta\left\|\nabla I_{1}\right\|^{2}\right)\right)
$$

- The simplex constraint (SPX) ensures that the soft assginments sum to one at each pixel, thus $$u_{l}^{i} \in[0,1]$$ can be interpreted as the probability that pixel $$i$$ belongs to the motion model $$F_{l}$$.

- The matrices $$F_{l} \in \mathbb{R}^{3 \times 3}$$ encapsulate the epipolar geometry of the pixel belonging to the segment $$l$$. The rank constraints (EPI) ensure that each $$F_{l}$$ is a fundamental matrix.

- In order to discover the number of motion models we opt for a simple greedy bootstrapping strategy, where we make extensive use of the outlier label to mine potential fundamental matrices from the data.
    - We start by mining a small set of candidate motions by iteratively applying the normalized 8-point algorithm in a robust least-median-of-squares (LMedS) framework [^Torr1998].
    - Based on this initialization, we solve energy (2) using the previously described alternating minimization approach until no further decrease in energy can be made.
    - We then expand the pool of candidate motions. New models are added by robustly estimating motion models from pixels that have the outlier label as their most probable assignment. Specifically, we robustly fit a motion model to each connected component of the pixels assigned to the outlier label.
    - We further expand the pool by splitting labels with disconnected regions and fitting motion models to these regions if the size of the region is larger than a threshold T. (Note that we do not remove the original models from the set of models.)
    - We again perform alternating minimization based on this new label set and repeat this process until no further decrease in energy can be made.

- The result of the motion segmentation stage is a set of epipolar geometries $$F_{l}^{*}$$ as well as membership probabilities $$u_{l}^{*}$$ for each pixel. We obtain the final pixel-to-model associations by extracting the label with maximum probability from $$u_{l}^{*}$$ to get $$\hat{u}_{l}$$:

$$
\hat{u}_{l}^{i}=\left\{\begin{array}{ll}{1} & {\text { if } \quad l=\max _{l \in\{1, \ldots, L+1\}}\left(u_{l}^{*}\right)^{i}} \\ {0} & {\text { else. }}\end{array}\right. \quad (6)
$$

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/algorithm_motion_segmentation.png?token=AEVZO3ICRHJ3XROSNX34G2K6PCEGQ)


#### Reconstruction

- We use a superpixel-based formulation in order to robustly reconstruct the dynamic scene from optical flow correspondences and the epipolar models estimated.

- We begin by triangulating each correspondence that was not labeled an outlier by the motion segmentation stage using its associated motion model $$F_{l}^{*}$$ . This yields a set of depth estimates $$z_{l} \in \mathbb{R}^{M}$$. Note that each depth estimate is only valid for pixel $$i$$ with $$\hat{u}_{l}^{i}=1$$. We set pixels that belong to the segment with largest support as environment pixels and fix their scales to 1.

- We now estimate the relative scales between all segments. This cannot be done without additional prior assumptions as the problem is ill-posed in general. To resolve scale ambiguities and assemble the scene, we use a prior assumption that is often appropriate in daily life: objects are supported by their environment. We model this assumption using a combination of two constraints:
    - An ordering constraint, which captures the assumption that dynamic objects occlude the static environment. This can be expressed by requiring the inverse depth of segments belonging to the dynamic objects to be larger or equal to the inverse depth of the environment in their immediate vicinity.
    2. A smoothness constraint, which states that jumps in inverse depth between dynamic objects and segments belonging to the environment should be minimized. This constraint connects the dynamic objects with the environment, subject to the ordering constraint.

- In order to be robust to outliers in the input data, we formulate these constraints as an energy minimization problem defined on a superpixel graph. Consider a superpixel segmentation of the reference image into $$K$$ segments. That is, each pixel $$i$$ is assigned to one of $$K$$ superpixels. We formally write $$i \in P_{k}$$ for the set of pixels belonging to superpixel $$k$$ and denote the edges in the superpixel graph by E. We use Quickshift [^Vedaldi2008] to produce a superpixel segmentation and break up superpixels that straddle boundaries in the motion segmentation.

- Our goal is to estimate a plane for each superpixel $$k$$ with parameters $$\theta_{k}=\left[\theta_{k}^{1}, \theta_{k}^{2}, \theta_{k}^{3}\right]^{\top}$$ and scales $$
s=\left[1, s_2, \dots, s_{L}\right]^{\top}$$ for all independently moving objects, subject to the previously described constraints. This is formulated as a convex optimization problem with the following objective:
$$
E(s, \theta)=E_{\mathrm{ord}}(\theta)+E_{\mathrm{sm}}(\theta)+E_{\mathrm{fit}}(s, \theta)
$$

### Coarse-to-fine planar regularization for dense monocular depth estimation[^Liwicki2016]

#### Abstract

Simultaneous localization and mapping (*SLAM*) using the whole image data is an appealing framework to address shortcoming of sparse feature-based methods { in particular frequent failures in textureless environments. Hence, direct methods bypassing the need of feature extraction and matching became recently popular. Many of these methods operate by alternating between pose estimation and computing (semi-)dense depth maps, and are therefore not fully exploiting the advantages of joint optimization with respect to depth and pose. In this work, we propose a framework for monocular SLAM, and its local model in particular, which optimizes simultaneously over depth and pose. In addition to a planarity enforcing smoothness regularizer for the depth we also constrain the complexity of depth map updates, which provides a natural way to avoid poor local minima and reduces unknowns in the optimization. Starting from a holistic objective we develop a method suitable for online and real-time monocular SLAM. We evaluate our method quantitatively in pose and depth on the TUM dataset, and qualitatively on our own video sequences.

#### Contributions

1. we formulate a global energy for planar regularized inverse depth that is optimized iteratively at each frame,
2. we revisit depth and pose optimization normally considered separately, and introduce a coarse-to-ne strategy that refines both truly simultaneously,
3. we establish our method as semi-dense, andnd pose and depth twice as fast as LSD-SLAM, by adding minimal cost to LSD-SLAM's tracking thread,
4. we evaluate pose and depth quantitatively on the TUM dataset.


### FLaME: Fast Lightweight Mesh Estimation using Variational Smoothing on Delaunay Graphs [^Greene2017]

#### Abstract

We propose a lightweight method for dense online monocular depth estimation capable of reconstructing 3D meshes on computationally constrained platforms. Our main contribution is to pose the reconstruction problem as a non-local variational optimization over a time-varying Delaunay graph of the scene geometry, which allows for an efficient, keyframeless approach to depth estimation. The graph can be tuned to favor reconstruction quality or speed and is continuously smoothed and augmented as the camera explores the scene. Unlike keyframe-based approaches, the optimized surface is always available at the current pose, which is necessary for low-latency obstacle avoidance.

#### Introduction

Instead of a dense every-pixel approach, we propose a novel alternative that we call FLaME (Fast Lightweight Mesh Estimation) that directly estimates a triangular mesh of the scene (similar to the stereo work of [^Pillai2016]) and is advantageous for several reasons. 
    - First, meshes are more compact, efficient representations of the geometry and therefore require fewer depth estimates to encode the scene for a given level of detail. 
    - Second, by interpreting the mesh as a graph we show that we can exploit its connectivity structure to apply (and accelerate) state-of-the-art second-order variational regularization techniques that otherwise require GPUs to run online. 
    - Third, by reformulating the regularization objective in terms of the vertices and edges of this graph, we allow the smoothing optimization to be both incremental (in that new terms can be trivially added and removed as the graph is modified over time) and keyframeless (in that the solution can be easily propagated in time without restarting the optimization).

#### Algorithm Overview

FLaME directly estimates an inverse depth mesh of the environment that efficiently encodes the scene geometry and allows for efficient, incremental, and keyframeless second-order variational regularization to recover smooth surfaces. Given an image sequence $$I_k$$ from a moving camera with known pose $$T^{W}_{k}$$, our task entails:
- Estimating the inverse depth for a set of sampled pixels
- Constructing the mesh using the sampled points
- Defining a suitable smoothness cost over the graph induced by the mesh 
- Minimizing the smoothness cost[^Chambolle2011]
- Projecting the mesh from frame to frame

FLaME Overview: FLaME operates on image streams with known poses. 
- Inverse depth is estimated for a set of features using the fast, filtering-based approach of [^Engel2013]. 
- When the inverse depth estimate for a given feature converges, it is inserted as a new vertex in a graph defined in the current frame and computed through Delaunay triangulations[^Shewchuk1996][^Shewchuk2002]. 
- This Delaunay graph is then used to efficiently smooth away noise in the inverse depth values and promote piecewise planar structure by minimizing a second-order variational cost defined over the graph.    

### Dense Mono Reconstruction: Living with the Pain of the Plain Plane [^Pinies2015]

#### Abstract

This paper is about dense depthmap estimation using a monocular camera in workspaces with extensive textureless surfaces. Current state of the art techniques have been shown to work in real time with an admirable performance in desktop-size environments. Unfortunately, as we show in this paper, when applied to larger indoor environments, performance often degrades. A common cause is the presence of large affine texture-less areas like by walls, floors, ceilings and drab objects such as chairs and tables. These produce noisy and worse still, grossly erroneous initial seeds for the depthmap that greatly impede successful optimisation. 

We solve this problem via the introduction of a new nonlocal higher-order regularisation term that enforces piecewise affine constraints between image pixels that are far apart in the image. This property leverages the observation that the depth at the edges of bland regions are often well estimated whereas their inner pixels are deeply problematic. A welcome by-product of our proposed technique is an estimate of the surface normals at each pixel. We will show that in terms of implementation, our algorithm is a natural extension of the often used variational approaches. We evaluate the proposed technique using real datasets for which we have ground truth models.


### Literature

[^Achanta2012]: R. Achanta, A. Shaji, K. Smith, A. Lucchi, P. Fua, and S. Susstrunk. SLIC superpixels compared to state-of-the-art superpixel methods. IEEE transactions on pattern analysis and machine intelligence, 34(11):2274–2282, 2012.

[^Bailer2015]: C. Bailer, B. Taetz, and D. Stricker, “Flow fields: Dense correspondence fields for highly accurate large displacement optical flow estimation,” in CVPR, 2015, pp. 4015–4023.

[^Benson2002]: H. Y. Benson, R. J. Vanderbei, and D. F. Shanno, “Interior-point methods for nonconvex nonlinear programming: Filter methods and merit functions,” Computational Optimization and Applications, vol. 23, no. 2, pp. 257–272, 2002.

[^Benson2014]: H. Y. Benson and D. F. Shanno, “Interior-point methods for non-convex nonlinear programming: cubic regularization,” Computational optimization and applications, vol. 58, no. 2, pp. 323–346, 2014.

[^Bloomenthal1994]: J. Bloomenthal. An implicit surface polygonizer. In Graphics Gems IV, pages 324–349. Academic Press, 1994.

[^Bresson2007]: SX. Bresson, S. Esedoglu, P. Vandergheynst, J.-P. Thiran, and S. Osher, “Fast global minimization of the active contour/snake model,” Journal of Mathematical Imaging and Vision, vol. 28, no. 2, 2007.

[^Chambolle2011]: A. Chambolle and T. Pock. A first-order primal-dual algorithm for convex problems with applications to imaging. Journal of Mathematical Imaging and Vision, 2011.

[^Chambolle2012]: A. Chambolle, D. Cremers, and T. Pock. A convex approach to minimal partitions. SIAM Journal on Imaging Sciences, 5(4), 2012.

[^Chambolle2015]: A. Chambolle and T. Pock. On the ergodic convergence rates of a first-order primal-dual algorithm. Mathematical Programming, 2015.

[^Engel2013]: J. Engel, J. Sturm, and D. Cremers. Semi-dense visual odometry for a monocular camera. In Proc. ICCV, 2013.

[^Greene2017]: Greene, W. Nicholas, and Nicholas Roy. "Flame: Fast lightweight mesh estimation using variational smoothing on delaunay graphs." 2017 IEEE International Conference on Computer Vision (ICCV). IEEE, 2017.

[^Isack2012]: H. Isack and Y. Boykov. Energy-based geometric multi-model fitting. IJCV, 97(2), 2012.

[^Kolmogorov2006]: V. Kolmogorov, “Convergent tree-reweighted message passing for energy minimization,” T-PAMI, vol. 28, no. 10, pp. 1568–1583, 2006.

[^Kumar2019]: Kumar, Suryansh, Yuchao Dai, and Hongdong Li. "Superpixel soup: Monocular dense 3d reconstruction of a complex dynamic scene." IEEE transactions on pattern analysis and machine intelligence (2019).

[^Liwicki2016]: Liwicki, Stephan, et al. "Coarse-to-fine planar regularization for dense monocular depth estimation." European Conference on Computer Vision. Springer, Cham, 2016.

[^Newcombe2010]: Newcombe, Richard A., and Andrew J. Davison. "Live dense reconstruction with a single moving camera." 2010 IEEE Computer Society Conference on Computer Vision and Pattern Recognition. IEEE, 2010.

[^Ohtake2003]: Y. Ohtake, A. Belyaev, and H.-P. Seidel. A multi-scale approach to 3D scattered data interpolation with compactly supported basis functions. In Proceedings of Shape Modeling International, 2003.

[^Pizzoli2014]: Pizzoli, Matia, Christian Forster, and Davide Scaramuzza. "REMODE: Probabilistic, monocular dense reconstruction in real time." 2014 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2014.

[^Pillai2016]: Pillai, Sudeep, Srikumar Ramalingam, and John J. Leonard. "High-performance and tunable stereo reconstruction." 2016 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2016.

[^Pinies2015]: P. Pinies, L. M. Paz, and P. Newman. Dense mono reconstruction: Living with the pain of the plain plane. In Proc. ICRA, 2015.

[^Ranftl2016]: R. Ranftl, V. Vineet, Q. Chen, and V. Koltun, “Dense monocular depth estimation in complex dynamic scenes,” in CVPR, 2016, pp.4058–4066.

[^Sorkine2007]: O. Sorkine and M. Alexa, “As-rigid-as-possible surface modeling,” in Symposium on Geometry processing, vol. 4, 2007.

[^Shewchuk1996]: J. R. Shewchuk. Triangle: Engineering a 2D Quality Mesh Generator and Delaunay Triangulator. In Applied Computational Geometry: Towards Geometric Engineering, 1996.

[^Shewchuk2002]: J. R. Shewchuk. Delaunay refinement algorithms for triangular mesh generation. Computational Geometry, 2002.s

[^Torr1998]: P. H. S. Torr. Geometric motion segmentation and model selection. Philosophical Transactions of the Royal Society A, 356(1740), 1998.

[^Vedula1999]: S. Vedula, S. Baker, P. Rander, R. Collins, and T. Kanade. Three-dimensional scene flow. In Proceedings of the International Conference on Computer Vision (ICCV), 1999

[^Vogiatzis2011]: G. Vogiatzis and C. Hernandez, _Video-based, real-time multi-view stereo_, Image and Vision Computing, vol. 29, no. 7, 2011.

[^Vogiatzis2008]: G. Vogiatzis, P. H. S. Torr, S. M. Seitz, and R. Cipolla. Reconstructing relief surfaces. Image and Vision Computing (IVC), 26(3):397–404, 2008.

[^Vedaldi2008]: Vedaldi and S. Soatto. Quick shift and kernel methods for mode seeking. In ECCV, 2008.

