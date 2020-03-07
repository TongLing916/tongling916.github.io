---
layout:     post
title:      "Monocular Depth Estimation"
date:       2020-3-6
author:     Tong
catalog: true
tags:
    - SLAM
---

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

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/algorithm_motion_segmentation.png)


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


### Literature

[^Ranftl2016]: Ranftl, Rene, et al. "Dense monocular depth estimation in complex dynamic scenes." Proceedings of the IEEE conference on computer vision and pattern recognition. 2016.

[^Chambolle2012]: A. Chambolle, D. Cremers, and T. Pock. A convex approach to minimal partitions. SIAM Journal on Imaging Sciences, 5(4), 2012.

[^Isack2012]: H. Isack and Y. Boykov. Energy-based geometric multi-model fitting. IJCV, 97(2), 2012.

[^Chambolle2015]: A. Chambolle and T. Pock. On the ergodic convergence rates of a first-order primal-dual algorithm. Mathematical Programming, 2015.

[^Torr1998]: P. H. S. Torr. Geometric motion segmentation and model selection. Philosophical Transactions of the Royal Society A, 356(1740), 1998.

[^Vedaldi2008]: Vedaldi and S. Soatto. Quick shift and kernel methods for mode seeking. In ECCV, 2008.

[^Liwicki2016]: Liwicki, Stephan, et al. "Coarse-to-fine planar regularization for dense monocular depth estimation." European Conference on Computer Vision. Springer, Cham, 2016.
