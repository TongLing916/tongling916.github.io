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

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/live_dense_reconstruction_pipeline_overview.png)

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



### Literature

[^Kumar2019]: Kumar, Suryansh, Yuchao Dai, and Hongdong Li. "Superpixel soup: Monocular dense 3d reconstruction of a complex dynamic scene." IEEE transactions on pattern analysis and machine intelligence (2019).

[^Ranftl2016]: R. Ranftl, V. Vineet, Q. Chen, and V. Koltun, “Dense monocular depth estimation in complex dynamic scenes,” in CVPR, 2016, pp.4058–4066.

[^Achanta2012]: R. Achanta, A. Shaji, K. Smith, A. Lucchi, P. Fua, and S. Susstrunk. SLIC superpixels compared to state-of-the-art superpixel methods. IEEE transactions on pattern analysis and machine intelligence, 34(11):2274–2282, 2012.

[^Sorkine2007]: O. Sorkine and M. Alexa, “As-rigid-as-possible surface modeling,” in Symposium on Geometry processing, vol. 4, 2007.

[^Bailer2015]: C. Bailer, B. Taetz, and D. Stricker, “Flow fields: Dense correspondence fields for highly accurate large displacement optical flow estimation,” in CVPR, 2015, pp. 4015–4023.

[^Benson2002]: H. Y. Benson, R. J. Vanderbei, and D. F. Shanno, “Interior-point methods for nonconvex nonlinear programming: Filter methods and merit functions,” Computational Optimization and Applications, vol. 23, no. 2, pp. 257–272, 2002.

[^Benson2014]: H. Y. Benson and D. F. Shanno, “Interior-point methods for non-convex nonlinear programming: cubic regularization,” Computational optimization and applications, vol. 58, no. 2, pp. 323–346, 2014.

[^Kolmogorov2006]: V. Kolmogorov, “Convergent tree-reweighted message passing for energy minimization,” T-PAMI, vol. 28, no. 10, pp. 1568–1583, 2006.

[^Pizzoli2014]: Pizzoli, Matia, Christian Forster, and Davide Scaramuzza. "REMODE: Probabilistic, monocular dense reconstruction in real time." 2014 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2014.

[^Bresson2007]: SX. Bresson, S. Esedoglu, P. Vandergheynst, J.-P. Thiran, and S. Osher, “Fast global minimization of the active contour/snake model,” Journal of Mathematical Imaging and Vision, vol. 28, no. 2, 2007.

[^Vogiatzis2011]: G. Vogiatzis and C. Hernandez, _Video-based, real-time multi-view stereo_, Image and Vision Computing, vol. 29, no. 7, 2011.

[^Newcombe2010]: Newcombe, Richard A., and Andrew J. Davison. "Live dense reconstruction with a single moving camera." 2010 IEEE Computer Society Conference on Computer Vision and Pattern Recognition. IEEE, 2010.

[^Vogiatzis2008]: G. Vogiatzis, P. H. S. Torr, S. M. Seitz, and R. Cipolla. Reconstructing relief surfaces. Image and Vision Computing (IVC), 26(3):397–404, 2008.

[^Bloomenthal1994]: J. Bloomenthal. An implicit surface polygonizer. In Graphics Gems IV, pages 324–349. Academic Press, 1994.

[^Ohtake2003]: Y. Ohtake, A. Belyaev, and H.-P. Seidel. A multi-scale approach to 3D scattered data interpolation with compactly supported basis functions. In Proceedings of Shape Modeling International, 2003.

[^Vedula1999]: S. Vedula, S. Baker, P. Rander, R. Collins, and T. Kanade. Three-dimensional scene flow. In Proceedings of the International Conference on Computer Vision (ICCV), 1999
