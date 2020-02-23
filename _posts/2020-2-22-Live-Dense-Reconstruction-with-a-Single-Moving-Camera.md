---
layout:     post
title:      "Live Dense Reconstruction with a Single Moving Camera [2010]"
date:       2020-2-22
author:     Tong
catalog: true
tags:
    - SLAM
---

### Abstract

We present a method which enables rapid and dense reconstruction of scenes browsed by a single live camera. We take point-based real-time structure from motion (SFM) as our starting point, generating accurate 3D camera pose estimates and a sparse point cloud. Our main novel contribution is to use an approximate but smooth base mesh generated from the SFM to predict the view at a bundle of poses around automatically selected reference frames spanning the scene, and then warp the base mesh into highly accurate depth maps based on view-predictive optical flow and a constrained scene flow update. The quality of the resulting depth maps means that a convincing global scene model can be obtained simply by placing them side by side and removing overlapping regions. We show that a cluttered indoor environment can be reconstructed from a live hand-held camera in a few seconds, with all processing performed by current desktop hardware. Real-time monocular dense reconstruction opens up many application areas, and we demonstrate both real-time novel view synthesis and advanced augmented reality where augmentations interact physically with the 3D scene and are correctly clipped by occlusions.


### Contribution

- In the approach we propose, real-time SFM is first used to estimate live camera pose and provide a 3D feature point cloud to which we fit an initial base surface approximating the real scene.

- The key to dense reconstruction is precise dense correspondence between images offering sufficient baseline for triangulation, and the main novelty of our approach is the manner in which this is enabled. In a similar manner to [^Vogiatzis2008] we utilise a coarse base surface model as the initial starting point for dense reconstruction. Our base surface permits view predictions to be made between the cameras in a local bundle, and these predictions are then compared with the true images using GPU-based variational optical flow. This permits dense correspondence fields of sub-pixel accuracy to be obtained, even in areas of very low image texture, and this correspondence information is then used to update the base surface prior into highly accurate local depth maps using constrained scene flow optimisation. Depth map creation is pipelined, and multiple depth maps are straightforwardly fused to create complete scene reconstructions.

### Overview

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/live_dense_reconstruction_pipeline_overview.png)

### Structure from Motion

- We make a rough initial surface normal estimate for each point by averaging the optic axis directions of the key-frames in which it is visible.

### Base Surface Construction

- A reconstructed mesh is extracted using the method of [^Bloomenthal1994].

- We use a state of the art multi-scale compactly supported radial basis function (MSCSRBF) technique for 3D scattered data interpolation [^Ohtake2003].

### Constrained Scene Flow Dense Reconstruction

#### Scene Motion and Image Motion

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

#### Optimising the Base Mesh

- We wish to use image correspondences information to obtain the vertex updates $$\Delta \mathbf{x}_{j}$$ required to deform each vertex of the current base model $$\mathbf{x}_{j}$$ into a new estimate $$\mathbf{x}_{j}^{\prime}$$ accurately representing the true scene.

#### Triangulating the Depth Map

#### Surface Errors

- $$E^{s}$$: The per vertex mean reprojection error

- $$E^{v}$$: A measure of the visibility of the surface element in the reference view

$$
E^{s}=\left\|\mathbf{K}_{j} \lambda_{j}-\Delta \mathbf{u}_{j}\right\|_{2}, E^{v}=\left|\mathbf{r}_{j}^{i} \cdot \mathbf{n}_{j}^{i}\right|
$$

### High Quality Dense Correspondence

- Our image pairs, each consisting of the reference image and a comparison view, are relatively short baseline. We therefore make use of high accuracy, variational optimisation based optical algorithm to obtain dense, sub-pixel quality correspondences.

#### View-Predictive Optical Flow Correspondence

- Our dense base mesh provides not only a prediction of the position and normal of a visible element at a given frame, but also of every pixel’s intensity. We back-project the reference frame image onto the base model surface and project the textured model into the comparison camera frame to synthesize an image prediction in that camera. This is performed efficiently on GPU hardware using projective texturing.

#### Minimally Destructive View Prediction  

#### Iterating Model Prediction

### Local Model Integration

- Two classes of algorithms to enable the fusion of multiple depth maps
    - Make use of the computed depth maps, dense oriented point samples, or volumetric signed distance functions computed from the depth maps to fit a globally consistent function that is then polygonised.
    - Work directly on the partial mesh reconstructions.

- Approach in this paper: Given a newly triangulated mesh obtained at reference $$P^{r e f}$$, we render a depth map of the currently integrated dense reconstruction into $$P^{r e f}$$ and remove the vertices in the new model where the distance to the current vertex is within $$\epsilon_{d i s t}$$ of the new depth value. We also remove less visible vertices with high solution error in the new mesh where  $$E^{v}<0.9$$and $$E^{s}>1 e^{-3}$$.

### Camera Bundle Selection

### Literature

[^Vogiatzis2008]: G. Vogiatzis, P. H. S. Torr, S. M. Seitz, and R. Cipolla. Reconstructing relief surfaces. Image and Vision Computing (IVC), 26(3):397–404, 2008.

[^Bloomenthal1994]: J. Bloomenthal. An implicit surface polygonizer. In Graphics Gems IV, pages 324–349. Academic Press, 1994.

[^Ohtake2003]: Y. Ohtake, A. Belyaev, and H.-P. Seidel. A multi-scale approach to 3D scattered data interpolation with compactly supported basis functions. In Proceedings of Shape Modeling International, 2003.

[^Vedula1999]: S. Vedula, S. Baker, P. Rander, R. Collins, and T. Kanade. Three-dimensional scene flow. In Proceedings of the International Conference on Computer Vision (ICCV), 1999
