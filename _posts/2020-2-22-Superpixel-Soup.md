---
layout:     post
title:      "Superpixel Soup: Monocular Dense 3D Reconstruction of a Complex Dynamic Scene"
date:       2020-2-23
author:     Tong
catalog: true
tags:
    - SLAM
---

### Abstract

This work addresses the task of dense 3D reconstruction of a complex dynamic scene from images. The prevailing idea to solve this task is composed of a sequence of steps and is dependent on the success of several pipelines in its execution [^Ranftl2016]. To overcome such limitations with the existing algorithm, we propose a unified approach to solve this problem. We assume that a dynamic scene can be approximated by numerous piecewise planar surfaces, where each planar surface enjoys its own rigid motion, and the global change in the scene between two frames is as-rigid-as-possible (ARAP). Consequently, our model of a dynamic scene reduces to a _soup_ of planar structures and rigid motion of these local planar structures. Using planar over-segmentation of the scene, we reduce this task to solving a “3D jigsaw puzzle” problem. Hence, the task boils down to correctly assemble each rigid piece to construct a 3D shape that complies with the geometry of the scene under the ARAP assumption. Further, we show that our approach provides an effective solution to the inherent scale-ambiguity in structure-from-motion under perspective projection. We provide extensive experimental results and evaluation on several benchmark datasets. Quantitative comparison with competing approaches shows state-of-the-art performance.


### Contributions

- Our method, _SuperPixelSoup_ algorithm, is built upon two basic assumptions about the scene.
    - The dynamic scene can be approximated by a collection of _piecewise planar surfaces_ each having its own _rigid motion_.
    - The deformation of the scene between two frames is _locally-rigid_, but _globally as-rigid-as-possible_.

- The main contributions of this work are:
    - A framework which disentangles object-level motion segmentation for dense 3D reconstruction of a complex dynamic scene.
    - A common framework for dense two-frame 3D reconstruction of a complex dynamic scene (including deformable objects), which achieves state-of-the-art performance.
    - A new idea to resolve the inherent relative scale ambiguity problem in monocular 3D reconstruction by exploiting the as-rigid-as-possible (ARAP) constraint [^Sorkine2007].

### Algorithm: SuperPixel Soup

__Input__: Two consecutive image frames of a dynamic scene and dense optical flow correspondences between them.

__Output__: 3D reconstruction of both images.

1. Divide the image into $$N$$ superpixel[^Achanta2012] and construct a K-NN graph to represent the entire scene as a graph $$G(V, E)$$ defined over superpixels.
2. Employ the two-view epipolar geometry to recover the rigid motion and shape for each 3D superpixel.
3. Optimize the proposed energy function to assemble (or glue) and align all the  reconstructed superpixels (“3D Superpixel Jigsaw Puzzle”)

### Solution

- Build a K-NN graph
    - We identify a 3D superpixel by its anchor point. The distance between two 3D superpixels is defined as the Euclidean distance between their anchor points in 3D space. By connecting K nearest neighbors, we build a K-NN graph G(V,E). The graph vertices are anchor points, connecting with each other via graph edges.

- As-Rigid-As-Possible (ARAP) Energy Term.

- Planar Re-projection Energy Term.

- 3D Continuity Energy Term.

- Combined Energy Function.

### Implementation

We partition the reference image using SLIC superpixels [^Achanta2012]. We used the current  state-of-the-art optical flow algorithm to compute dense optical flow [^Bailer2015]. To initialize the motion and geometry variables, we used the the procedure discussed in §4.5.1. Interior point algorithm [^Benson2002][^Benson2014] and TRW-S [^Kolmogorov2006] were employed to solve the proposed optimization.

### Limitations

- The success of our method depends on the effectiveness of the piece-wise planar and as rigid as possible assumption. As a result, our method may fail if the piece-wise smooth model is no longer a valid approximation for the dynamic scene.
    - For example, very fine or very small structures which are considerably far from the camera are difficult to recover under the piecewise planar assumption.
    - Further, what about as rigid as possible assumption, _when as rigid as possible assumption may fail?_ When the motion of the dynamic objects between consecutive frame is significantly large such that most of its neighboring relations in the reference frame get violated in the next frame.
    - Additionally, if the non-rigid shape shrinks or expands over frames such as a _deflating or inflating balloon_, ARAP model fails.

### Literature

[^Ranftl2016]: R. Ranftl, V. Vineet, Q. Chen, and V. Koltun, “Dense monocular depth estimation in complex dynamic scenes,” in CVPR, 2016, pp.4058–4066.

[^Achanta2012]: R. Achanta, A. Shaji, K. Smith, A. Lucchi, P. Fua, and S. Susstrunk. SLIC superpixels compared to state-of-the-art superpixel methods. IEEE transactions on pattern analysis and machine intelligence, 34(11):2274–2282, 2012.

[^Sorkine2007]: O. Sorkine and M. Alexa, “As-rigid-as-possible surface modeling,” in Symposium on Geometry processing, vol. 4, 2007.

[^Bailer2015]: C. Bailer, B. Taetz, and D. Stricker, “Flow fields: Dense correspondence fields for highly accurate large displacement optical flow estimation,” in CVPR, 2015, pp. 4015–4023.

[^Benson2002]: H. Y. Benson, R. J. Vanderbei, and D. F. Shanno, “Interior-point methods for nonconvex nonlinear programming: Filter methods and merit functions,” Computational Optimization and Applications, vol. 23, no. 2, pp. 257–272, 2002.

[^Benson2014]: H. Y. Benson and D. F. Shanno, “Interior-point methods for non-convex nonlinear programming: cubic regularization,” Computational optimization and applications, vol. 58, no. 2, pp. 323–346, 2014.

[^Kolmogorov2006]: V. Kolmogorov, “Convergent tree-reweighted message passing for energy minimization,” T-PAMI, vol. 28, no. 10, pp. 1568–1583, 2006.
