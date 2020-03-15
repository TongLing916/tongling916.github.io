---
layout:     post
title:      "Conditional Random Field"
date:       2020-3-14
author:     Tong
catalog: true
tags:
    - Technique
---

### Efficient Inference in Fully Connected CRFs with Gaussian Edge Potentials [^Kraehenbuehl2011]

#### Abstract

Most state-of-the-art techniques for multi-class image segmentation and labeling use _conditional random fields_ defined over pixels or image regions. While region-level models often feature dense pairwise connectivity, pixel-level models are considerably larger and have only permitted sparse graph structures. In this paper, we consider fully connected CRF models defined on the complete set of pixels in an image. The resulting graphs have billions of edges, making traditional inference algorithms impractical. Our main contribution is a highly efficient approximate inference algorithm for fully connected CRF models in which the pairwise edge potentials are defined by a linear combination of Gaussian kernels. Our experiments demonstrate that dense connectivity at the pixel level substantially improves segmentation and labeling accuracy.

#### 1. Introduction

- Muti-class image segmentation and labeling
    - __Goal__: Label every pixel in the image with one of several predetermined object categories.

- A common approach is to pose this problem as maximum a posteriori (MAP) inference in a _conditional random field_ (CRF) defined over pixels or image pathces.
    - The CRF potentials incorporate smoothness terms that maximizes label agreement between similar pixels, and can integrate more elaborate terms that model contextual relationships between object classes.
    - Basic CRF models are composed of unary potentials on invividual pixels or image patches and pairwise potentials on neighboring pixels or patches. The resulting _adjacency CRF_ structure is limited in its ability to model long-range coneections within the image and generally results in excessive smoothing of object boundaries.

#### 2. Contribution

In this paper, we use a _fully connected CRF_ that establishes pairwise potentials on all pairs of pixels in the image. The main challenge is the size of the model, which has tens of thousands of nodes and billions of edges even on low-resolution images.

Our main contribution is a highly efficient inference algorithm for fully connected CRF models in which the pairwise edge potentials are defined by a linear combination of Gaussian kernels in an arbitrary feature space. The algorithm is based on a mean field approximation to the CRF distribution. This approximation is iteratively optimized through a series of message passing steps, each of which updates a single variable by aggregating information from all other variables. We show that a mean field update of all variables in a fully connected CRF can be performed using Gaussian filtering in feature space. This allows us to reduce the computational complexity of message passing from quadratic to linear in the number of variables by employing efficient approximate high-dimensional filtering [^Paris2006][^Adams2009][^Adams2010]. The resulting approximate inference algorithm is sublinear in the number of edges in the model.

### Literature

[^Kraehenbuehl2011]: Krähenbühl, Philipp, and Vladlen Koltun. "Efficient inference in fully connected crfs with gaussian edge potentials." Advances in neural information processing systems. 2011.

[^Paris2006]: Paris, Sylvain, and Frédo Durand. "A fast approximation of the bilateral filter using a signal processing approach." European conference on computer vision. Springer, Berlin, Heidelberg, 2006.

[^Adams2009]: Adams, Andrew, et al. "Gaussian kd-trees for fast high-dimensional filtering." ACM SIGGRAPH 2009 papers. 2009. 1-12.

[^Adams2010]: Adams, Andrew, Jongmin Baek, and Myers Abraham Davis. "Fast high‐dimensional filtering using the permutohedral lattice." Computer Graphics Forum. Vol. 29. No. 2. Oxford, UK: Blackwell Publishing Ltd, 2010.
