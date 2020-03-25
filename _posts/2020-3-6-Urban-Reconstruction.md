---
layout:     post
title:      "Urban Reconstruction"
date:       2020-3-6
author:     Tong
catalog: true
tags:
    - Reconstruction
---

### Automated as-built 3D reconstruction of civil infrastructure using computer vision: Achievements, opportunities, and challenges [^Fathi15]

#### Abstract

### A survey of urban reconstruction [^Musialski13]

#### Abstract

### Multi-view superpixel stereo in urban environments [^Micusik10]

#### Abstract 

Urban environments possess many regularities which can be efficiently exploited for 3D dense reconstruction from multiple widely separated views. We present an approach utilizing properties of piecewise planarity and restricted number of plane orientations to suppress reconstruction and matching ambiguities causing failures of standard dense stereo methods. We formulate the problem of the 3D reconstruction in MRF framework built on an image pre-segmented into superpixels. Using this representation, we propose novel photometric and superpixel boundary consistency terms explicitly derived from superpixels and show that they overcome many difficulties of standard pixelbased formulations and handle favorably problematic scenarios containing many repetitive structures and no or low textured regions. We demonstrate our approach on several wide-baseline scenes demonstrating superior performance compared to previously proposed methods.

#### Introduction

- The main contribution of this work is the way of encoding unique priors of the urban environments in a pipeline towards a 3D piecewise planar dense reconstruction. We choose superpixels as an image representation and suggest accordingly novel energy terms in Markov Random Field (MRF) formulation. Compared to standard pixel-based formulations we argue that the superpixels and their associated novel photometric and geometric consistency measures exhibit many advantageous properties and facilitate the dense stereo reconstruction significantly. Our proposed superpixel based formulation is beneficial for the urban environments in many aspects:
    1. It solves the ambiguities present in standard dense stereo methods at places with no or low texture. Those places are merged in superpixels and are treated as larger entities implicitly restricting their possible 3D positions. 
    2. It is more robust to camera misalignment enabling us to handle favorably wide-baseline settings with large illumination or camera exposure changes across views. Photoconsistency measure defined on a larger superpixel area outperforms standard measures on small square windows centered at individual pixels. Moreover, the shape of the superpixel boundary encodes information about possible orientation and can be favorably utilized in the MRF setting.
    3. Significant reduction of computational complexity can be achieved as the number of nodes in the graph built on superpixels is much smaller, by a factor of 1000, compared to graphs built on all image pixels.

#### Multi-View Stereo

We want to find $$\mathcal{P}^* = \left \{ {\prod}_s^* : s = 1, \cdots , S \right \}, \quad {\prod}_s = \left [ \mathbf{n}_s^T \quad d_s \right ]^T$$ as a Maximum Posterior Proabbility (MAP) assignment of a MRF, whose graph structure is induced by neighborhood relationships between the superpixels. Formally, we seek such $$\mathcal{P}^*$$ that 
$$\mathcal{P}^* = \text{arg} \min_{\mathcal{P}} \left [ \sum_{s}E_{\text{photo}} + \lambda_1 \sum_{s}E_{\text{bnd}} + \lambda_2 \sum_{\left \{  s,s^\prime \right \}}E_{\text{norm}} + \lambda_3 \sum_{\left \{  s,s^\prime \right \}}E_{\text{depth}}\right ]$$.

#### Proposed Solution

We formulate the problem as a discrete _labeling_ problem on a graph with fixed number of $$L$$ labels per superpixel obtained in the sweeping stage and one "don't know" label for ambiguous regions with low solution confidence.

A label $$l_s$$ for a given superpixel $$s$$ corresponds to a possible canddiate for depth and normal obtained in the sweeping stage.

1. Restriction on Normals
    - Urban scenes usually possess three dominant mutually orthogonal directions, thus fulfill a Manhattan world assumption, which can be captured by vanishing points detected in an image. 
    - We utilize only vanishing points as such an approach can be used even in situations where one does not have point correspondences between views and has only pose estimations.
2. Depth Candidates
    - The following procedure summarizes in details all the steps towards getting the depth and plane normal candidates for each superpixel. The procedure is done for each superpixel and a given normal independently.
        1. _Depth sweeping_.
        2. _Superpixel projections_. Note, that only the rim points are mapped.
        3. _Photometric normalization_. Photometrically normalize each superpixel such that their mean and variance in each color channel become 0 and 1, respectively.
        4. _Feature vectors_. 
        5. _Photoconsistency of the projections_. Compute histogram difference (Chi-squared histogram distance [^Malik01]) between the feature vector of the superpixel in the reference image and feature vector each its projection in the _k_-th view.
        6. _Depth candidates_.
    - The similarity measure based on comparison of histograms from superpixel areas, suggested in step 5, is __important detail__ of the method.
    - The photometric normalization in Step 3 is essential for handling scene illumination and camera exposure changes, which are often present in wide-baseline settings.
3. Energy Terms
    - _Photomconsistency term_. This term penalizes appearance inconsistencies of the superpixels across all views and directly corresponds to photoconsistency cost $$C(d)$$ for particular depth hypothesis.
    - _Boundary term_. This term expresses consistency of a superpixel boundary with a particular plane normal. This is measured via a deviation of gradient orientation of the pixels along the boundary of the superpixel to two vanishing points corresponding to that plane.
    - _Plane normal term_.  It locally smooths the 3D model by penalizing changes of a normal between neighboring superpixels, enforcing thus larger areas with the same normals.
    - _Depth term_. This term penalizes the changes in 3D depth of points of the common boundary of two neighboring superpixels.
4. MAP Solution of MRF
    - The overall 3D reconstruction can be viewed as a discrete _labeling_ problem where labels $$l_s$$ correspond to columns in the matrices $$T^s$$. In particular, given $$L$$ labels, we seek a set of planes $$\mathcal{P}$$, which minizes the nergy.
    -  Very efficient and fast algorithms for solving this type of labeling problem through linear programming relaxation and its Lagrangian dual have been reviewed in [^Kolmogorov06], [^Werner07].

5. Second Iteration Utilizing Plane Priors
    - We ultilize prior plane probabilities $$p \left ( {\prod}_s \right )$$ set as normalized histograms computed from estimated depths of all pxiels for each normal separately.
    - The second iteration stage contributes to smoother surface, favor the dominant planes, and help to solve for inconsistencies in the depth estimates remaining after the first step.

### Multi-view Superpixel Stereo in Man-made Environments [^Micusik08]

#### Abstract

Man-made environments possess many regularities which can be efficiently exploited for 3D dense reconstruction from multiple widely separated views. We present an approach utilizing properties of piecewise planarity and restricted number of plane orientations to suppress the ambiguities causing failures of standard dense stereo methods. We formulate the problem of the 3D reconstruction in _Markov Random Field_ (MRF) framework built on an image presegmented into superpixels. Using this representation, we propose novel robust cost measures, which overcome many difficulties of standard pixel-based formulations and handles favorably problematic scenarios containing many repetitive structures and no or low textured regions. We demonstrate our approach on several low textured, wide-baseline scenes demonstrating superior performance compared to previously proposed methods.


### Detailed real-time urban 3d reconstruction from video [^Pollefeys08]

#### Abstract


### Literature

[^Musialski13]: Musialski, Przemyslaw, et al. "A survey of urban reconstruction." Computer graphics forum. Vol. 32. No. 6. 2013.

[^Fathi15]: Fathi, Habib, Fei Dai, and Manolis Lourakis. "Automated as-built 3D reconstruction of civil infrastructure using computer vision: Achievements, opportunities, and challenges." Advanced Engineering Informatics 29.2 (2015): 149-161.

[^Pollefeys08]: Pollefeys, Marc, et al. "Detailed real-time urban 3d reconstruction from video." International Journal of Computer Vision 78.2-3 (2008): 143-167.

[^Micusik10]: Micusik, Branislav, and Jana Košecká. "Multi-view superpixel stereo in urban environments." International journal of computer vision 89.1 (2010): 106-119.

[^Micusik08]: B. Micusık and J. Kosecka. Multi-view superpixel stereo in man-made environments. Technical Report GMU-CS-TR-2008-1, George Mason University, USA, 2008.

[^Malik01]: Malik, J., Belongie, S., Leung, T. K., & Shi, J. (2001). Contour and texture analysis for image segmentation. International Journal of Computer Vision, 43(1), 7–27.

[^Kolmogorov06]: V. Kolmogorov, “Convergent tree-reweighted message passing for energy minimization,” T-PAMI, vol. 28, no. 10, pp. 1568–1583, 2006.

[^Werner07]: Werner, T. (2007). A linear programming approach to Max-sum problem: A review. IEEE Transactions on Pattern Analysis and Machine Intelligence, 29(7), 1165–1179.