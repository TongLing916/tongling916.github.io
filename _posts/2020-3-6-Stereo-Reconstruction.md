---
layout:     post
title:      "Stereo Reconstruction"
date:       2020-3-6
author:     Tong
catalog: true
tags:
    - SLAM
---

### Stereo Processing by Semiglobal Matching and Mutual Information[^Hirschmuller2007]

#### Abstract

This paper describes the Semiglobal Matching (SGM) stereo method. It uses a pixelwise, Mutual Information (MI)-based matching cost for compensating radiometric differences of input images. Pixelwise matching is supported by a smoothness constraint that is usually expressed as a global cost function. SGM performs a fast approximation by pathwise optimizations from all directions. The discussion also addresses occlusion detection, subpixel refinement, and multibaseline matching. Additionally, postprocessing steps for removing outliers, recovering from specific problems of structured environments, and the interpolation of gaps are presented. Finally, strategies for processing almost arbitrarily large images and fusion of disparity images using orthographic projection are proposed. A comparison on standard stereo images shows that SGM is among the currently top-ranked algorithms and is best, if subpixel accuracy is considered. The complexity is linear to the number of pixels and disparity range, which results in a runtime of just 1-2 seconds on typical test images. An in depth evaluation of the MI-based matching cost demonstrates a tolerance against a wide range of radiometric transformations. Finally, examples of reconstructions from huge aerial frame and pushbroom images demonstrate that the presented ideas are working well on practical problems.

#### 1. Introduction

This paper describes the Semiglobal Matching (SGM) method [^Hirschmuller2005], [^Hirschmuller2006], which calculates the matching cost hierarchically by __Mutual Information__. __Cost aggregation__ is performed as approximation of a global energy function by pathwise optimizations from all directions through the image. __Disparity computation__ is done by winner takes all and supported by disparity refinements like consistency checking and subpixel interpolation. __Multibaseline matching__ is handled by fusion of disparities. Further __disparity refinements__ include peak filtering, intensity consistent disparity selection, and gap interpolation. Previously unpublished is the extension for matching almost arbitrarily large images and the fusion of several disparity images using orthographic projection.

#### 2. Semiglobal Matching

The Semiglobal Matching (SGM) method is based on the idea of pixelwise matching of Mutual Information and approximating a global, 2D smoothness constraint by combining many 1D constraints.

##### 2.1 Pixelwise Matching Cost Calculation

1. Input images are assumed to have a known epipolar geometry.

2. The matching cost calculation can be based on Mutual Information (MI)[^Viola1997], which is insensitive to recording and illumination changes. It is defined from the entropy $$H$$ of two images (that is, their information content), as well as their joint entropy:

$$
M I_{I_{1}, I_{2}}=H_{I_{1}}+H_{I_{2}}-H_{I_{1}, I_{2}}
$$

3. The entropies are calculated from the probability distributions $$P$$ of intensities of the associated images:

$$
\begin{aligned} H_{I} &=-\int_{0}^{1} P_{I}(i) \log P_{I}(i) d i \\ H_{I_{1}, I_{2}} &=-\int_{0}^{1} \int_{0}^{1} P_{I_{1}, I_{2}}\left(i_{1}, i_{2}\right) \log P_{I_{1}, I_{2}}\left(i_{1}, i_{2}\right) d i_{1} d i_{2} \end{aligned}
$$

4. The MI matching cost
$$
\begin{aligned} C_{M I}(\mathbf{p}, d) &=-m i_{I_{b}, f_{D}\left(I_{m}\right)}\left(I_{b \mathbf{p}}, I_{m \mathbf{q}}\right) \\ \mathbf{q} &=e_{b m}(\mathbf{p}, d) \end{aligned}
$$

5. An implementation of the hierarchical MI computation (HMI) would collect all alleged correspondences defined by an initial disparity (that is, up-scaled from previous hierarchical level or random in the beginning). From the correspondences, the probability distribution $$P$$ is calculated according to (6). The size of $$P$$ is the square of the number of intensities, which is constant (for example, $$256 \times 256$$). The subsequent operations consist of Gaussian convolutions of P and calculating the logarithm.

$$
P_{I_{1}, I_{2}}(i, k)=\frac{1}{n} \sum_{\mathbf{p}} \mathrm{T}\left[(i, k)=\left(I_{1 \mathrm{p}}, I_{2 \mathrm{p}}\right)\right] \quad (6)
$$

##### 2.2 Cost Aggregation

1. An additional constraint is added that supports smoothness by penalizing changes of neighboring disparities. The pixelwise cost and the smoothness constraints are expressed by defining the energy $$E(D)$$ that depends on the disparity image $$D$$:
$$
E(D)= \sum_{\mathbf{p}}\left(C\left(\mathbf{p}, D_{\mathbf{p}}\right)+\sum_{\mathbf{q} \in N_{\mathbf{p}}} P_{1} \mathbf{T}\left[\left|D_{\mathbf{p}}-D_{\mathbf{q}}\right|=1\right]\right. \left.+\sum_{\mathbf{q} \in N_{\mathbf{p}}} P_{2} \mathbf{T}\left[\left|D_{\mathbf{p}}-D_{\mathbf{q}}\right|>1\right]\right)
$$

2. Discontinuities are often visible as intensity changes. This is exploited by adapting $$P2$$ to the intensity gradient.

3. Unfortunately, such a global minimization, that is, in 2D, is NP-complete for many discontinuity preserving energies[^Boykov2001]. This leads to the new idea of aggregating matching costs in 1D from all directions equally.

4. The aggregated (smoothed) cost $$S(p, d)$$ for a pixel $$p$$ and disparity $$d$$ is calculated by summing the costs of all 1D minimum cost paths that end in pixel $$p$$ at disparity $$d$$.

$$\begin{aligned}
L_{\mathbf{r}}(\mathbf{p}, d)=& C(\mathbf{p}, d)+\min \left(L_{\mathbf{r}}(\mathbf{p}-\mathbf{r}, d)\right.\\
& L_{\mathbf{r}}(\mathbf{p}-\mathbf{r}, d-1)+P_{1} \\
& L_{\mathbf{r}}(\mathbf{p}-\mathbf{r}, d+1)+P_{1} \\
&\left.\min L_{\mathbf{r}}(\mathbf{p}-\mathbf{r}, i)+P_{2}\right)-\min _{k} L_{\mathbf{r}}(\mathbf{p}-\mathbf{r}, k)
\end{aligned}$$

##### 2.3 Disparity Computation

The disparity image $$D_b$$ that corresponds to the base image $$I_b$$ is determined as in local stereo methods by selecting for each pixel p the disparity $$d$$ that corresponds to the minimum cost. For subpixel estimation, a quadratic curve is fitted through the neighboring costs, that is, at the next higher and lower disparity, and the position of the minimum is calculated. Using a quadratic curve is theoretically justified only for correlation using the sum of squared differences. However, it is used as an approximation due to the simplicity of calculation. This supports fast computation.

##### 2.4 Multibaseline Matching

Multibaseline matching is performed by pairwise matching between the base and all match images individually. The consistency check (Section 2.3) is used after pairwise matching for eliminating wrong matches at occlusions and many other mismatches. Finally, the resulting disparity images are fused by considering individual scalings.

##### 2.5 Disparity Refinement

- Removal of peaks
- Intensity consistent disparity selection
- Discontinuity reserving interpolation

##### 2.6 Processing of Huge Images

##### 2.7 Fusion of Disparity Images

First, the height data is segmented in the same way as described in Section 2.5.1 by allowing height values of neighboring grid cells within one segment to vary by a certain predefined amount. Each segment is considered to be a physical surface. Holes can exist within or between segments. The former are filled by Inverse Distance Weighted (IDW) interpolation from all valid pixels just next to the hole. The latter case is handled by only considering valid pixels of the segment whose pixel have the lowest mean compared to the valid bordering pixel of all other segments next to the hole. This strategy performs smooth interpolation but maintains height discontinuities by extrapolating the background. Using IDW instead of pathwise interpolation is computationally more expensive, but it is performed only once on the fused result and not on each disparity image individually.

### Literature

[^Hirschmuller2005]: Hirschmuller, Heiko. "Accurate and efficient stereo processing by semi-global matching and mutual information." 2005 IEEE Computer Society Conference on Computer Vision and Pattern Recognition (CVPR'05). Vol. 2. IEEE, 2005.

[^Hirschmuller2006]: Hirschmuller, Heiko. "Stereo vision in structured environments by consistent semi-global matching." 2006 IEEE Computer Society Conference on Computer Vision and Pattern Recognition (CVPR'06). Vol. 2. IEEE, 2006.

[^Hirschmuller2007]: Hirschmuller, Heiko. "Stereo processing by semiglobal matching and mutual information." IEEE Transactions on pattern analysis and machine intelligence 30.2 (2007): 328-341.

[^Viola1997]: Viola, Paul, and William M. Wells III. "Alignment by maximization of mutual information." International journal of computer vision 24.2 (1997): 137-154.

[^Boykov2001]: Boykov, Yuri, Olga Veksler, and Ramin Zabih. "Fast approximate energy minimization via graph cuts." IEEE Transactions on pattern analysis and machine intelligence 23.11 (2001): 1222-1239.
