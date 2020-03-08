---
layout:     post
title:      "Dense SLAM"
date:       2020-2-22
author:     Tong
catalog: true
tags:
    - SLAM
---

### DTAM[^Newcombe2011a]

#### Abstract

DTAM is a system for real-time camera tracking and reconstruction which relies not on feature extraction but dense, every pixel methods. As a single hand-held RGB camera flies over a static scene, we estimate detailed textured depth maps at selected keyframes to produce a surface patchwork with millions of vertices. We use the hundreds of images available in a video stream to improve the quality of a simple photometric data term, and minimise a global spatially regularised energy functional in a novel non-convex optimisation framework. Interleaved, we track the camera’s
6DOF motion precisely by frame-rate whole image alignment against the entire dense model. Our algorithms are highly parallelisable throughout and DTAM achieves realtime
performance using current commodity GPU hardware. We demonstrate that a dense model permits superior tracking performance under rapid motion compared to a state of
the art method using features; and also show the additional usefulness of the dense model for real-time scene interaction in a physics-enhanced augmented reality application.

#### Method

The overall structure of our algorithm is straightforward. Given a dense model of the scene, we use dense whole image alignment against that model to track camera motion at frame-rate. And tightly interleaved with this, given images from tracked camera poses, we update and expand the model by building and refining dense textured depth maps. Once bootstrapped, the system is fully self-supporting and no feature-based skeleton or tracking is required.

#### Dense Mapping

- Define a projective photometric cost volume $$\mathrm{C}_{r}$$. A row $$\mathrm{C}_{r}(\mathbf{u})$$ in the cost volume called a disparity space image in stereo matching [^Szeliski2004], and generalized more recently in [^Rhemann2011] for any discrete per pixel labelling) stores the accumulated photometric error as a function of inverse depth $$d$$.

$$
\mathbf{C}_{r}(\mathbf{u}, d)=\frac{1}{|\mathcal{I}(r)|} \sum_{m \in \mathcal{I}(r)}\left\|\rho_{r}\left(\mathbf{I}_{m}, \mathbf{u}, d\right)\right\|_{1},
$$

where $$\mathcal{I}(\tau)$$ is the sete of frames nearby and overlapping $$r$$, the photometric error for each overlapping image is
$$
\rho_{r}\left(\mathbf{I}_{m}, \mathbf{u}, d\right)=\mathbf{I}_{r}(\mathbf{u})-\mathbf{I}_{m}\left(\pi\left(\mathrm{KT}_{m r} \pi^{-1}(\mathbf{u}, d)\right)\right)
$$.

- Rather than using a patch-based normalised score, or pre-processing the input data to increase illumination invariance over wide baselines, we take the opposite approach and show the advantage of reconstruction from a large number of video frames taken from __very close viewpoints__ where very high quality matching is possible.

##### Regularised Cost

-  We seek an inverse depth map which minimises an energy functional comprising the photometric error cost as a _data term_ and a _regularisation term_ that penalises deviation from a spatially smooth inverse depth map solution.

$$
E_{\xi}=\int_{\Omega}\left\{g(\mathbf{u})\|\nabla \xi(\mathbf{u})\|_{\epsilon}+\lambda \mathbf{C}(\mathbf{u}, \xi(\mathbf{u}))\right\} \mathrm{d} \mathbf{u},
$$

$$
g(\mathbf{u})=e^{-\alpha\left\|\nabla \mathbf{I}_{r}(\mathbf{u})\right\|_{2}^{\beta}}
$$

where $$\nabla \xi(\mathbf{u})$$ is the gradient of the inverse depth map, $$g(\mathbf{u})$$ is the per pixel weight.

- Following the large displacement optic flow method of [^Steinbrucker2009], we approximate the energy functional by coupling the data and regularisation terms through an auxiliary variable $$\alpha: \Omega \rightarrow \mathbb{R}$$,

$$
\mathbf{E}_{\boldsymbol{\xi}, \boldsymbol{\alpha}}=\int_{\Omega}\{ g(\mathbf{u})\|\boldsymbol{\nabla} \boldsymbol{\xi}(\mathbf{u})\|_{\boldsymbol{\epsilon}}+\frac{1}{2 \theta}(\boldsymbol{\xi}(\mathbf{u})-\boldsymbol{\alpha}(\mathbf{u}))^{2} +\lambda \mathbf{C}(\mathbf{u}, \boldsymbol{\alpha}(\mathbf{u}))\} \mathrm{d} \mathbf{u}
$$

- Importantly, the discrete cost volume $$C$$, can be computed by keeping the average cost up to date as each overlapping frame from $$\mathbf{I}_{m \in \mathcal{I}(r)}$$ arrives removing the need to store images or poses and enabling constant time optimisation for any number of overlapping images.

### KinectFusion[^Newcombe2011b]

### Literature

[^Szeliski2004]: R. Szeliski and D. Scharstein. Sampling the disparity space image. IEEE Transactions on Pattern Analysis and Machine Intelligence (PAMI), 26:419–425, 2004.

[^Rhemann2011]: C. Rhemann, A. Hosni, M. Bleyer, C. Rother, and M. Gelautz. Fast cost-volume filtering for visual correspondence and beyond. In Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition (CVPR), 2011.

[^Steinbrucker2009]: F. Steinbrucker, T. Pock, and D. Cremers. Large displacement optical flow computation without warping. In Proceedings of the International Conference on Computer Vision (ICCV), 2009.

[^Newcombe2011a]: Newcombe, Richard A., Steven J. Lovegrove, and Andrew J. Davison. "DTAM: Dense tracking and mapping in real-time." 2011 international conference on computer vision. IEEE, 2011.

[^Newcombe2011b]: Newcombe, Richard A., et al. "KinectFusion: Real-time dense surface mapping and tracking." 2011 10th IEEE International Symposium on Mixed and Augmented Reality. IEEE, 2011.
