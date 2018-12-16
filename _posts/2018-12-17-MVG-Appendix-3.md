---
layout: post
title: "Appendix 3: Parameter Estimation"
date:       2018-12-17
author:     Tong
catalog: true
tags:
    - MVG
---

> All contents come from <<Multiple View Geometry in Computer Vision>>.

###

The general lesson to be learnt from this discussion is that many of these concepts depend strongly on the particular parametrization of the model. In problems such as 3D projective reconstruction, where there is no preferred parametrization, these concepts are not well defined, or depend very strongly on assumed noise models.

### A3.1 Bias

1. The value of bias is strongly dependent on the noise distribution.

### A3.2 Variance of an estimator

1. If an algorithm performs badly when noise is added, this means that either the bias or variance of the algorithm is high. This is the case, for instance with the DLT algorithm 4.1, or the unnormalized 8-point algorithm in 11.1. The variance of the algorithm grows quickly with added noise.

2. __Result A3.1. Cramér–Rao lower bound.__

### A3.3 The posterior distribution

1. If the parameter space does not have a natural affine coordinate system (for instance if the parameter space is projective), then the MAximum A Posteriori (MAP) etimate does not really make a lot of sense.

### A3.4 Estimation of corrected measurements

1. In a general setting, the ML estimator of the corrected measurements is not only unbiased but attains the Cramér–Rao lower bound, when the noise model is Gaussian.

2. Provided the measurements surface is well approximated by its tangent plane, the ML estimate of the corrected measurement vector is unbiased, provided the noise is zero-mean. In addtion, if the noise is Gaussian and isotropic, then the ML estimate meets the Cramér–Rao lower bound.

3. The Fisher Information Matrix does not depend on the shape of the measurement surface, but only on its first-order approximation, the tangent plane.

4. If the Cramér–Rao lower bound is met, then the noise distribution must be Gaussian. In other words, if the noise distribution is not Gaussian, then we cannot meet the lower bound.

### A3.5 Notes and exercises
