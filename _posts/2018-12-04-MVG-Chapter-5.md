---
layout: post
title: "Chapter 5: Algorithm Evaluation and Error Analysis"
date:       2018-12-04
author:     Tong
catalog: true
tags:
    - MVG
---

> All contents come from <<Multiple View Geometry in Computer Vision>>.

### Abstract

This chapter describes methods for assessing and quatifying the results of estimation algorithms. Often it is not sufficient to simply have an estimate of a variable or transformation. Instead some measure of confidence or uncertainty is also requried.

Two methods for computing this uncertainty (covariance) are outlined here. The first is based on linear approximations and involves concatenating various Jacobian expressions. The sencond is the easier to implement Monte Carlo method.

### 5.1 Bounds on performance

1. Testing the performance of an estimation can be done by testing it on real or on synthetic data. This section will use synthetic data.

2. Testing will begin with the synthetic generation of a set of image correspondences beetween two images. Corresponding points will be chosen in such a way that they correspond via a given fixed projective transformation, and the correspondence is exact up to machine accuracy.

3. Next, artificial Gaussian noise will be added to the image measurements by perturbing both the $$x$$- and $$y$$-coordinates of the point by a zero-mean Gaussian random variable with known variance. The estimation algorithm is then run to compute the estiamted quantity.

4. The algorithm is then evaluated according to how closely the computed model matches the (noisy) input data, how closely the estiamted model agrees with the original noise-free data. This procedure should be carried out many times with different noise (i.e. a different seed for the random number generator, though each time with the same noise variance) in order to obtain a statistically meaningful performance evaluation.

5. __Convention__: 1) $$x$$: measured, 2) $$\hat{x}$$: estimated, 3) $$\bar{x}$$: true value.

#### 5.1.1 Error in one image

1. For simplicity, we consider the case where noise is added to the coordinates of the second image only.

#### 5.1.2 Error in two images



#### 5.1.3 Optimal estimators (MLE)



#### 5.1.4 Determining the correct convergence of an algorithms


### 5.2 Covariance of the estimated transformation

1.

#### 5.2.1 Forward propagation of Covariance

1.

#### 5.2.2 Backward propagation of covariance

1.

#### 5.2.3 Over-parametrization

1.

#### 5.2.4 Application and examples

#### 5.2.5 Error in both images

1.

#### 5.2.6 Using the covariance matrix in point transfer

1.

### 5.3 Monte Carlo estimation of covariance

1.

### 5.4 Closure
