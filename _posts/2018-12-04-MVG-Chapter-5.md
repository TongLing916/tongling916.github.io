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

2. The RMS (root-mean-squared) residual error $$\epsilon _{res} = (\frac{1}{2n} \sum_{i=1}^{n} d(x_i^\prime, \hat{x_i}^\prime)^2)^{1/2} \quad \quad (5.1)$$ measures the average difference beetween the noisy input data ($$x_i^\prime$$) and the estimated points $$\hat{x_i}^\prime = \hat{H} \bar{x_i}$$.

3. The value of the residual error is _not_ in itself an absolute measure of the quality of the solution obtained.

4. Asymptotically, the variance should decrease in inverse proportion to the number of point matches. At the same time, the residual error will increase.  

#### 5.1.2 Error in two images

1. In the case of error in both images, the ressidual error is $$\epsilon _{res} = \frac{1}{\sqrt{4n}}( \sum_{i=1}^{n} d(x_i, \hat{x_i})^2 + \sum_{i=1}^{n} d(x_i^\prime, \hat{x_i}^\prime)^2)^{1/2} \quad \quad (5.2)$$.

#### 5.1.3 Optimal estimators (MLE - Maximum Likelihood Estimor is

1. __Result 5.1.__ The projection of an isotropic Gaussian distribution defined on $$\mathbb{R}^N$$ with total variance $$N\sigma ^2$$ onto a subspace of dimension $$s$$ is an isotropic Gaussiance distribution with total variance $$s \sigma ^2$$.

2. __Result 5.2.__ Consider an estiamtion problem where $$N$$ measurements are to be modelled by a function depending on a set of $$d$$ essential parameters. Suppose the measurements are subject to independent Gaussian noise with standard deviation $$\sigma$$ in each measurement variable.<br>
(i) The RMS __residual error__ (distance of the measured from the estimated value) for the ML estimator is $$\epsilon _{res} = E\left [ \left \| \hat{X} - X \right \|^2 /N \right ]^{1/2} = \sigma (1 - d/N)^{1/2} \quad \quad (5.3)$$
(ii) The RMS __estimation error__ (distance of the estimated from the true value) for the ML estimator is $$\epsilon _{res} = E\left [ \left \| \hat{X} - \bar{X} \right \|^2 /N \right ]^{1/2} = \sigma (d/N)^{1/2} \quad \quad (5.3)$$

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
