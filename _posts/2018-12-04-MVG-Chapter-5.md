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

#### 5.1.3 Optimal estimators (MLE - Maximum Likelihood Estimate)

1. __Result 5.1.__ The projection of an isotropic Gaussian distribution defined on $$\mathbb{R}^N$$ with total variance $$N\sigma ^2$$ onto a subspace of dimension $$s$$ is an isotropic Gaussiance distribution with total variance $$s \sigma ^2$$.

2. __Result 5.2.__ Consider an estiamtion problem where $$N$$ measurements are to be modelled by a function depending on a set of $$d$$ essential parameters. Suppose the measurements are subject to independent Gaussian noise with standard deviation $$\sigma$$ in each measurement variable.<br>
(i) The RMS __residual error__ (distance of the measured from the estimated value) for the ML estimator is $$\epsilon _{res} = E\left [ \left \| \hat{X} - X \right \|^2 /N \right ]^{1/2} = \sigma (1 - d/N)^{1/2} \quad \quad (5.3)$$ <br>
(ii) The RMS __estimation error__ (distance of the estimated from the true value) for the ML estimator is $$\epsilon _{res} = E\left [ \left \| \hat{X} - \bar{X} \right \|^2 /N \right ]^{1/2} = \sigma (d/N)^{1/2} \quad \quad (5.4)$$

3. __2D homography - error in one image.__ d = 8 and N = 2n.

4. __Error in both images.__ d = 2n + 8 and N = 4n.

5. __Mahalanobis distance.__ The formulae quoted above were derived under the assumption that the error distribution in measurement space was an isotropic Gaussian distribution. We may assume any Gaussian distribution of error, with covariance matrix $$\sum $$. The formulae of result 5.2 remain true with $$\epsilon$$ being replaced with the expected Mahalanobis distance $$E\left [ \left \| \hat{X} - X \right \|_{\sum}^2 /N \right ]^{1/2}$$.

#### 5.1.4 Determining the correct convergence of an algorithms

1. In most estimation problems, this assumption of planarity will be very close to correct at the scale set by typical noise magnitude. In this case, the Pythagorean equality may be written as $$\left \|  X - \bar{X} \right \|^2 = \left \| X - \hat{X} \right \|^2 + \left \| \bar{X} - \hat{X} \right \|^2 \quad \quad (5.7)$$

2. In evaluating an algorithm with synthetic data, this equality allows a simple test to see whether the algorithm has converged to the optimal value. If the estimated value $$\hat{X}$$ satisfies this equality, then it is a strong indication that the algorithm has found the true global minimum. Note that it is unnecessary in applying this test to determine the number of degrees of freedom of the problem. A few more properties are listed:

- This test can be used to determine on a run-by-run basis whether the algorithm has succeeded.
- This test can only be used for synthetic data, or at least data for which the true measurements $$\bar{X}$$ are known.
- The equality depends on the assumption that the surface $$S_M$$ consisting of valid measurements is locally planar.
- The test is a test for the algorithm finding the global, not a local solution.  

### 5.2 Covariance of the estimated transformation

1. The chief concern is how accurately the transformation itself has been computed. The uncertainty of the computed transformation is conveniently captured in the _covariance matrix_ of the transformation. Since $$H$$ is a matrix with 9 entries, its covariance matrix will be a $$9 \times 9$$ matrix.

2. In this section, it will be seen how this covariance matrix may be computed.

#### 5.2.1 Forward propagation of Covariance

1. __Result 5.3.__ Let $$v$$ be a random vector in $$\mathbb{R}^M$$ with mean $$\hat{v}$$ and covariance matrix $$\sum$$, and suppose that $$f: \mathbb{R}^M \rightarrow \mathbb{R}^N$$ is an affine mapping defined by $$f(v) = f(\bar{v}) + A(v - \bar{v})$$. Then $$f(v)$$ is a random variable with mean $$f(\bar{v})$$ and covariance matrix $$A\sum A^T$$.

2. __Result 5.6.__ Let $$v$$ be a random vector in $$\mathbb{R}^M$$ with mean $$\hat{v}$$ and covariance matrix $$\sum$$, and let $$f: \mathbb{R}^M \rightarrow \mathbb{R}^N$$ be differentiable in a neighbourhood to $$\bar{v}$$. Then up to a first-order approximations, $$f(v)$$ is a random variable with mean $$f(\bar{v})$$ and covariance matrix $$J\sum J^T$$, where $$J$$ is the Jacobian matrix of $$f$$, evaluated at $$\bar{v}$$.

#### 5.2.2 Backward propagation of covariance

#### 5.2.3 Over-parametrization

#### 5.2.4 Application and examples

1. __Error in one image.__ The problem is how to find the covariance of an estimated 2D homography $$H$$. The $$3 \times 3$$ matrix $$H$$ is represented by a 9-dimensional parameter vector which will be denoted by $$h$$. A procedure for computing the covariance matrix of the estimated transformation is as follows.

(i) Estimate the transformation $$\hat{H}$$ from the given data.
(ii) Compute the Jacobian matrix $$J_f = \frac{\partial X^\prime}{\partial h}$$, evaluated at $$\hat{h}$$.
(iii) The covariance matrix of the estimated $$h$$ is given by: $${\sum}_h = (J_f^T {\sum}_{X^\prime}^{-1} J_f)^+$$.

#### 5.2.5 Error in both images

#### 5.2.6 Using the covariance matrix in point transfer

1. Once one has the covariance, one may compute the uncertainty associated with a given point transfer.

2. Example: consider a new point $$x$$ in the first image, not used in the computation of the transformation, $$H$$. The Corresponding point in the second image $$x^\prime = Hx$$. However, because of the uncertainty in the estimation $$H$$, the correct location of the point $$x^\prime$$ will also have associated uncertainty. One may compute this uncertainty from the covariance matrix of $$H$$. If in addtion, the point $$x$$ itself is measured with some uncertainty, then one should compute this uncertainty from the $$H$$ and the $$x$$.

### 5.3 Monte Carlo estimation of covariance

1. The method of estimating covariance has relied on an assumption of linearity. It has also been assumed that the method of estimation of the transformation $$H$$ was the Maximum Likelihood Estimate.

2. A general method of getting an estimate of the covariance is by exhaustive simulation. Assming that the noise is drawn from a give noise distribution, one starts with a set of point matches corresponding perfectly to a given transformation. One then addes noise to the points and computes the corresponding transformation using the chosen estimation procedure. The covariance of the transformation $$H$$ or a further transferred point is then computed statistically from multiple trials with noise drawn from the assumed noise distribution.

### 5.4 Closure

1. The derivations throughout this chapter have been considerably simplified by only using first-order Taylor expansions, and assuming Gaussian error distributions.
