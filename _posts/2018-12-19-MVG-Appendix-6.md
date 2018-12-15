---
layout: post
title: "Appendix 6: Iterative Estimation Methods"
date:       2018-12-19
author:     Tong
catalog: true
tags:
    - MVG
---

> All contents come from <<Multiple View Geometry in Computer Vision>>.

### Abstract

The general idea of Newton iteration is familiar to most students of numerical methods as a way of finding the zeros of a function of a single variable. Its generalization to several variables and application to finding least-squares solutions rather then exact solutions is relatively straightforward.

The Levenberg-Marquardt method is a simple variation on Newton iteration designed to provide faster convergence and regularization in the case of over-parametrized problems. It may be seen as a hybrid between Newton iteration and a gradient descent method.

Important reductions of computation complexity are obtained by dividing the set of parameters into two parts. The two parts generally consist of a set of parameters representing camera matrices or homographies, and a set of parameters representing points. This leads to a sparse structure to the problem.

### A6.1 Newton iteration

1. Suppose we are given a hypothesized functional relation $$X=f(P)$$ where $$X$$ is a _measurement vector_ and $$P$$ is a _parameter vector_ in Euclidean spaces $$\mathbb{R}^N$$ and $$\mathbb{R}^M$$ respectively.

2. To summarize, we have so far considered three methods of minimization of a cost function $$g(P)=\frac{1}{2}\left \| \epsilon (P) \right \|^2$$ ($$\frac{1}{2}$$ is present for simplifying the succeeding computations):
  - __Newton.__ Update equation: $$g_{PP} \Delta =-g_p$$, where $$g_{PP} = \epsilon _ P ^T \epsilon _ P  + \epsilon _ {PP}^T \epsilon$$ and $$g_p =\epsilon _ P ^T \epsilon$$. Newton iteration is based on the assumption of an approximately quadratic cost function near the minimum, and will show rapid convergence if this condition is met. The disadvantage of this approach is that the computation of the Hessian may be difficult. In addition, far from the minimum the assumption of quadratic behaviour is probably invalid, so a lot of extra work is done with little benefit.
  - __Gauss-Newton.__ Update equation: $$\epsilon _ P ^T \epsilon _ P \Delta = - \epsilon _ {P}^T \epsilon$$. This is equivalent to Newton iteration in which the Hessian is approximated by $$\epsilon _ P ^T \epsilon _ P$$. Generally, this is a good approximation, particularly close to a minimum, or when $$\epsilon$$ is nearly linear in $$P$$.
  - __Gradient descent.__ Update equation: $$\lambda \Delta = - \epsilon _ {P}^T \epsilon = -g_ P$$. The Hessian in Newton iteration is replaced by a multiple of the identity matrix. Each update is in the direction of most rapid local decrease of the function value. The value of $$\lambda$$ may be chosen adaptively, or by a line search in the downward gradient direction. Generally, gradient descent by itself is not recommended, but in conjunction with Gauss-Newton, it yields the commonly used Levenberg-Marquardt method.

### A6.2 Levenberg-Marquardt iteration

1. The Levenberg-Marquardt iteration method is a slight variation on the Gauss-Newton iteration method. The normal equations $$J^TJ\Delta = -J^T \epsilon $$ are replaced by the _augmented normal equations_ $$(J^TJ + \lambda I)\Delta = -J^T \epsilon $$, for some value of $$\lambda$$ that varies from iteration to iteration. A typical initial value of $$\lambda$$ is $$10^{-3}$$ times the average of the diagonal elements of $$N=J^TJ$$.

2. The LM algorithm moves seamlessly between Gauss-Newton iteration, which will cause rapid convergence in the neighbourhood of the solution, and a gradient descent approach, which will guarantee a decrease in the cost function when the going is difficult.

### A6.3 A sparse Levenberg-Marquardt algorithm

#### A6.3.1 Partitioning the parameters in the LM method

1. __Algorithm A6.1.__ A partitioned Levenberg-Marquardt algorithm.
![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/post-algorithm-A6.1.JPG)

#### A6.3.2 Covariance

1. __Algorithm A6.2.__ Computation of the covariance matrix of the LM parameters.
![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/post-algorithm-A6.2.JPG)

#### A6.3.3 General sparse LM method

1. __Algorithm A6.3.__ A sparse Levenberg-Marquardt algorithm.
![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/post-algorithm-A6.3.JPG)

2. The important observation is that in this form, each step of the algorithm requires computation time linear in $$n$$. Without the advantage resulting from the sparse structure, the algorithm would have complexity of order $$n^3$$.

### A6.4 Application of sparse LM to 2D homography estimation

#### A6.4.1 Computation of the covariance

### A6.5 Application of sparse LM to fundamental matrix estimation

### A6.6 Application of sparse LM to multiple image bundle adjustment

1. One may apply the LM algorithm to the simultaneous estimation of multiple camera and points to compute projective structure, or perhaps affine or metric structure given appropriate constraints. This technique is called _bundle adjustment__.

2. __Algorithm 6.4.__ General sparse Levenberg-Marquardt algorithm.
![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/post-algorithm-A6.4.JPG)

### A6.7 Sparse methods for equations solving

1. For bundle-adjustment problems with banded track structure, sparseness can appear at two levels, first at the level of independence of the individual point measurements, and secondly arising from the banded track structure.  

#### A6.7.1 Banded structure in bundle-adjustment

1. The block $$S_{jk}$$ is non-zero only if there exists a point that is visible in both the $$j$$-th and $$k$$-th images.

#### A6.7.2 Solution of symmetric linear equations

1. __Result A6.1__ Any positive-definite symmetric matrix $$A$$ can be factored as $$A=LDL^T$$, in which $$L$$ is a lower-triangular matrix with unit diagonal entries, and $$D$$ is diagonal.

#### A6.7.3 Solution of sparse symmetric linear systems

### A6.8 Robust cost functions

1. In estimation problems of the Newton or Levenberg-Marquardt type, an important decision to make is the precise form of the cost function.

2. An assumption of Gaussian noise without outliers implies that the Maximum Likelihood estimate is given by a least-squares cost function involving the predicted errors in the measurements, where the noise is introduced.

3. __Statistically based cost functions.__ 1) Squared error. 2) Blake-Zisserman. 3) Corrupted Gaussian.

4. __Heuristic cost function.__ 1) Cauchy cost function. 2) The $$L1$$ cost function. 3) Huber cost function. 4) Pseudo-Huber cost function.

#### A6.8.1 Properties of the different cost functions

1. __Squared error.__

2. __Non-convex cost functions.__

3. __Asymptotically linear cost functions.__

#### A6.8.2 Performance of the different cost functions

1. The squared-error cost function is generally very susceptible to outliers, and may be regarded as unusable as long as outliers are present. It outliers have been thoroughly eradicated, using for instance RANSAC, then it may be used.

2. The non-convex cost functions, though generally having a stable minimum, not much effected by outliers have the significant disadvantage of having local minima, which can make convergence to a global minimum chancy. The estimate is not strongly attracted to the minimum from outside of its immediate neighbourhood. Thus, they are not useful, unless (or until) the estimate is close to the final correct value.

3. The Huber cost function has the pleasant property of being convex, which makes convergence to a global minimum more reliable. The minimum is quite immune to the baleful influence of outliers since it represents a compromise between the Maximum Likelihood estimate of the inliers and the median of the outliers. The pseudo-Huber cost function is a good alternative to Huber, but use of $$L1$$ should be approached with case, because of its non-differentiablity at the origin.

4. Choose a parametrization in which the error is as close as possible to being a linear function of the parameters, at least locally.

### A6.9 Parametrization

1. __Gauge freedom.__
  - The word gauge means a coordinate system for a parameter set, and gauge-freedom essentially refers to a change in the representation of the parameter set that does not essentially change the underlying geometry, and hence has no effect on the cost function.
  - The most important gauge freedoms commonly encountered are projective or other ambiguities, such as those arising in reconstruction problems.
  - The scale ambiguity of homogeneous vectors can be counted as gauge freedom also.
  - Gauge freedoms in the parametrization of an optimization problem cause the normal equations to be singular, and hence allow multiple solutions. This problem is avoided by the regularization (or enhancement) step in Levenberg-Marquardt, but there is evidence that excessive gauge freedoms, when gauge freedoms are present the covariance matrix of the estimated parameters is troublesome.
  - When gauge freedoms are present, the covariance matrix of the estimated parameters is troublesome, in that there will be infinite variance in unconstrained parameter directions. For instance, it makes no sense to talk of the covariance matrix of an estimated homogeneous vector, unless the scale of the vector is constrained.

2. __What makes a good parametrization?__ The foremost requirement of a good parametrization is that it be singularity-free, at least in the areas that are visited during the course of an iterative optimization. This means that the parametrization should be locally continuous, differentiable and one-to-one - in short a diffeomorphism.

#### A6.9.1 Parametrization of 3D rotations

#### A6.9.2 Parametrization of homogeneous vectors

#### A6.9.3 Parametrization of the n-sphere

### A6.10 Notes and exercises
