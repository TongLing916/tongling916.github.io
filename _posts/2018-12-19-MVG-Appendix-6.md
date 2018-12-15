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

3. 

### A6.2 Levenberg-Marquardt iteration

### A6.3 A sparse Levenberg-Marquardt algorithm

#### A6.3.1 Partitioning the parameters in the LM method

#### A6.3.2 Covariance

#### A6.3.3 General sparse LM method

### A6.4 Application of sparse LM to 2D homography estimation

#### A6.4.1 Computation of the covariance

### A6.5 Application of sparse LM to fundamental matrix estimation

### A6.6 Application of sparse LM to multiple image bundle adjustment

### A6.7 Sparse methods for equations solving

#### A6.7.1 Banded structure in bundle-adjustment

#### A6.7.2 Solution of symmetric linear equations

#### A6.7.3 Solution of sparse symmetric linear systems

### A6.8 Robust cost functions

#### A6.8.1 Properties of the different cost functions

#### A6.8.2 Performance of the different cost functions

### A6.9 Parametrization

#### A6.9.1 Parametrization of 3D rotations

#### A6.9.2 Parametrization of homogeneous vectors

#### A6.9.3 Parametrization of the n-sphere

### A6.10 Notes and exercises
