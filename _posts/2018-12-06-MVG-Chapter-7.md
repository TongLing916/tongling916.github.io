---
layout: post
title: "Chapter 7: Computation of the Camera Matrix P"
date:       2018-12-06
author:     Tong
catalog: true
tags:
    - MVG
---

> All contents come from <<Multiple View Geometry in Computer Vision>>.

### Abstract

This chapter describes numerical methods for estimating the camera projection matrix from corresponding 3-space and image entities. This computation of the camera matrix is known as _resectioning_.

Throughout this book, it is assumed that the map from 3-space to the image is linear. This assumption is invalid if there is lens distortion.

### 7.1 Basic equations

1. For each correspondence $$X_i \leftrightarrow x_i$$, we derive a relationship
$$\begin{bmatrix}0^\intercal & -\omega_i X_i^\intercal & y_i X_i^\intercal\\ \omega_i X_i^\intercal & 0^\intercal & -x_i X_i^\intercal\\ -y_i X_i^\intercal & x_i X_i^\intercal & 0^\intercal \end{bmatrix} \begin{pmatrix}P^1\\ P^2 \\ P^3 \end{pmatrix} = 0    \quad \quad (7.1)$$

2. Alternatively, one may choose to use only the first two equations: $$\begin{bmatrix}0^\intercal & -\omega_i X_i^\intercal & y_i X_i^\intercal\\ \omega_i X_i^\intercal & 0^\intercal & -x_i X_i^\intercal \end{bmatrix} \begin{pmatrix}P^1\\ P^2 \\ P^3 \end{pmatrix} = 0    \quad \quad (7.2)$$

3. __Minimal solution.__ Since the matrix $$P$$ has 12 entries, and (ignoring scale) 11 degrees of freedom, it is  necessary to have 11 equations to solve for $$P$$. Since each point correspondence leads two equations, at a minimum 5.5 such correspondences are required to solve for $$P.$$ The 0.5 indicates that only one of the equations is used from the sixth point. The solution is obtained by solving $$Ap = 0$$ where $$A$$ is an $$11 \times 12$$ matrix in this case.

4. __Over-determined solution.__ If the data is not exact, because of noise in the point coordinates, and $$n \geq 6$$ point correspondences are given, then there will not be an exact solution to the equations $$Ap = 0$$. As in the estimation of a homography, a solution for $$P$$ may be obtained by minimizing an algebraic or geometric error. The residual $$Ap$$ is known as the _algebraic error._

5. __Degenerate configurations.__ The most important critical configurations are as follows: 1) The camera and points all lie on a twisted cubic. 2) The points all lie on the union of a plane and a single straight line containing the camera centre.

6. __Data normlaization.__

7. __Line correspondences.__ It is a simple matter to extend the DLT algorithm to take account of line correspondences as well.

### 7.2 Geometric error

1. __Algorithm 7.1.__ The Gold Standard algorithm for estimating $$P$$ from world to image point correspondences in the case that the world points are very accurately knwon. <br>
![]()

#### 7.2.1 Geometric interpretation of algebraic error

1.

#### 7.2.2 Estimation of an affine camera

1.

### 7.3 Restricted camera estimation

1.

### 7.4 Radial distortion

1.


### 7.5 Closure

1.
