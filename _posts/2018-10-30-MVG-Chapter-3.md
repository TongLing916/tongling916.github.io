---
layout: post
title: "MVG Chapter 3-Projective Geometry and Transformations of 3D"
date:       2018-10-30
author:     Tong
catalog: true
tags:
    - MVG
---

1. In $$\mathbb{P^3}$$ Euclidean 3-space is augmented with a set of ideal points which are on a _plane_ at infinity, $$\pi_\infty$$. Parallel lines, and now parallel _planes_, intersect on $$\pi_\infty$$.

## 3.1 Points and projective transformations

## 3.2 Representing and transforming planes, lines and quadrics

### 3.2.1 Planes

1. A plane in 3-space may be written as <br>
$$
\pi_1 X + \pi_2 Y + \pi_3 Z + \pi_4 = 0.       \quad  (3.2)
$$
<br> A plane has 3 degrees of freedom in 3-space. The homogeneous representation of the plane if the 4-vector $$\pi = (\pi_1,\pi_2,\pi_3,\pi_4)^\intercal$$.

2. The point $$X$$ is on the plane $$\pi$$ can be expressed as <br>
$$
\pi^\intercal X = 0  \quad (3.2)
$$
<br> The first 3 components of $$\pi$$ correspond to the plane normal of Euclidean geometry - using inhomogeneous notation (3.2) becomes <br>
$$
n.\tilde{X} + d = 0
$$
<br> where $$n = (\pi_1,\pi_2,\pi_3)^\intercal, \tilde{X} = (X,Y,Z)^\intercal, x_4 = 1, d = \pi_4$$. In this form $$d/\left \| \pi \right \|$$ is the distance of the plane from the origin.

3. __Three points define a plane.__ <br>
$$
\begin{bmatrix}X_1^\intercal\\X_2^\intercal\\ X_3^\intercal\end{bmatrix} \pi = 0  \quad (3.3)
$$

4. We start from the matrix $$M = \begin{bmatrix} X & X_1 & X_2 & X_3 \end{bmatrix}$$ which is composed of a general point $$X$$ and the three points $$X_i$$ which define the plane $$\pi$$. The determinant $$\det M = 0$$ when $$X$$ lies on $$\pi$$. We can obtain <br>
$$
\det M = x_1 D_{234} - x_2 D_{134} + x_3 D_{124} -x_4 D_{123}
$$
<br> where $$D_{jkl}$$ is the determinant formed from the $$jkl$$ rows of the $$ 4 \times 3 $$ matrix $$\begin{bmatrix} X_1 & X_2 & X_3 \end{bmatrix}$$, So we can read off the plane coefficient as <br>
$$
\pi = (D_{234},-D_{134},D_{124},-D_{123})^\intercal     \quad (3.4)
$$

5. __Three planes define a point.__ <br>
$$
\begin{bmatrix}\pi_1^\intercal\\\pi_2^\intercal\\\pi_3^\intercal\end{bmatrix} X = 0  \quad (3.5)
$$

6. __Projective transformation.__ Under the point transformation $$X^\prime = HX$$, a plane transforms as <br>
$$
\pi^\prime = H^{-\intercal} \pi   \quad (3.6)
$$

7. __Parametrized points on a plane.__ The points $$X$$ on the plane $$\pi$$ may be written as <br>
$$
X = Mx
$$
<br> where the columns of the $$4 \times 3$$ matrix $$M$$ generate the rank 3 null-space of $$\pi^\intercal$$, i.e. $$\pi^\intercal M = 0$$, and the 3-vector $$x$$ (which is a point on the projective plane $$\mathbb{P^2}$$) parametrizes point on the plane $$\pi$$.


### 3.2.2 Lines

1. 

### 3.2.3 Quadrics and dual quadrics

## 3.3 Twisted cubics

## 3.4 The hierarchy of transformations

### 3.4.1 The screw decomposition

## 3.5 The plane at infinity

## 3.6 The absolute conic

## 3.7 The absolute dual quadric

## 3.8 Closure




[youtube-tensors]: https://www.youtube.com/playlist?list=PLJHszsWbB6hrkmmq57lX8BV-o-YIOFsiG





























