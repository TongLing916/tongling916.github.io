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

0. A line is defined by the _join_ of two points or the intersection of two planes. Lines have 4 degrees of freedom in 3-space.

1. __Null-space and span representation.__

2. __Plücker matrices.__ Here a line is represented by a $$4 \times 4$$ skew-symmetric homogeneous matrix. In particular,  the line joining the two points $$A, B$$ is represented by <br>
$$
L = A B^\intercal - B A^\intercal \quad (3.8)
$$

3. A dual Plücker representation $$L^*$$ is obtained for a line formed by the intersection of two planes. <br>
$$
L^* = P Q^\intercal - Q P^\intercal \quad (3.9)
$$
<br> Under the point transformation $$X^\prime = HX$$, the matrix $$L^*$$ transforms as $$L^{*\prime} = H^\T L H^\{-1}$$. The matrix $$L^*$$ can be obtained directly from $$L$$ by a simple rewrite rule: <br>
$$
l_{12} : l_{13} : l_{14} : l_{23} : l_{42} : l_{34} = l^*_{34} : l^*_{42} : l^*_{23} : l^*_{14} : l^*_{13} : l^*_{12}  \quad (3.10)
$$

4. The plane defined by the join of the point $$X$$ and line $$L$$ is <br>
$$
\pi = L^* X
$$
<br> and $$L^* X = 0$$ if, and only if, $$X$$ is on L.

5. The point define by the intersection of the line $$L$$ with the plane $$\pi$$ is <br>
$$
X = L \pi
$$
<br> and $$L \pi = 0$$ if, and only if, $$L$$ is on $$\pi$$.

5. __Plücker line coordinates.__ The Plücker line coordinates are the six non-zero elements of the $$4 \times 4$$ skew -symmetric Plücker matrix (3.8) $$L$$, namely <br>
$$
\mathcal{L} = \left \{ l_{12},l_{13},l_{14},l_{23},l_{42},l_{34} \right \}  \quad (3.11)
$$
<br> This is a homogeneous 6-vector, and thus is an element of $$\mathbb{P^5}$$. It follows from evaluating $$\det L = 0$$ that the coordinates satisfy the equation <br>
$$
l_{12}l_{34} + l_{13}l_{42} + l_{14}l_{23} = 0 \quad (3.12)
$$

6. Suppose two lines $$\mathcal{L}, \mathcal{\hat{L}}$$ are the joins of points $$A, B$$ and $$\hat{A}, \hat{B}$$ respectively. The lines intersect if and only if the four points are coplanar. It can be shown as <br>
$$
\begin{align}
\det [A,B,\hat{A},\hat{B}] & = l_{12}\hat{l_{34}} + l_{13}\hat{l_{42}} + l_{14}\hat{l_{23}} + \hat{l_{12}}l_{34} + \hat{l_{13}}l_{42} + \hat{l_{14}}l_{23}  \\
						   & = (\mathcal{L} | \mathcal{\hat{L}})  \quad \quad (3.13)
\end{align}
$$

7. __Result 3.5.__ Two lines $$\mathcal{L}$$ and $$\mathcal{\hat{L}}$$ are coplanar (and thus intersect) if and only if $$ (\mathcal{L}|\mathcal{\hat{L}}) = 0 $$.

8. Suppose two lines $$\mathcal{L}$$ and $$\mathcal{\hat{L}}$$ are the intersections of the planes $$P, Q$$ and $$\hat{P}, \hat{Q}$$ respectively. Then <br>
$$
(\mathcal{L} | \mathcal{\hat{L}}) = \det [P,Q,\hat{P},\hat{Q}]
$$

9. if $$\mathcal{L}$$ is the intersection of two planes $$P$$ and $$Q$$ and $$\mathcal{\hat{L}}$$ is the join of two points $$A$$ and $$B$$, then <br>
$$
(\mathcal{L} | \mathcal{\hat{L}}) = (P^\intercal A)(Q^\intercal B) - (Q^\intercal A)(P^\intercal B) \quad (3.14)
$$

### 3.2.3 Quadrics and dual quadrics

1. A quadric is a surface in $$\mathbb{P^3}$$ defined by the equation <br>
$$
X^\intercal Q X = 0 \quad (3.15)
$$
<br> where $$Q$$ is a symmetric $$4 \times 4$$ matrix.

2. A quadric has 9 degrees of freedom. These correspond to the ten independent elements of a $$4 \times 4$$ symmetric matrix less one for scale.

3. If the matrix $$Q$$ is singular, then the quadric is _degenerate_, and may be defined by fewer points.

4. Under the point transformation $$X^\prime = HX$$, a (point) quadric transforms as <br>
$$
Q^\prime = H^{-\intercal} Q H^{-1} \quad (3.16)
$$

5. The dual of a quadric is also a quadric. Dual quadrics are equations on planes: the tangent planes $$\pi$$ to the point quadric $$Q$$ satisfy $$\pi ^\intercal Q^* \pi = 0$$, where $$Q^* = adjoint Q$$, or $$Q^{-1}$$ if $$Q$$ is invertible. Under the point transformation $$X^\prime = HX$$, a dual quadric transforms as <br>
$$
Q^{* \prime} = H Q^* H^\intercal \quad (3.17)
$$

### 3.2.4 Classification of quadrics

1. __Ruled quadrics.__ Quadrics fall into two classes - ruled and unruled quadrics. A ruled quadric is one that contains a straight line.

## 3.3 Twisted cubics

1. A conic in the 2-dimensional projective plane may be described as a parametrized curve given by the equation <br>
$$
\begin{pmatrix}x_1 \\ x_2 \\ x_3\end{pmatrix} = A \begin{pmatrix} 1\\ \theta\\ \theta ^2 \end{pmatrix} = \begin{pmatrix}
a_{11} + a_{12} \theta + a_{13} \theta ^2\\ a_{21} + a_{22} \theta + a_{23} \theta ^2 \\ a_{31} + a_{32} \theta + a_{33} \theta ^2 \end{pmatrix}   \quad (3.18)
$$
<br> where $$A$$ is a non-singular $$3 \times 3$$ matrix.

2. A twisted cubic is defined to be a curve in $$\mathbb{P^3}$$ given in parametric form as <br>
$$
\begin{pmatrix}X_1 \\ X_2 \\ X_3 \\ X_4 \end{pmatrix} = A \begin{pmatrix} 1\\ \theta\\ \theta ^2 \\ \theta ^3 \end{pmatrix} = \begin{pmatrix}
a_{11} + a_{12} \theta + a_{13} \theta ^2 + a_{14} \theta ^3\\ a_{21} + a_{22} \theta + a_{23} \theta ^2 + a_{24} \theta ^3\\ a_{31} + a_{32} \theta + a_{33} \theta ^2 + a_{34} \theta ^3 \\ a_{41} + a_{42} \theta + a_{43} \theta ^2 + a_{44} \theta ^3 \end{pmatrix}  \quad (3.19)
$$

3. A twisted cubic has 12 degrees of freedom (counted as 15 for the matri A, less 3 for a 1D projectivity on the parametrization $$\theta$$, which leaves the curve unaltered).

4. All twisted cubics are projectively equivalent. 


## 3.4 The hierarchy of transformations
![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/post-geometric-properties-invariant-3D.JPG)

### 3.4.1 The screw decomposition

1. __Result 3.6.__ _Any particular translation and rotation is equivalent to a rotation about a screw axis together with a translation along the screw axis. The screw axis is parallel to the rotation axis._

## 3.5 The plane at infinity

## 3.6 The absolute conic

## 3.7 The absolute dual quadric

## 3.8 Closure


































