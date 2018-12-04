---
layout: post
title: "Chapter 3: Projective Geometry and Transformations of 3D"
date:       2018-10-27
author:     Tong
catalog: true
tags:
    - MVG
---

> All contents come from <<Multiple View Geometry in Computer Vision>>.
### Abstract

This chapter describes the properties and entities of projective 3-space, or $$\mathbb{P}^3$$.

### 3.1 Points and projective transformations

1. In $$\mathbb{P^3}$$ Euclidean 3-space is augmented with a set of ideal points which are on a _plane_ at infinity, $$\pi_\infty$$. Parallel lines, and now parallel _planes_, intersect on $$\pi_\infty$$.

### 3.2 Representing and transforming planes, lines and quadrics

#### 3.2.1 Planes

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


#### 3.2.2 Lines

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
<br> Under the point transformation $$X^\prime = HX$$, the matrix $$L^*$$ transforms as $$L^{*\prime} = H^\intercal L H^{-1}$$. The matrix $$L^*$$ can be obtained directly from $$L$$ by a simple rewrite rule: <br>
$$
l_{12} : l_{13} : l_{14} : l_{23} : l_{42} : l_{34} = l^*_{34} : l^*_{42} : l^*_{23} : l^*_{14} : l^*_{13} : l^*_{12}  \quad (3.10)
$$

4. The plane defined by the join of the point $$X$$ and line $$L$$ is <br>
$$
\pi = L^* X
$$
<br> and $$L^* X = 0$$ if, and only if, $$X$$ is on $$L$$.

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

7. __Result 3.5.__ Two lines $$\mathcal{L}$$ and $$\mathcal{\hat{L}}$$ are coplanar (and thus intersect) if and only if <br>
$$
(\mathcal{L}|\mathcal{\hat{L}}) = 0
$$

8. Suppose two lines $$\mathcal{L}$$ and $$\mathcal{\hat{L}}$$ are the intersections of the planes $$P, Q$$ and $$\hat{P}, \hat{Q}$$ respectively. Then <br>
$$
(\mathcal{L} | \mathcal{\hat{L}}) = \det [P,Q,\hat{P},\hat{Q}]
$$

9. if $$\mathcal{L}$$ is the intersection of two planes $$P$$ and $$Q$$ and $$\mathcal{\hat{L}}$$ is the join of two points $$A$$ and $$B$$, then <br>
$$
(\mathcal{L} | \mathcal{\hat{L}}) = (P^\intercal A)(Q^\intercal B) - (Q^\intercal A)(P^\intercal B) \quad (3.14)
$$

#### 3.2.3 Quadrics and dual quadrics

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

#### 3.2.4 Classification of quadrics

1. __Ruled quadrics.__ Quadrics fall into two classes - ruled and unruled quadrics. A ruled quadric is one that contains a straight line.

### 3.3 Twisted cubics

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


### 3.4 The hierarchy of transformations
![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/post-geometric-properties-invariant-3D.JPG)

#### 3.4.1 The screw decomposition

1. __Result 3.6.__ _Any particular translation and rotation is equivalent to a rotation about a screw axis together with a translation along the screw axis. The screw axis is parallel to the rotation axis._

### 3.5 The plane at infinity

1. The plane at infinity has the canonical position $$\pi_infty = (0,0,0,1)^\intercal$$ in affine 3-space.It contains the directions $$D = (x_1,x_2,x_3,0)^\intercal$$, and enables the identification of affine properties such as parallelism. In particular:
	- Two palnes are parallel if, and only if, their line of intersection is on $$\pi_\infty$$.
	- A line is parallel to another line, or to a plane, if the point of intersection is on $$\pi_\infty$$.

2. The plane $$\pi_\infty$$ is a geometric representation of the 3 degrees of freedom required to specify affine properties in a projective coordinate frame. In loose terms, the plane at infinity is a fixed plane under any affine transformation, but "sees" (is moved by) a projective transformation.

3. __Result 3.7.__ _The plane at infinity, $$\pi_\infty$$, is a fixed plane under the projective transformation $$H$$ if, and only if, $$H$$ is an affinity._

4. __Affine properties of a reconstruction.__ Once $$\pi_\infty$$ is identified in projective 3-space, i.e. its projective coordinates are known, it is then possible to determine affine properties of the reconstruction such as whether geometric entities are parallel - they are parallel if they intersect on $$\pi_\infty$$.

### 3.6 The absolute conic

1. The absolute conic, $$\Omega_\infty$$, is a (point) conic on $$\pi_\infty$$. Points on $$\Omega_\infty$$ satisfy <br>
$$
\left.\begin{matrix} X_1^2 + X_2^2 + X_3^2 \\ X_4  \end{matrix}\right\} = 0  \quad (3.21)
$$

2. For directions on $$\pi_\infty$$ (i.e. points with $$X_4 = 0$$) the defining equation can be written <br>
$$
(X_1,X_2,X_3)I(X_1,X_2,X_3)^\intercal = 0
$$
<br> so that $$\Omega_\infty$$ corresponds to a conic $$C$$ with matrix $$C = I$$. It is thus a conic of purely imaginary points on $$\pi_\infty$$.

3. The conic $$\Omega_\infty$$ is a geometric representation of the 5 additional degrees of freedom that are required to specify metric properties in an affine coordinate frame. A key property of $$\Omega_\infty$$ is that it is a fixed conic under any similarity transformation.

4. __Result 3.9.__ _The absolute conic, $$\Omega_\infty$$, is a fixed conic under the projective transformation $$H$$ if, and only if, $$H$$ is a similarity transformation._

5. __Metric properties.__ Consider two lines with directions (3-vectors) $$d_1$$ and $$d_2$$. The angle between these directions in a Euclidean world frame is given by <br>
$$
\cos\theta = \frac{d_1^\intercal d_2}{\sqrt{(d_1^\intercal d_1)(d_2^\intercal d_2))}} \quad (3.22)
$$
<br> This may be written as <br>
$$
\cos\theta = \frac{d_1^\intercal \Omega_\infty d_2}{\sqrt{(d_1^\intercal \Omega_\infty d_1)(d_2^\intercal \Omega_\infty d_2))}} \quad (3.23)
$$
<br> where $$d_1$$ and $$d_2$$ are the points of intersections of the lines with the plane $$\pi_\infty$$ containing the conic $$\Omega_\infty$$, and $$\Omega_\infty$$ is the matrix representation of the absolute conic in that plane.

### 3.7 The absolute dual quadric

1. The dual of the absolute conic $$\Omega_\infty$$ is a degenerate dual _quadric_ in 3-space called the _absolute dual quadric_, and denoted $$Q_\infty^*$$. Geometrically $$Q_\infty^*$$ consists of the planes tangent to $$\Omega_\infty$$, so that $$\Omega_\infty$$ is the "rim" of $$Q_\infty^*$$. This is called a _rim quadric_.

2. $$Q_\infty^*$$ is represented by a $$4 \times 4$$ homogeneous matrix of rank 3, which in metirc 3-space has the canonical form <br>
$$
Q_\infty^* = \begin{bmatrix} I & 0 \\ 0^\intercal & 0 \end{bmatrix}   \quad (3.24)
$$

3. The dual quadric $$Q_\infty^*$$ is a degenerate quadric and has 8 degrees of freedom (a symmetric matrix has 10 independent elements, but irrelevant scale and zero determinant condition each reduce the degrees of freedom by 1).

4. __Result 3.10.__ _The absolute dual quadric, $$Q_\infty^*$$, is fixed under the projective transformation $$H$$ if, and only if, $$H$$ is a similarity._

5. __Result 3.11.__ _The plane at infinity $$\pi_\infty$$ is the null-vector of $$Q_\infty^*$$._

6. __Result 3.12.__ _The angle between two planes $$\pi_1$$ and $$\pi_2$$ is given by_ <br>
$$
\cos\theta = \frac{\pi_1^\intercal Q_\infty^* \pi_2}{\sqrt{(\pi_1^\intercal Q_\infty^* \pi_1)(\pi_2^\intercal Q_\infty^* \pi_2))}} \quad (3.25)
$$


### 3.8 Closure
