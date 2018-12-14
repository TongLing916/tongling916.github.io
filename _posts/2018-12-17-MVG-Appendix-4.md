---
layout: post
title: "Appendix 4: Matrix Properties and Decompositions"
date:       2018-12-17
author:     Tong
catalog: true
tags:
    - MVG
---

> All contents come from <<Multiple View Geometry in Computer Vision>>.

### A4.1 Orthogonal matrices

1. The orthogonal matrices of dimension $$n$$ with positive determinant form a group, called $$SO_n$$. An element of $$SO_n$$ is called an $$n$$-dimensional rotation.

2. __Norm-preserving properties of orthogonal matrices.__ An important property of orthogonal matrices is that multiplying a vector by an orthogonal matrix preserves its norm. ($$(Ux)^T(Ux) = x^TU^TUx=x^Tx$$)

3. By the QR decomposition of a matrix is usually meant the decomposition of the matrix $$A$$ into a product $$A=QR$$, where $$Q$$ is orthogonal, and $$R$$ is an upper-triangular matrix. (R: Right, upper-triangular; L: Left, lower-triangular matrix)

#### A4.1.1 Givens rotations and RQ decomposition

1. A 3-dimensional Givens rotation a rotation about one of the three coordinates axes. The three Givens rotations are $$Q_x = \begin{bmatrix}1 &  & \\  & c & -s \\ & s & c\end{bmatrix}\quad \quad Q_y = \begin{bmatrix}c &  & s\\  & 1 & \\ -s & & c\end{bmatrix}\quad \quad Q_z = \begin{bmatrix}c & -s &\\ s & c & \\  & & 1\end{bmatrix}\quad \quad (A4.1)$$ where $$c=\cos (\theta)$$ and $$s=\sin (\theta)$$ for some angle $$\theta$$ and blank entries represent zeros. 

#### A4.1.2 Housholder matrices and QR decomposition

### A4.2 Symmetric and skew-symmetric matrices

#### A4.2.1 Positive-definite symmetric matrices

### A4.3 Representations of rotation matrices

#### A4.3.1 Rotations in n-dimensions

1.

#### A4.3.2 Rotations in 3-dimensions

1.

#### A4.3.3 Quaternions

1.

### A4.4 Singular value decomposition
