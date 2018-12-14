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

2. __Algorithm A4.1.__ RQ decomposition of a $$3 \times 3$$ matrix.
![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/post-RQ-decomposition.JPG)

3. $$AQ_xQ_yQ_z = R$$ where $$R$$ is upper-triangular. Consequently, $$A=RQ_z^TQ_y^TQ_x^T$$ $$ and so $$A=RQ$$ where $$Q=Q_z^TQ_y^TQ_x^T$$ is a rotation.

#### A4.1.2 Housholder matrices and QR decomposition

1. For matrices of larger dimension, the QR decompposition is more efficiently carried out using Householder matrices.

### A4.2 Symmetric and skew-symmetric matrices

1. A matrix is called _symmetric_ if $$A^T=A$$ and _skew-symmetric_ if $$A^T=-A$$.

2. __Result A4.1.__ Eigenvalue decomposition.

3. __Result A4.4.__ If $$F=\left [ e^\prime \right ]_ \times M$$ is a fundamental matrix (a $$3 \times 3$$ singular matrix), then $$\left [ e^\prime \right ]_ \times \left [ e^\prime \right ]_ \times F = F$$ (up to scale). Hence, one may decompose $$F$$ as $$F=\left [ e^\prime \right ]_ \times M$$, where $$M=\left [ e^\prime \right ]_ \times F$$.

#### A4.2.1 Positive-definite symmetric matrices

### A4.3 Representations of rotation matrices

#### A4.3.1 Rotations in n-dimensions

1. The matrices $$e^S$$ where $$S$$ is an $$n \times n$$ skew-symmetric are exactly the set of $$n$$-dimensional rotation matrices.

#### A4.3.2 Rotations in 3-dimensions

1. __Result A4.6.__ The matrix $$e^{\left [ t \right ]_ \times}$$ is a rotation matrix representing a rotation through an angle $$\left \| t \right \|$$ about the axis represented by the vector $$t$$. This representation of a rotation is called the _angle-axis_ representation.

#### A4.3.3 Quaternions

### A4.4 Singular value decomposition

1. Given a square matrix $$A$$, the $$SVD$$ is a factorization of $$A$$ as $$A=UDV^T$$, where $$U$$ and $$V$$ are orthogonal matrices, and $$D$$ is a diagonal matrix with non-negative entries. We will assume that the diagonal entries of $$D$$ are in descending order.

2. The entries in $$D$$ are the singular values of the matrix $$A$$.
