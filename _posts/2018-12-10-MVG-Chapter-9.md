---
layout: post
title: "Chapter 9: Epipolar Geometry and the Fundamental Matrix"
date:       2018-12-10
author:     Tong
catalog: true
tags:
    - MVG
---

> All contents come from <<Multiple View Geometry in Computer Vision>>.

### Abstract

The epipolar geometry is the intrinsic projective geometry between two view.

The fundamental matrix $$F$$ encapsulates this intrinsic geometry.

### 9.1 Epipolar geometry

1. This geometry is usually motivated by considering the search for corresponding points in stereo matching.

2. The __epipole__ is the _point_ of intersection of the line joining the camera centres (the baseline) with the image plane. Equivalently, the epipole is the image in one view of the camera centre of the other view. It is also the vanishing point of the baseline (translation) direction.

3. An __epipolar plane__ is a plane containing the baseline. There is a one-parameter family (a pencil) of epipolar planes.

3. An __epipolar line__ is the intersection of an epipolar plane with the image plane. All epipolar lines intersect at the epipole. An epipolar plane intersects the left and the right image planes in epipolar lines, and defines the correspondence between the lines.   

### 9.2 The fundamental matrix $$F$$

1. There is a map $$ x \mapsto l^\prime $$ from a point in one image to its corresponding epipolar line in the other image. This is represented by a matrix $$F$$, the fundamental matrix.

#### 9.2.1 Geometric derivation

1. __Result 9.1.__ The fundamental matrix $$F$$ may be written as $$F = \left [ e^\prime \right ]_ \times H_\pi$$, where $$H_\pi$$ is the transfer mapping from one image to another via any plane $$\pi$$. Furthermore, since $$\left [ e^\prime \right ]_ \times$$ has rank 2 and $$H_\pi$$ rank 3, $$F$$ is a matrix of rank 2.

#### 9.2.2 Algebraic derivation

1. One possible formulat for the fundamental matrix $$F$$ is $$F = \left [ e^\prime \right ]_ \times P^\prime P^+\quad \quad (9.1)$$, where $$P^+$$ is the pseudo-inverse of $$P$$.

#### 9.2.3 Correspondence condition

1. __Result 9.3.__ The fundamental matrix satisfies the condition that for any pair of corresponding points $$x\leftrightarrow x^\prime$$ in the two images $$x^{\prime T} Fx = 0$$.

2. Result 9.3 gives a way of characterizing the fundamental matrix without reference to the camera matrices, i.e. only in terms of corresponding image points.

3. In general, at least 7 correspondences are required to compute $$F$$.

#### 9.2.4 Properties of the fundamental Matrix

1. $$F$$ is a rank 2 homogeneous matrix with 7 degrees of freedom.

#### 9.2.5 The epipolar line homography

1. __Result 9.5.__ Suppose $$l$$ and $$l^\prime$$ are corresponding epipolar lines, and $$k$$ is any line not passing through the epipole $$e$$, then $$l$$ and $$l^\prime$$ are related by $$l^\prime = F \left [ k \right ]_ \times l$$. Symmetrically, $$l = F^T \left [ k^\prime \right ]_ \times l^\prime$$.

### 9.3 Fundamental matrices arising from special motions

1. The "pure" indicates that there is no change in the internal parameters.

#### 9.3.1 Pure translation

1. $$x, x^\prime$$ and $$e=e^\prime$$ are collinear (assuming both images are overlaid on top of each other). This collinearity property is termed $$auto-epipolar$$, and does not hold for general motion.

2. __General motion.__ Given two arbitrary cameras, we may rotate the camera used for the first image so that it is aligned with the second camera. This rotation may be simulated by applying a projective transformation to the first image. A further correction may be applied to the first image to account for any difference in the calibration matrices of the two images. The result of these two corrections is a projective transformation $$H$$ of the first image. If one assumes these corrections have been made, then the effective relationship of the two cameras to each other is that of a pure translation.

#### 9.3.2 Pure planar motion

1. In this case, the rotation axis is orthogonal to the translation direction.

### 9.4 Geometric representation of the fundamental matrix

#### 9.4.1 Pure planar motion

### 9.5 Retrieving the camera matrices

#### 9.5.1 Projective invariance and canonical cameras

1. __Result 9.8__ If $$H$$ is a $$4 \times 4$$ matrix representing a projective transformation of 3-space, then the fundamental matrices corresponding to the pairs of camera matrices $$(P, P^\prime)$$ and $$(PH, P^\prime H)$$ are the same.

2. The fundamental matrix is unchanged by a projective transformation of 3-space.

3. __Result 9.9.__ The fundamental matrix corresponding to a pair of camera matrices $$P= \left [ I \;\; 0 \right ]$$ and $$P^\prime= \left [ M \;\; m \right ]$$ is euqal to $$\left [ m \right ]_ \times M$$.

#### 9.5.2 Projective ambiguity of cameras given $$F$$

#### 9.5.3 Canonical cameras given $$F$$

1. __Result 9.12.__ A non-zero matrix $$F$$ is the fundamental matrix corresponding to a pair of camera matrices $$P$$ and $$P^\prime$$ if and only if $$P^{\prime T}FP$$ is skew-symmetric.

2. __Result 9.15.__ The general formula for a pair of canonic camera matrices corresponding to a fundamental matrix $$F$$ is given by $$P= \left [ I \;\; 0 \right ] \quad \quad \quad P= \left [ \left [ e^\prime \right ]_ \times F+e^\prime v^T \;\; \lambda e^\prime \right ] \quad \quad (9.10)$$, where $$v$$ is any 3-vector, and $$\lambda$$ a non-zero scalar.

### 9.6 The essential matrix

1. The essential matrix is the specialization of the fundamental matrix to the case of normalized image coordinates.

2. Let $$\hat{x} = K^{-1} x$$, then $$\hat{x} = \left [  R \; \; t\right ]X$$, where $$\hat{x}$$ is the image point expressed in _normalized coordinates_. It may be thought of as the image of the point $$X$$ with respect to a camera $$\left [  R \; \; t\right ]$$ having the identity matrix $$I$$ as calibration matrix.

3. __Definition 9.16.__ The defining equation for the essential matrix is $$\hat{x}^{\prime T} E \hat{x} = 0 \quad \quad (9.11)$$ in terms of the normalized image coordinates for corresponding points $$x \leftrightarrow x^\prime$$.

4. The relationship between the fundamental and essential matrices is $$E=K^{\prime T} FK \quad \quad (9.12)$$.

#### 9.6.1 Properties of the essential matrix

1. $$E$$ has five DOF.

2. __Result 9.17.__ A $$3 \times 3$$ matrix is an essential matrix if and only if two of its singular values ar equal, and the third is zero.

#### 9.6.2 Extraction of cameras from the essential matrix

#### 9.6.3 Geometrical interpretation of the four solutions

1. There are four solutions for the second camera matrix, where a reconstructed point $$X$$ will be in front of both cameras in one of these solutions only. 

### 9.7 Closure
