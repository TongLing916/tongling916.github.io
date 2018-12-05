---
layout: post
title: "Chapter 6: Camera Models"
date:       2018-12-05
author:     Tong
catalog: true
tags:
    - MVG
---

> All contents come from <<Multiple View Geometry in Computer Vision>>.

### Abstract

This chapter describes the projection of 3D scene space onto a 2D image plane. There are two particularly important classes of camera matrix: finite cameras, and cameras with their centre at infinity such as the _affine camera_ which represents parallel projection.

The principal camera of interest in this book is _central projection_. All cameras modelling central projection are specializations of the _general projective camera_.

### 6.1 Finite cameras

1. __The basic pinhole model.__ Let the centre of projection be the origin of a Euclidean coordinate system, and consider the plane $$Z = f$$, which is called the _image plane_ or _focal plane_. Under the pinhole camera model, a point in space is mapped to the point on the image plane where a line joining the point to the centre of projection meets the image plane. We see that $$ (X, Y, Z)^T \mapsto (fX/Z, fY/Z)^T \quad \quad (6.1)$$ describes the central projection mapping from world to image coordinates.

2. The centre of projection is called the _camera centre_ or the _optical centre_. The line from the camera centre perpendicular to the image plane is called the _principal axis_ or _principal ray_ of the camera. The plane through the camera centre parallel to the image plane is called the _principal plane_ of the camera.

3. __Central projection using homogeneous coordinates.__ (6.1) can be written by using a $$P$$, the $$3 \times 4$$ homogeneous _camera projection matrix_. Then $$x = PX \quad \quad (6.2)$$ which defines the camera matrix for the pinhole model of central projection as $P = diag(f,f,1)\left [ I | 0\right ]$$.

4. __Principal point offset.__ The expression (6.1) assumed that the origin of coordinates in the image plane is at the principal point. In genral, there is a mapping $$ (X, Y, Z)^T \mapsto (fX/Z + p_x, fY/Z + p_y)^T $$ where $$(p_x, p_y)^T$$ are the coordinates of the principal point. Now $$\begin{pmatrix} X\\ Y\\ Z\\ 1\end{pmatrix} \mapsto \begin{pmatrix} fX + Zp_x\\ fY + Zp_y\\ 1\end{pmatrix} = \begin{bmatrix}
f &  & p_x & 0\\
 & f & p_y & 0\\
 &  & 1 & 0
\end{bmatrix}\begin{pmatrix}
X\\
Y\\
Z\\
1
\end{pmatrix} \quad \quad (6.3)$$ and $$ K = \begin{bmatrix}
f &  & p_x \\
 & f & p_y \\
 &  & 1
\end{bmatrix}\quad \quad (6.4)$$. Then $$ x = K\left [ I | 0\right ] X_{cam}\quad \quad (6.5)$$ where the matrix $$K$$ is called the _camera calibration matrix_ and the point $$X_{cam}$$ is expressed in the _camera coordinate frame_.

5. 

### 6.2 The projective camera

1.

#### 6.2.1 Camera anatomy

1.

#### 6.2.2 Action of a projective camera on points

1.

#### 6.2.3. Depth of points

1.

#### 6.2.4 Decomposition of the camera matrix

1.

#### 6.2.5. Euclidean vs projective spaces

1.

### 6.3 Cameras at infinity

1.

#### 6.3.1 Affine cameras

1.

#### 6.3.2 Error in employing an affine camera

1.

#### 6.3.3 Decomposition of $$P_{\infty}$$

1.

#### 6.3.4 A hierarchy of affine camera

1.

#### 6.3.5 More properties of the affine camera

1.

#### 6.3.6 General cameras at infinity

1.

### 6.4 Other camera models

#### 6.4.1 Pushbroom Cameras

#### 6.4.2 Line cameras

### 6.5 Closure

1.
