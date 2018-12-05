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

3. __Central projection using homogeneous coordinates.__ (6.1) can be written by using a $$P$$, the $$3 \times 4$$ homogeneous _camera projection matrix_. Then $$x = PX \quad \quad (6.2)$$ which defines the camera matrix for the pinhole model of central projection as $$P = diag(f,f,1)\left [ I | 0\right ]$$.

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

5. __Camera rotation and translation.__ In general, points in space will be expressed in the _world coordinate frame_. Convention: $$\tilde{X}$$ is inhomogeneous.

6. Now $$\tilde{X}_{cam} = R(\tilde X - \tilde C)$$, where $$\tilde{C}$$ represents the coordinates of the camera centre in the world coordinate frame, and $$R$$ is a $$3 \times 3$$ rotation matrix representing the orientation of the camera coordinate frame. The equation may be written as $$X_{cam} = \begin{bmatrix}
R & -R \tilde C\\
0 & 1
\end{bmatrix} \begin{pmatrix}
X\\
Y\\
Z\\
1
\end{pmatrix} = \begin{bmatrix}
R & -R \tilde C\\
0 & 1
\end{bmatrix} X \quad \quad (6.6)$$. Putting this together with (6.5) leads to the formula $$x = KR\left [ I | -\tilde C\right ] X \quad \quad (6.7)$$ where $$X$$ is now in a world coordinate frame.

7. One sees that a general pinhole camera, $$P = KR \left [ I \;  -\tilde C \right ] $$, has 9 degrees of freedom: 3 for $$K$$, 3 for $$R$$, and 3 for $$C$$. The parameters of $$K$$ are called the _internal_ camera paraters or the _internal orientation_ of the camera. The parameters of $$R$$ and $$\tilde C$$ which relate the camera orientation and position to a world coordinate system are called the _external_ parameters or the _exterior orientation_.

8. It is often to write $$\tilde{X}_{cam} = R\tilde X + t$$. Then $$P = K\left[R \; t\right] \quad \quad (6.8)$$ where $$t = -R \tilde C$$.

9. __CCD cameras.__ CCD: Charge-coupled device. The pinhole camera model assumes that image coordinates are Euclidean coordinates having equal scales in both axial directions. In the case of CCD cameras, there is the additional possibility of having non-square pixels. If the number of pixels per unit distance in image coordinates are $$m_x$$ and $$m_y$$ in the $$x$$ and $$y$$ directions, then the general form of the calibration matrix of a CCD camera is $$ K = \begin{bmatrix}
\alpha_x &  & x_0 \\
 & \alpha_y & y_0 \\
 &  & 1
\end{bmatrix}\quad \quad (6.9)$$ where $$\alpha_x = fm_x$$ and $$\alpha_y = fm_y$$ represent the focal length of the camera and $$x_0 = m_x p_x$$ and $$y_0 = m_y p_y$$ in terms of pixel dimensions. A CCD camera thus has 10 DOF.

10. __Finite projective camera.__ A calibration matrix of the form $$K = \begin{bmatrix}
\alpha_x & s & x_0 \\
 & \alpha_y & y_0 \\
 &  & 1
\end{bmatrix} \quad \quad (6.10)$$. The added parameter $$s$$ is referred to as the _skew_ parameter. Then, a camera $$KR\left [ I | -\tilde C\right ] \quad \quad (6.11)$$ for which the calibration matrix $$K$$ is of the form (6.10) will be called a _finite projective_ camera with 11 DOF.

11. The set of camera matrices of finite projective cameras is identical with the set of homogeneous $$3 \times 4$$ matrices for which the left hand $$3 \times 3$$ submatrix is non-singular.

12. __General projective cameras.__ A _general projective_ camera is one represented by an arbitrary homogeneous $$3 \times 4$$ matrix of rank 3. It has 11 DOF.

### 6.2 The projective camera

#### 6.2.1 Camera anatomy

1. A general projective camera may be decomposed into blocks according to $$P = [M \; p_4]$$, where $$M$$ is a $$3 \times 3$$ matrix. It will be seen if $$M$$ is non-singular, then this is a finite camera, otherwise it is not.

2. Camera centre, column vectors, row vectors, the principal plane, axis plane, the principal point, the principal axis vector.

3. $$v = det(M)m^3$$ is a vector in the direction of the principal axis, directed towards the front of the camera. ($$m^{3T}$$ is the third row of $$M$$)

#### 6.2.2 Action of a projective camera on points

1. __Forward projection.__

2. __Back-projection of points to rays.__

#### 6.2.3. Depth of points

1. __Result 6.1.__ Let $$X = (X, Y, Z, T)^T be a 3D point and $$P = [M p_4]$$ be a camera matrix for a finite camera. Suppose $$P(X,Y,Z,T)^T = w(x,y,1)^T$$. Then $$depth(X;P) = \frac{sign(det\: M)\omega }{T\left \| m^3 \right \|} \quad \quad (6.15)$$ is the depth of the point $$X$$ in front of the principal plane of the camera.

2. This formula is an effective way to determine if a point $$X$$ is in front of the camera. One verifies that the value of $$depth(X;P)$$ is unchanged if either the point $$X$$ or the camera matrix $$P$$ is multiplied by a constant factor $$k$$.

#### 6.2.4 Decomposition of the camera matrix

1. __Finding the camera centre.__ The camera centre $$C$$ is the point for which $$ PC = 0$$. Use SVD of P to obtain C.

2. __Finding the camera orientation and internal parameters.__

3. __When is $$s \neq 0$$?__ In realistic circumstances, a non-zero skew might arise as a result of taking an image of an image.

4. __Where is the decomposition required?__

5. __A not on coordinate orientation.__ In the derivation of the camera model and its parametrization, it is assumed that the coordinate system used in both the image and the 3D world are __right handed systems__. However, it is very common to use a __left handed coordinate system__. Then, the formula will be different.  

#### 6.2.5. Euclidean vs projective spaces

1. __Euclidean and affine interpretations.__ Although a (finite) $$3 \times 4$$ matrix can always be decomposed to obtain a rotation matrix, a calibration matrix, and so forth, Euclidean interpretations of the parameters so obtained are only meaningful if the image and space coordinates are in an appropriate frame.

2. In the decomposition case, a Euclidean frame is required for both image and 3-space.

3. The interpretation of the null-vector of $$P$$ as the camera centre is valid even if both frames are projective - the interpretation requires only collinearity, which is a projective notion.

4. The interpretation of $$P^3$$ as the principal plane requires at least affine frames for the image and 3-space.

4. The interpretation of $$m^3$$ as the principal ray requires an affine image frame but a Euclidean world frame in order for the concept of orthogonality (to the principal plane) to be meaningful.

### 6.3 Cameras at infinity

1. Cameras at infinity: 1) affine cameras 2) non-affine cameras.

2. __Definition 6.3.__ An _affine_ camera is one that has a camera matrix $$P$$ in which the last row $$P^{3T}$$ if of the form $$(0,0,0,1)$$.

3. It is called an affine camra because points at infinity are mapped to points at infinity.

#### 6.3.1 Affine cameras

#### 6.3.2 Error in employing an affine camera

1. __Affine imaging conditions.__

#### 6.3.3 Decomposition of $$P_{\infty}$$

1. __Parallel projection.__ The essential differences between $$P_\infty$$ and a finite camera.

#### 6.3.4 A hierarchy of affine camera

1. __Orthographic projection.__

2. __Scaled orthographic projection.__

3. __Weak perspective projection.__

4. __The affine camera.__

#### 6.3.5 More properties of the affine camera

#### 6.3.6 General cameras at infinity

### 6.4 Other camera models

#### 6.4.1 Pushbroom Cameras

#### 6.4.2 Line cameras

### 6.5 Closure

1. The affine camera is the most genral linear mapping on homogeneous coordinates that maps parallel world lines to parallel image lines. 
