---
layout: post
title: "Chapter 8: More Single View Geometry"
date:       2018-12-08
author:     Tong
catalog: true
tags:
    - MVG
---

> All contents come from <<Multiple View Geometry in Computer Vision>>.

### Abstract

This chapter describes the link between other 3D entities and their images under perspective projection. These entities include planes, lines, conics and quadrics; and we develop their forward and back-projection properties.

### 8.1 Action of a projective camera on planes, lines, and conics

#### 8.1.1 On planes

1. The most general transformation that can occur between a scene plane and an image plane under perspective imaging is a plane projective transformation.

#### 8.1.2 On lines

1. __Forward projection.__ A line in 3-space projects to a line in the image.

2. __Back-projection of lines.__ The set of points in space which map to a line in the image is a plane in space defined by the camera centre and image line.

3. __Result 8.2.__ The set of points in space mapping to a line $$l$$ via the camera matrix $$P$$ is the plane $$P^T l$$.

#### 8.1.3 On conics

1. __Back-projection of conics.__ A conic $$C$$ back-projects to a cone. A cone is degenerate quadric, i.e. the $$4 \times 4$$ matrix representing the quadric does not have full rank. The cone vertex, in this case the camera centre, is the null-vector of the quadric matrix.

2. __Result 8.6.__ Under the camera $$P$$ the conic $$C$$ back-projects to the cone $$Q_{CO} = P^T CP$$.

### 8.2 Images of smooth surfaces

1. The image outline of a smooth surface $$S$$ results from surface points at which the imaging rays are _tangent_ to the surface.

2. __Definition 8.8.__ The _contour generator_ $$\Gamma $$ is the set of points $$X$$ on $$S$$ at which rays are tangent to the surface. The corresponding image _apparent contour_ $$\gamma$$ is the set of points $$x$$ which are the image of $$X$$, i.e. $$\gamma$$ is the image of $$\Gamma $$.

### 8.3 Action of a projective camera on quadrics

1. A quadric is a smooth surface and so its outline curve is given by points where the back-projected rays are tangent to the quadric surface.

2. __Forward projection of quadrics.__ Since the outline arises from tangency, it is not surprising that the dual of the quadric, $$Q^* $$, is important here since it defines the tangent planes to the quadric $$Q$$.

3. __Result 8.9.__ Under the camera matrix $$P$$, the outline of the quadric $$Q$$ is the conic $$C$$ given by $$C^* = P Q^* P^T \quad \quad (8.5)$$.

4. The plane of $$\Gamma$$ for a quadric $$Q$$ and camera with centre $$C$$ is given by $$\pi_\Gamma = QC$$.

5. __Result 8.10.__ The cone with vertex $$V$$ and tangent to the quadric $$Q$$ is the degenerate quadric $$Q_{CO} = (V^T QV)Q - (QV)(QV)^T$$. Note that $$Q_{CO} V = 0$$, so that $$V$$ is the vertex of the cone as required.

### 8.4 The importance of the camera centre

1. An object in 3-space and camera centre define a set of rays, and an image is obtained by intersecting these rays with a plane. Often this set is referred to as a _cone_ of rays.

#### 8.4.1 Moving the image plane

1. The effect of zooming by a factor $$k$$ is to multiply the calibration matrix $$K$$ on the right by $$diag(k,k,1)$$.

#### 8.4.2 Camera rotation

1. The homography is a _conjugate rotation_.

2. The transformation $$H=KRK^{-1}$$ is an example of the _infinite homography mapping_ $$H_\infty$$.


#### 8.4.3 Applications and examples

1. The homographic relation between images with the same camera centre can be exploited in several ways. 1) One is the creation of synthetic image by projective warping. 2) Another is mosaicing, where panoramic images can be created by using planar homographies to "sew" together views obtained by a rotating camera.

2. Algorithm of synthetic views: 1) Compute the homography $$H$$ which maps the image quadrilateral to a a rectangle with the correct aspect ratio. 2) Projectively warp the source image with this homography.

3. Algorithm of planar panoramic mosaicing: 1) Choose one image of the set as a reference. 2) Compute the homography $$H$$ which maps one of the other images of the set to this reference image. 3) Projectively warp the image with this homography, and augment the reference image with the non-overlapping part of the warped image. 4) Repeat the last two steps for the remaining images of the set.

#### 8.4.4 Projective (reduced) notation

1. __reduced camera matrix.__

2. All images acquired by cameras with the same camera centre are projectively equivalent.

#### 8.4.5 Moving the camera centre

1. If the camera centre is moved, then the map between corresponding image points __does__ depend on the 3-space structure,and indeed may often be used to (partially) determine the structure.

2. How can one determine from the images alone whether the camera centre has moved? Consider two 3-space points which have coincident images in the first view, i.e., the points are on the same ray. If the camera centre is moved (not along the ray), the image coincidence is lost. This relative displacement of previously coincident image points is termed _parallax_.

3. A convenient method for obtaining a camera motion that is only a rotation about its centre is to adjust the motion until there is no parallax.

4. An important special case of 3-space structure is when all scene points are coplanar. In this case, the images of corresponding points are related by a planar homography even if the camera centre is moved.

### 8.5 Camera calibration and the image of the absolute conic

1. What is gained if the camera internal calibration, $$K$$, is known? Euclidean properties, such as the angle between two rays, can then be measured.

2. __Result 8.15.__ The camera calibration matrix $$K$$ is the (affine) transformation between $$x$$ and the ray's direction $$d=K^{-1}x$$ measured in the camera's Euclidean coordinate frame.

3. __Result 8.16.__ An image line $$l$$ defines a plane through the camera centre with normal direction $$n=K^T l$$ measured in the camera's Euclidean coordinate frame.

#### 8.5.1 The image of the absolute conic

1. The mapping between the plane at infinity $$\pi_\infty$$ and an image is given by the planar homography $$x=Hd$$ with $$H=KR \quad \quad (8.8)$$.

2. __Result 8.17.__ The image of the absolute conic (the IAC) is the conic $$\omega = (KK^T)^{-1} = K^{-T}K^{-1}$$.

3. The dual image of the absolute conic (the DIAC): $$\omega^* = \omega^{-1} = KK^T \quad \quad (8.11)$$.

4. Result 8.17 shows that once $$\omega$$ (or equivalently $$\omega^* )$$ is identified in an image, then $$K$$ is also determined.

5. A plane $$\pi$$ intersects $$\pi_\infty$$ in a line, and this line intersects the absolute conic $$\Omega_\infty$$ in two points which are the circular points of $$\pi$$. The imaged circular points lie on $$\omega$$ at the points at which the vanishing line of the line of the plane $$\pi$$ intersects $$\omega$$.

6. (4) and (5) are the basis for a calibration algorithm.

#### 8.5.2 Orthogonality and $$\omega$$

1. __Result 8.19.__ A point $$x$$ and line $$l$$ back-projecting to a ray and plane respectively that are orthogonal are related by $$l=\omega x$$.

### 8.6 Vanishing points and vanishing lines

#### 8.6.1 Vanishing points

#### 8.6.2 Vanishing lines

#### 8.6.3 Orthogonality relationships amongst vanishing points and lines

### 8.7 Affine 3D measurements and reconstruction

### 8.8 Determining camera calibration K from a single View

#### 8.8.1 The geometry of the constraints

### 8.9 Single view reconstruction

### 8.10 The calibration conic

### 8.11 Closure

1.
