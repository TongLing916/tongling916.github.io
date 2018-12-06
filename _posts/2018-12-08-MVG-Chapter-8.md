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

3. __ Result 8.9.__ Under the camera matrix $$P$$, the outline of the quadric $$Q$$ is the conic $$C$$ given by $$C^* = P Q^* P^T \quad \quad (8.5)$$.

4. The plane of $$\Gamma$$ for a quadric $$Q$$ and camera with centre $$C$$ is given by $$\pi_\Gamma = QC$$.

5. __Result 8.10.__ The cone with vertex $$V$$ and tangent to the quadric $$Q$$ is the degenerate quadric $$Q_{CO} = (V^T QV)Q - (QV)(QV)^T$$. Note that $$Q_{CO} V = 0$$, so that $$V$$ is the vertex of the cone as required.

### 8.4 The importance of the camera centre

1.

#### 8.4.1 Moving the image plane

1.

#### 8.4.2 Camera rotation

1.


#### 8.4.3 Applications and examples

1.

#### 8.4.4 Projective (reduced) notation

#### 8.4.5 Moving the camera centre

### 8.5 Camera calibration and the image of the absolute conic

#### 8.5.1 The image of the absolute conic

#### 8.5.2 Orthogonality and $$\omega$$

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
