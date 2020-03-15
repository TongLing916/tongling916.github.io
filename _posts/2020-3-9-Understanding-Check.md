---
layout:     post
title:      "Vision Algorithms for Mobile Robotics - Understanding Check"
date:       2020-3-9
author:     Tong
catalog: true
tags:
    - SLAM
---

### 1. Introduction to Computer Vision and Visual Odometry

- Provide a definition of Visual Odometry?
- Explain the most important differences between VO, VSLAM and SFM?
- Describe the needed assumptions for VO?
- Illustrate its building blocks

### 2. Image Formation 1: perspective projection and camera models

- Explain what a blur circle is?
- Derive the thin lens equation and perform the pinhole approximation?
- Define vanishing points and lines?
- Prove that parallel lines intersect at vanishing points?
- Explain how to build an Ames room?
- Derive a relation between the field of view and the focal length?
- Explain the perspective projection equation, including lens distortion and world to camera projection?

### 3. Image Formation 2: camera calibration algorithms

- Describe the general PnP problem and derive the behavior of its solutions?
- Explain the working principle of the P3P algorithm?
Explain and derive the DLT? What is the minimum number of point
correspondences it requires?
- Define central and non central omnidirectional cameras?
- What kind of mirrors ensure central projection?

### 4. Filtering & Edge detection

- Explain the differences between convolution and correlation?
- Explain the differences between a box filter and a Gaussian filter
- Explain why should one increase the size of the kernel of a Gaussian filter if is large (i.e. close to the size of the filter kernel?
- Explain when would we need a median & bilateral filter?
- Explain how to handle boundary issues?
- Explain the working principle of edge detection with a 1D signal?
- Explain how noise does affect this procedure?
- Explain the differential property of convolution?
- Show how to compute the first derivative of an image intensity function along 𝑥and 𝑦
- Explain why the Laplacian of Gaussian operator is useful?
- List the properties of smoothing and derivative filters
- Illustrate the Canny edge detection algorithm?
- Explain what non maxima suppression is and how it is implemented?

### 5. Point Feature Detectors, Part 1

- Explain what is template matching and how it is implemented?
- Explain what are the limitations of template matching? Can you use it to recognize cars?
- Illustrate the similarity metrics SSD, SAD, NCC, and Census transform?
- What is the intuitive explanation behind SSD and NCC?
- Explain what are good features to track? In particular, can you explain what are corners and blobs together with their pros and cons?
- Explain the Harris corner detector? In particular:
    - Use the Moravec definition of corner, edge and flat region.
    - Show how to get the second moment matrix from the definition of SSD and first order approximation (show that this is a quadratic expression) and what is the intrinsic interpretation of the second moment matrix using an ellipse?
    - What is the M matrix like for an edge, for a flat region, for an axis aligned 90 degree corner and for a non axis aligned 90 degree corner?
    - What do the eigenvalues of M reveal?
    - Can you compare Harris detection with Shi Tomasi detection?
    - Can you explain whether the Harris detector is invariant to illumination or scale changes? Is it invariant to view point changes?
    - What is the repeatability of the Harris detector after rescaling by a factor of 2?

### 6. Point Feature Detectors, Part 2

### 7. Multiple-view geometry

### 8. Multiple-view geometry 2

### 9. Multiple-view geometry 3

### 10. Multiple-view geometry 3 continued

### 11. Optical Flow and Tracking (Lucas-Kanade)

### 12a. Dense 3D Reconstruction

### 12b. Place recognition

### 12c. Deep Learning Tutorial

### 13. Visual inertial fusion

### 14. Event based vision
