---
layout:     post
title:      "VIO - IMU Initialization"
date:       2020-10-26
author:     Tong
catalog: true
tags:
    - VIO
---

### Goal

- The goal of this step is to obtain good initial values for the inertial variables $$\mathcal{X}_{I}=\left[\mathbf{v}_{b_{0}}^{b_{0}}, \mathbf{v}_{b_{1}}^{b_{1}}, \ldots \mathbf{v}_{b_{m}}^{b_{n}}, \mathbf{g}^{c_{0}}, s\right]$$:
  1. body velocties (3x1)
     - number of velocties = number of image frames
     - we integrate IMU measurements between two image frames.
  2. gravity (3x1)
     - usually given magnitude such as 9.81.
     - only 2 DoF for its direction.
     - usually need to be aligned to world z-axis at the end.
  3. accelerometer bias (3x1)
     - sometimes assumed to be zero, because it is quite small compared to gravity and hard to estimate. Besides, it has only small influence on initialization.
  4. gyroscope bias (3x1)
     - should be estimated

### [VINS Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)[^Qin17]

#### Idea

The basic idea is to match the up-to-scale visual structure with IMU preintegration.

#### Assumption

1. $$q_{bc}$$ is already known (calibrated offline or online).
2. $$t_{bc}$$ is assumed to be zero.

#### A. Vision-Only SfM in Sliding Window

1. Find correspondences through KLT tracker using Shi-Tomasi corners.
   - Repeat this step until we have tracked enough images (e.g. 10).
2. Choose an old frame as the temporal world origin (let's say camera 0)
   - Camera 0 system will be aligned with world system by rotating gravity to z-axis in world later.
   - Camera 0 should have enough matches with the latest image frame, and enough average parallax (i.e., moving distance of point on normalized plane)
   - We will compute the relative pose between camera 0 and the latest frame by computing an essential matrix and decompose it. This relative pose will be used in next step to initialize the absolute poses of camera 0 and latest frame. 
   - Moreover, these absolute poses will serve as an initial guess for solving PnP.
3. Global SfM
   1. Initialize 3D points' positions by triangulation of any two frames. 
      - If the relative pose is unknown, we will use `cv::solvePnP` to compute it (the intial guesses of camera poses are given by the frames' neighbors). 
   2. Bundle adjustment to update all $$T_{wc}$$ (world is now camera 0). Note that we do not update points' positions here.
4. Update camera poses by solving PnP again using optimized camera poses

#### B. Visual-Inertial Alignment

1. Gyroscope Bias Calibration
   - Using absolute rotation of two frames and preintegration between them, we would like to get $$\mathbf{q}_{b_{k+1}}^{c_{0}} \otimes \mathbf{q}_{b_{k}}^{c_{0}} \otimes \gamma_{b_{k+1}}^{b_{k}}=\begin{bmatrix} 1 \\ \mathbf{0} \end{bmatrix}$$
   - The preintegration is perturbed by an increment of gyroscope bias as $$\gamma_{b_{k+1}}^{b_{k}} \approx \hat{\gamma}_{b_{k+1}}^{b_{k}} \otimes\left[\begin{array}{c}1 \\\frac{1}{2} \mathbf{J}_{b_{w}}^{\gamma} \delta \mathbf{b}_{w}\end{array}\right]$$
   - Thus, we have 
     - $$\mathbf{q}^{b_{k+1}}_{c_{0}} \otimes \mathbf{q}_{b_{k}}^{c_{0}} \otimes \hat{\gamma}_{b_{k+1}}^{b_{k}} \otimes\left[\begin{array}{c}1 \\\frac{1}{2} \mathbf{J}_{b_{w}}^{\gamma} \delta \mathbf{b}_{w}\end{array}\right]=\begin{bmatrix} 1 \\ \mathbf{0} \end{bmatrix}$$
     - $$\mathbf{J}_{b_{w}}^{\gamma} \delta \mathbf{b}_{w} = 2 \cdot \hat{\gamma}_{b_{k}}^{b_{k+1}} \otimes \mathbf{q}^{b_{k}}_{c_{0}} \otimes \mathbf{q}_{b_{k+1}}^{c_{0}}$$ 
     - $$\mathbf{J}\mathbf{x}=\mathbf{r} \rightarrow \mathbf{J}^{T}\mathbf{J}\mathbf{x}=\mathbf{J}^{T}\mathbf{r} $$
   - Therefore, we just need to stack all information between two consecutive frames, and solve for the increment of gyroscope bias. Take carefully the indices of the frames in the Hessian matrix.
   - The new gyroscope bias equals to the orignal one plus the computed increment.
   - After update of gyroscope bias, we need to repropogate using the new gyroscope bias.

2. Velocity, Gravity Vector, and Metric Scale Initialization
   - We need to find the relationships between preintegration (positions and velocities) and variables to find
     - $$\left[\begin{array}{c}\hat{\alpha}_{b_{k+1}}^{b_{k}}-\mathbf{p}_{c}^{b}+\mathbf{R}_{c_{0}}^{b_{k}} \mathbf{R}_{b_{k+1}}^{c_{0}} \mathbf{p}_{c}^{b} \\ \hat{\beta}_{b_{k+1}}^{b_{k}}\end{array}\right]=\mathbf{H}_{b_{k+1}}^{b_{k}} \mathcal{X}_{I}$$
     - $$\mathbf{H}_{b_{k+1}}^{b_{k}}=\left[\begin{array}{ccc}-\mathbf{I} \Delta t_{k} & \mathbf{0} & \frac{1}{2} \mathbf{R}_{c_{0}}^{b_{k}} \Delta t_{k}^{2}  & \mathbf{R}_{c_{0}}^{b_{k}}\left(\overline{\mathbf{p}}_{c_{k+1}}^{c_{0}}-\overline{\mathbf{p}}_{c_{k}}^{c_{0}}\right) \\ -\mathbf{I} & \mathbf{R}_{c_{0}}^{b_{k}} \mathbf{R}_{b_{k+1}}^{c_{0}} & \mathbf{R}_{c_{0}}^{b_{k}} \Delta t_{k} & \mathbf{0}\end{array}\right]$$
     - $$\mathbf{J}\mathbf{x}=\mathbf{r} \rightarrow \mathbf{J}^{T}\mathbf{J}\mathbf{x}=\mathbf{J}^{T}\mathbf{r} $$
   - The goal of this step is only to obtain an initial guess of gravity.
3. Gravity Refinement
   - Because gravity has only 2 DoF (its magnitude is assumed to be known), we perturb it using 
     - $$\hat{\mathrm{g}}^{c_{0}}=\|g\| \cdot \hat{\overline{\mathrm{g}}}^{c_{0}}+w_{1} \vec{b}_{1}+w_{2} \vec{b}_{2}$$
   - The problem is converted into solving $$\begin{bmatrix}w_1 \\ w_2 \end{bmatrix}$$
   - $$\vec{b}_{1}$$ and $$\vec{b}_{2}$$ are computed using the last guess of gravity's direction.
   - The refinement is performed just like the previous step. The only difference compared to it is that our gravity's DoF here is 2, but in the previous step, it is 3.
4. Completing Initialzation
   1. Estimate features' depths (up to scale)
   2. Repropagation using new gyroscope bias
   3. Scale position change and velocity change wrt. camera 0
   4. Scale features' depth
   5. Align gravity to the direction $$\begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}$$
   6. Convert position, velocity, and rotation change to the world system

#### Discussion

- Why don't we estimate accelerometer bias?
   - Usually small value compared to gravity's magnitude --> Hard to estimate.
   - We think that it has only small influence.
- Failure cases
  - Pure rotation: Not able to do SfM.

### [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) [^Campos20]

#### Idea

- Consider the IMU initialization as a MAP estimation problem.

#### A. Vision-Only MAP Estimation

- Visual SLAM only. Recover a up-to-scale map composed of 10 camera poses and hundreds of points.

#### B. Inertial-Only MAP Estimation

- Align IMU trajectory and ORB-SLAM trajectory. Solve in one single step for all inertial parameters. 

#### C. Visual-Inertial MAP Estimation

- Joint visula-inertial optimization.
- Common bias for all keyframes.
- Same prior information.

#### Discussion

- We do not assume IMU biases to be zero.

### Literature

[^Qin17]: Qin, Tong, and Shaojie Shen. "Robust initialization of monocular visual-inertial estimation on aerial robots." 2017 IROS.

[^Campos20]: C. Campos, J. M. M. Montiel and J. D. Tardós, "Inertial-Only Optimization for Visual-Inertial Initialization," 2020 ICRA.