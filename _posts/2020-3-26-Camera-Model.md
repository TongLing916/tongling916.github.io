---
layout:     post
title:      "Camera Model"
date:       2020-3-26
author:     Tong
catalog: true
tags:
    - Technique
---

### [The Double Sphere Camera Model](https://vision.in.tum.de/research/vslam/double-sphere) [^Usenko18]

> https://gitlab.com/VladyslavUsenko/basalt

> https://github.com/VladyslavUsenko/basalt-mirror

#### Abstract

Vision-based motion estimation and 3D reconstruction, which have numerous applications (e.g., autonomous driving, navigation systems for airborne devices and augmented reality) are receiving significant research attention. To increase the accuracy and robustness, several researchers have recently demonstrated the benefit of using large fieldof-view cameras for such applications.

In this paper, we provide an extensive review of existing models for large field-of-view cameras. For each model we provide projection and unprojection functions and the subspace of points that result in valid projection. Then, we propose the Double Sphere camera model that well fits with large field-of-view lenses, is computationally inexpensive and has a closed-form inverse. We evaluate the model using a calibration dataset with several different lenses and compare the models using the metrics that are relevant for Visual Odometry, i.e., reprojection error, as well as computation time for projection and unprojection functions and their Jacobians. We also provide qualitative results and discuss the performance of all models.

#### Introduction 

- We propose a novel projection model for fisheye cameras that has the following advantages. 
    - The proposed projection model is well suited to represent the distortions of fisheye lenses.
    - The proposed model does not require computationally expensive trigonometric operations for projection and unprojection.
    - Differing from projection models based on higher order polynomials [^Kannala06] [^Scaramuzza06], that use iterative methods to unproject points, the inverse of the projection function exists in a closed form.

#### Assumption

- For all camera models we assume all projections cross a single point (i.e., central projection) that defines the position of the camera coordinate frame. The orientation of the camera frame is defined as follows. The z axis is aligned with the principal axis of the camera, and two other orthogonal directions (x, y) align with the corresponding axes of the image plane.

#### Pinhole Camera Model

- Projection function
$$\pi(\mathbf{x}, \mathbf{i})=\left[\begin{array}{l}
f_{x} \frac{x}{z} \\
f_{y} \frac{1}{z}
\end{array}\right]+\left[\begin{array}{l}
c_{x} \\
c_{y}
\end{array}\right]$$

$$\begin{aligned}
\pi^{-1}(\mathbf{u}, \mathbf{i}) &=\frac{1}{\sqrt{m_{x}^{2}+m_{y}^{2}+1}}\left[\begin{array}{l}
m_{x} \\
m_{y} \\
1
\end{array}\right] \\
m_{x} &=\frac{u-c_{x}}{f_{x}} \\
m_{y} &=\frac{v-c_{y}}{f_{y}}
\end{aligned}$$

#### Unified Camera Model [^Geyer00] [^Ying04] [^Mei07]

- Projection function
$$\pi(\mathbf{x}, \mathbf{i})=\left[\begin{array}{l}
f_{x} \frac{x}{\alpha d+(1-\alpha) z} \\
f_{y} \frac{y}{\alpha d+(1-\alpha) z}
\end{array}\right]+\left[\begin{array}{l}
c_{x} \\
c_{y}
\end{array}\right]$$

- The set of 3D points that result in valid projections is defined as follows
$$\begin{array}{l}
\Omega=\left\{\mathbf{x} \in \mathbb{R}^{3} | z>-w d\right\} \\
w=\left\{\begin{array}{ll}
\frac{\alpha}{1-\alpha}, & \text { if } \alpha \leq 0.5 \\
\frac{1-\alpha}{\alpha} & \text { if } \alpha>0.5
\end{array}\right.
\end{array}$$

- Unprojection function
$$\begin{aligned}
\pi^{-1}(\mathbf{u}, \mathbf{i}) &=\frac{\xi+\sqrt{1+\left(1-\xi^{2}\right) r^{2}}}{1+r^{2}}\left[\begin{array}{l}
m_{x} \\
m_{y} \\
1
\end{array}\right]-\left[\begin{array}{l}
0 \\
0 \\
\xi
\end{array}\right] \\
m_{x} &=\frac{u-c_{x}}{f_{x}}(1-\alpha) \\
m_{y} &=\frac{v-c_{y}}{f_{y}}(1-\alpha) \\
r^{2} &=m_{x}^{2}+m_{y}^{2} \\
\xi &=\frac{\alpha}{1-\alpha}
\end{aligned}$$, where $$\Theta$$ is defined as follows
$$\Theta=\left\{\begin{array}{ll}
\mathbb{R}^{2} & \text { if } \alpha \leq 0.5 \\
\left\{\mathbf{u} \in \mathbb{R}^{2} | r^{2} \leq \frac{(1-\alpha)^{2}}{2 \alpha-1}\right\} & \text { if } \alpha>0.5
\end{array}\right.$$

#### Extended Unified Camera Model [^Khomutenko15]

The EUCM can be interpreted as a generalization of the UCM where the point is projected onto an ellipsoid symmetric around the z axis. That study also indicated that when treating the model as a projection on a quadratic surface followed by orthographic projection on the image plane the model is complete in the sense that it can represent all possible quadratic surfaces.

- Projection function
$$\begin{aligned}
\pi(\mathbf{x}, \mathbf{i}) &=\left[\begin{array}{l}
f_{x} \frac{x}{\alpha d+(1-\alpha) z} \\
f_{y} \frac{y}{\alpha d+(1-\alpha) z}
\end{array}\right]+\left[\begin{array}{c}
c_{x} \\
c_{y}
\end{array}\right] \\
d &=\sqrt{\beta\left(x^{2}+y^{2}\right)+z^{2}}
\end{aligned}$$

- Unprojection function
$$\begin{aligned}
\pi^{-1}(\mathbf{u}, \mathbf{i}) &=\frac{1}{\sqrt{m_{x}^{2}+m_{y}^{2}+m_{z}^{2}}}\left[\begin{array}{l}
m_{x} \\
m_{y} \\
m_{z}
\end{array}\right] \\
m_{x} &=\frac{u-c_{x}}{f_{x}} \\
m_{y} &=\frac{v-c_{y}}{f_{y}} \\
r^{2} &=m_{x}^{2}+m_{y}^{2} \\
m_{z} &=\frac{1-\beta \alpha^{2} r^{2}}{\alpha \sqrt{1-(2 \alpha-1) \beta r^{2}}+(1-\alpha)}
\end{aligned}$$, where $$\Theta$$ is defined as follows
$$\Theta=\left\{\begin{array}{ll}
\mathbb{R}^{2} & \text { if } \alpha \leq 0.5 \\
\left\{\mathbf{u} \in \mathbb{R}^{2} | r^{2} \leq \frac{1}{\beta(2 \alpha-1)}\right\} & \text { if } \alpha>0.5
\end{array}\right.$$

#### Kannala-Brandt Camera Model [^Kannala06]

- Projection function
$$\begin{aligned}
\pi(\mathbf{x}, \mathbf{i}) &=\left[\begin{array}{l}
f_{x} d(\theta) \\
f_{y} d(\theta)
\end{array} \frac{x}{r}\right]+\left[\begin{array}{c}
c_{x} \\
c_{y}
\end{array}\right] \\
r &=\sqrt{x^{2}+y^{2}} \\
\theta &=\operatorname{atan} 2(r, z) \\
d(\theta) &=\theta+k_{1} \theta^{3}+k_{2} \theta^{5}+k_{3} \theta^{7}+k_{4} \theta^{9}
\end{aligned}$$

- Unprojection function
$$\begin{aligned}
\pi^{-1}(\mathbf{u}, \mathbf{i}) &=\left[\begin{array}{c}
\sin \left(\theta^{*}\right) \frac{m_{x}}{r_{y}} \\
\sin \left(\theta^{*}\right) \\
\cos \left(\theta^{*}\right)^{\frac{\pi_{y}}{u}}
\end{array}\right] \\
m_{x} &=\frac{u-c_{x}}{f_{x}} \\
m_{y} &=\frac{v-c_{y}}{f_{y}} \\
r_{u} &=\sqrt{m_{x}^{2}+m_{y}^{2}} \\
\theta^{*} &=d^{-1}\left(r_{u}\right)
\end{aligned}$$

#### Field-of-View Camera Model [^Devernay01]

- Projection function
$$\begin{aligned}
\pi(\mathbf{x}, \mathbf{i}) &=\left[\begin{array}{l}
f_{x} r_{d} \frac{x}{r_{y}} \\
f_{y} r_{d} \frac{y}{r_{u}}
\end{array}\right]+\left[\begin{array}{c}
c_{x} \\
c_{y}
\end{array}\right] \\
r_{u} &=\sqrt{x^{2}+y^{2}} \\
r_{d} &=\frac{\operatorname{atan} 2\left(2 r_{u} \tan \frac{w}{2}, z\right)}{w}
\end{aligned}$$

- Unprojection function
$$\begin{aligned}
\pi^{-1}(\mathbf{u}, \mathbf{i}) &=\left[\begin{array}{l}
m_{x} \frac{\sin \left(r_{d} w\right)}{2_{d_{d}} \tan \frac{w}{2}} \\
m_{y} \frac{\sin \left(r_{g} w\right)}{2 \pi_{d} \tan \frac{w}{2}} \\
\cos \left(r_{d} w\right)
\end{array}\right] \\
m_{x} &=\frac{u-c_{x}}{f_{x}} \\
m_{y} &=\frac{v-c_{y}}{f_{y}} \\
r_{d} &=\sqrt{m_{x}^{2}+m_{y}^{2}}
\end{aligned}$$

#### Double Sphere Camera Model

- Projection function
$$\begin{aligned}
\pi(\mathbf{x}, \mathbf{i}) &=\left[\begin{array}{l}
f_{x} \frac{x}{\alpha d_{2}+(1-\alpha)\left(\xi d_{1}+z\right)} \\
f_{y} \frac{y}{\alpha d_{2}+(1-\alpha)\left(\xi d_{1}+z\right)}
\end{array}\right]+\left[\begin{array}{c}
c_{x} \\
c_{y}
\end{array}\right] \\
d_{1} &=\sqrt{x^{2}+y^{2}+z^{2}} \\
d_{2} &=\sqrt{x^{2}+y^{2}+\left(\xi d_{1}+z\right)^{2}}
\end{aligned}$$

- A set of 3D points that results in valid projection is expressed as follows:
$$\begin{array}{l}
\Omega=\left\{\mathbf{x} \in \mathbb{R}^{3} | z>-w_{2} d_{1}\right\} \\
w_{2}=\frac{w_{1}+\xi}{\sqrt{2 w_{1}} \xi+\xi^{2}+1} \\
w_{1}=\left\{\begin{array}{ll}
\frac{\alpha}{1-\alpha}, & \text { if } \alpha \leq 0.5 \\
\frac{1-\alpha}{\alpha} & \text { if } \alpha>0.5
\end{array}\right.
\end{array}$$

- Unprojection function
$$\begin{aligned}
\pi^{-1}(\mathbf{u}, \mathbf{i}) &=\frac{m_{z} \xi+\sqrt{m_{z}^{2}+\left(1-\xi^{2}\right) r^{2}}}{m_{z}^{2}+r^{2}}\left[\begin{array}{l}
m_{x} \\
m_{y} \\
m_{z}
\end{array}\right]-\left[\begin{array}{l}
0 \\
0 \\
\xi
\end{array}\right] \\
m_{x} &=\frac{u-c_{x}}{f_{x}} \\
m_{y} &=\frac{v-c_{y}}{f_{y}} \\
r^{2} &=m_{x}^{2}+m_{y}^{2} \\
m_{z} &=\frac{1-\alpha^{2} r^{2}}{\alpha \sqrt{1-(2 \alpha-1) r^{2}}+1-\alpha}
\end{aligned}$$, where the following holds
$$\Theta=\left\{\begin{array}{ll}
\mathbb{R}^{2} & \text { if } \alpha \leq 0.5 \\
\left\{\mathbf{u} \in \mathbb{R}^{2} | r^{2} \leq \frac{1}{2 \alpha-1}\right\} & \text { if } \alpha>0.5
\end{array}\right.$$

### Literature

[^Usenko18]: Usenko, Vladyslav, Nikolaus Demmel, and Daniel Cremers. "The double sphere camera model." 2018 International Conference on 3D Vision (3DV). IEEE, 2018.

[^Khomutenko15]: Khomutenko, Bogdan, Gaëtan Garcia, and Philippe Martinet. "An enhanced unified camera model." IEEE Robotics and Automation Letters 1.1 (2015): 137-144.

[^Kannala06]: J. Kannala and S. S. Brandt. A generic camera model and calibration method for conventional, wide-angle, and fish-eye lenses. IEEE transactions on pattern analysis and machine intelligence, 28(8):1335–1340, 2006.

[^Scaramuzza06]: D. Scaramuzza, A. Martinelli, and R. Siegwart. A flexible technique for accurate omnidirectional camera calibration and structure from motion. In Fourth IEEE International Conference on Computer Vision Systems (ICVS’06), pages 45–45, Jan 2006.

[^Devernay01]: F. Devernay and O. Faugeras. Straight lines have to be straight. Machine vision and applications, 13(1):14–24, 2001.

[^Geyer00]: C. Geyer and K. Daniilidis. A unifying theory for central panoramic systems and practical implications. In D. Vernon, editor, Computer Vision — ECCV 2000, pages 445–461, Berlin, Heidelberg, 2000. Springer Berlin Heidelberg.

[^Ying04]: X. Ying and Z. Hu. Can we consider central catadioptric cameras and fisheye cameras within a unified imaging model. In T. Pajdla and J. Matas, editors, Computer Vision - ECCV 2004, pages 442–455, Berlin, Heidelberg, 2004. Springer Berlin Heidelberg.

[^Mei07]: C. Mei and P. Rives. Single view point omnidirectional camera calibration from planar grids. In Proceedings 2007 IEEE International Conference on Robotics and Automation, pages 3945–3950, April 2007.

[^Olson11]: E. Olson. AprilTag: A robust and flexible visual fiducial system. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pages 3400–3407. IEEE, May 2011.