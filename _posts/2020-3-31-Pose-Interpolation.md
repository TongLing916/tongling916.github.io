---
layout:     post
title:      "Pose Interpolation"
date:       2020-3-31
author:     Tong
catalog: true
tags:
    - Technique
---

> [Eigen::slerp()](https://eigen.tuxfamily.org/dox/classEigen_1_1QuaternionBase.html#ac840bde67d22f2deca330561c65d144e)

### [Survey of Higher Order Rigid Body Motion Interpolation Methods for Keyframe Animation and Continuous-Time Trajectory Estimation](https://github.com/adrelino/interpolation-methods/) [^Haarbach18]

#### Abstract 

In this survey, we carefully analyze the characteristics of higher order rigid body motion interpolation methods to obtain a continuous trajectory from a descrete set of poses. We first discuss the tradeoff between continuity, local control and approximation of classical Euclidean interpolation schemes such Bezier and B-splines. The benefits of the manifold of unit quaternions $$\mathbb{SU}\left ( 2 \right )$$, a double-cover of rotation matrices $$\mathbb{SO}\left ( 3 \right )$$, as rotation parameterization are presented, which allow for an elegant formulation of higher order orientation interpolation with easy analytic derivatives, made possible through the Lie Algebra $$su(2)$$ of pure quaternions and the cumulative form of cubic B-splines. The same construction scheme is then applied for joint interpolation in the full rigid body pose space, which had previously been done for the matrix representation $$\mathbb{SE}\left ( 3 \right )$$ and its twists, but not for the more efficient unit dual quaternion $$\mathbb{DH}_{1}$$ and its screw motions. Both suffer from the effects of coupling translation and rotation that have mostly been ignored by previous work. We thus conclude that split interpolation [^Kim95] in $$\mathbb{R}^{3} \times \mathbb{S U}(2)$$ is preferable for most applications. Our final runtime experiments show that joint interpolation in $$\mathbb{SE}\left ( 3 \right )$$ is 2 times and in $$\mathbb{DH}_{1}$$ 13 times slower - which furthermore justifies our suggestion from a practical point of view. 

#### Introduction

In this paper we compare many existing methods for camera trajectory interpolation providing an intuitive visual and experimental comparison among them and highlighting insights that are not obvious. In the end we furthermore briefly introduce two novel formulations (ScFus, DLFus) of higher order dual quaternion interpolation as efficient alternatives to the usual joint interpolation with $$\mathbb{SE}\left ( 3 \right )$$ matrices.


#### Related Literature

- Orientation interpolation methods, e.g. SLERP (Spherical linear interpolation)[^Shoemake85], SQUAD (Spherical cubic spline quadrangle)[^Shoemake87], CuBsp (Cumulative B-spline curve)[^Kim95].
- Rigid body motion interpolation methods, SpFus (Spline Fusion twist curve)[^Lovegrove13] [^Perez15].
- Continuous-time estimation theory [^Sommer15]
- Dual quaternion approaches [^Busam17] [^Busam16]

#### Euclidean interpolation methods

- A $$C^{2}$$ continuous composite cubic Bezier curve __interpolates__ the control points but has __global control__.
- A $$C^{2}$$ continuous cubic B-spline has __local control__ but only __approximates__ the control points.

#### Orientation interpolation methods

#### Rigid body motion interpolation methods

- Dual quaternions [^Kavan06] [^Kenwright12]

### Camera Pose Filtering with Local Regression Geodesics on the Riemannian Manifold of Dual Quaternions [^Busam17]

#### Abstract 

Time-varying, smooth trajectory estimation is of great interest to the vision community for accurate and well behaving 3D systems. In this paper, we propose a novel principal component local regression filter acting directly on the Riemannian manifold of unit dual quaternions DH1. We use a numerically stable Lie algebra of the dual quaternions together with exp and log operators to locally linearize the 6D pose space. Unlike state of the art path smoothing methods which either operate on $$\mathbb{SO}\left ( 3 \right )$$ of rotation matrices or the hypersphere H1 of quaternions, we treat the orientation and translation jointly on the dual quaternion quadric in the 7-dimensional real projective space RP7. We provide an outlier-robust IRLS algorithm for generic pose filtering exploiting this manifold structure. Besides our theoretical analysis, our experiments on synthetic and real data show the practical advantages of the manifold aware filtering on pose tracking and smoothing.

### A General Construction Scheme for Unit Quaternion Curves with Simple High Order Derivatives [^Kim95]

#### Abstract 

This paper proposes a new class of unit quaternion curves in $$\mathbb{SO}\left ( 3 \right )$$. A general method is developed that transforms a curve in $$R^3$$ (defined as a weighted sum of basis functions) into its unit quaternion analogue in $$\mathbb{SO}\left ( 3 \right )$$. Applying the method to well-known spline curves (such as Bezier, Hermite, and B-spline curves), we are able to construct various unit quaternion curves which share many important differential properties with their original curves.

Many of our naive common beliefs in geometry break down even in the simple non-Euclidean space $$S^3$$ or $$\mathbb{SO}\left ( 3 \right )$$. For example, the de Casteljau type construction of cubic B-spline quaternion curves does not preserve $$C^2$$-continuity[^Kim95b]. Through the use of decomposition into simple primitive quaternion curves, our quaternion curves preserves most of the algebraic and differential properties of the original spline curves.

#### Introduction

- __Split interpolation (SPLIT)__ in $$\mathbb{R}^{3} \times \mathbb{S U}(2)$$ separately interpolates orientation and position by applying SLERP (Spherical linear interpolation)[^Shoemake85] on $$\mathrm{q}_{0}$$, $$\mathrm{q}_{1}$$ and LERP (linear interpolation) on $$\mathrm{t}_{0}$$, $$\mathrm{t}_{0}$$ along $$u \in[0,1]$$:

$$
\begin{array}{l}
\operatorname{SPLIT}\left(\left(\mathbf{q}_{0}, \mathbf{t}_{0}\right),\left(\mathbf{q}_{1}, \mathbf{t}_{1}\right), u\right) \\
\quad=\left(\operatorname{SLERP}\left(\mathbf{q}_{0}, \mathbf{q}_{1}, u\right), \operatorname{LERP}\left(\mathbf{t}_{0}, \mathbf{t}_{1}, u\right)\right) \\
\quad=\left(\mathbf{q}_{0} \exp \left(u \log \left(\overline{\mathbf{q}}_{0} \mathbf{q}_{1}\right)\right), \mathbf{t}_{0}+u\left(\mathbf{t}_{1}-\mathbf{t}_{0}\right)\right)
\end{array}
$$


### Animating rotation with quaternion curves [^Shoemake85]

#### Abstract 

Solid bodies roll and tumble through space. In computer animation, so do cameras. The rotations of these objects are best described using a four coordinate system, quaternions, as is shown in this paper. Of all quaternions, those on the unit sphere are most suitable for animation, but the question of how to construct curves on spheres has not been much explored. This paper gives on answer by presenting a new kind of spline curve, created on a sphere, suitable for smoothly in-betweening (i.e. interpolating) sequences of arbitrary rotations. Both theory and experiment show that the motion generated is smooth and natural, without quirks found in earlier methods.

### Literature

[^Haarbach18]: Haarbach, Adrian, Tolga Birdal, and Slobodan Ilic. "Survey of higher order rigid body motion interpolation methods for keyframe animation and continuous-time trajectory estimation." 2018 International Conference on 3D Vision (3DV). IEEE, 2018.

[^Shoemake85]: Shoemake, Ken. "Animating rotation with quaternion curves." Proceedings of the 12th annual conference on Computer graphics and interactive techniques. 1985.

[^Shoemake87]: K. Shoemake. Quaternion calculus and fast animation. In ACM SIGGRAPH Course Notes 10, Computer Animation: 3-D motion specification and control, number 10, pages 101–121. Siggraph, 1987.

[^Busam17]: Busam, Benjamin, Tolga Birdal, and Nassir Navab. "Camera pose filtering with local regression geodesics on the riemannian manifold of dual quaternions." Proceedings of the IEEE International Conference on Computer Vision Workshops. 2017.

[^Busam16]: B. Busam, M. Esposito, B. Frisch, and N. Navab. Quaternionic upsampling: Hyperspherical techniques for 6 dof pose tracking. In 3D Vision (3DV), 2016 Fourth International Conference on, pages 629–638. IEEE, 2016. 

[^Kim95]: M.-J. Kim, M.-S. Kim, and S. Y. Shin. A general construction scheme for unit quaternion curves with simple high order derivatives. In Proceedings of the 22nd annual conference on Computer graphics and interactive techniques, pages 369–376. ACM, 1995. 

[^Kim95b]: Kim, M.-J., KIM, M.-S., AND SHIN, S. A C2-continuous Bspline quaternion curve interpolating a given sequence of solid rientations. Proc. of Computer Animation ’95 (1995), pp. 72–81.

[^Lovegrove13]: S. Lovegrove, A. Patron-Perez, and G. Sibley. Spline fusion: A continuous-time representation for visual-inertial fusion with application to rolling shutter cameras. In BMVC, 2013.

[^Perez15]: A. Patron-Perez, S. Lovegrove, and G. Sibley. A spline-based trajectory representation for sensor fusion and rolling shutter cameras. International Journal of Computer Vision, 113(3):208–219, 2015. 

[^Sommer15]: H. Sommer, J. R. Forbes, R. Siegwart, and P. Furgale. Continuous-time estimation of attitude using b-splines on lie groups. Journal of Guidance, Control, and Dynamics, 39(2):242–261, 2015.

[^Kenwright12]: B. Kenwright. A beginners guide to dual-quaternions: what they are, how they work, and how to use them for 3d character hierarchies. 2012.

[^Kavan06]: L. Kavan, S. Collins, C. O’Sullivan, and J. Zara. Dual quaternions for rigid transformation blending. Trinity College Dublin, Tech. Rep. TCD-CS-2006-46, 2006. 