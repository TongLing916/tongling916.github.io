---
layout:     post
title:      "DSO - Null Space"
date:       2020-4-24
author:     Tong
catalog: true
tags:
    - DSO
---

> [Direct sparse odometry](https://vision.in.tum.de/research/vslam/dso)

> [(Chinese) DSO(5)——零空间的计算与推导](https://blog.csdn.net/wubaobao1993/article/details/105106301)

> [(Chinese) DSO零空间与尺度漂移](https://blog.csdn.net/xxxlinttp/article/details/100080080)


### What is Null Space? 

A null space is usually defined in combination with a matrix, namely 

$$
\mathbf{H} \mathbf{x} = \mathbf{0}
$$

Then, we can say $$\mathbf{x}$$ lies in the __(right) null space__ of $$\mathbf{H}$$.

### Why Null Space?

Usually when we use Gauss-Newton method, we need to solve the following equation

$$
\mathbf{J}^{T} \mathbf{J} \mathbf{x} = -\mathbf{J}^{T} \mathbf{e}
$$

If $$\mathbf{J}^{T} \mathbf{J}$$ is invertible, we obtain the __only__ one solution 

$$
\mathbf{x} = -(\mathbf{J}^{T} \mathbf{J})^{-1} \mathbf{J}^{T} \mathbf{e}
$$, which means we project $$\mathbf{e}$$ into the column space of $$\mathbf{J}$$. (Reference: [MIT Linear Algebra](https://ocw.mit.edu/courses/mathematics/18-06sc-linear-algebra-fall-2011/least-squares-determinants-and-eigenvalues/projections-onto-subspaces/))

However, if $$\mathbf{J}^{T} \mathbf{J}$$ is __not__ invertible, i.e., there are some free variables to choose so that 

$$
\mathbf{J}^{T} \mathbf{J} \Delta\mathbf{x} = 0
$$

Therefore, if we find one solution $$\mathbf{x}$$ fulfilling $$\mathbf{J}^{T} \mathbf{J} \mathbf{x} = -\mathbf{J}^{T} \mathbf{e}$$, then $$\mathbf{x} + \lambda\Delta\mathbf{x}$$ is also a solution, i.e., we have infinite solutions for the problem $$\mathbf{J}^{T} \mathbf{J} \mathbf{x} = -\mathbf{J}^{T} \mathbf{e}$$. 

### What is Null Space in DSO?

DSO, as a __monocular__ visual odometry system, suffers __scale ambiguity__. That is, if we scale the whole trajecotry and points' positions, it does __not__ affect the optimization (e.g. Gauss-Newton, Levenberg-Marquadt).

What's more, like any other SLAM system, there is also a coordinate system ambiguity. That is, if we multiply all camera poses with a $$\text{SE}(3)$$ (rigid-body transformation), it does not make any difference in the optimization. The result of SLAM / VO will still be perfect, if it was. :)

The above 7 degrees of freedom are common in all monocular system. If they change for all poses and points, the result of the optimization in the sytem remain the same. Specifically, DSO introduces two new variables - photometric parameters. If these two change for all frames, our results will also remain the same.

These DoF can be considered __null space__ for the optimization in SLAM / VO.

With the description above, it seems impossible to solve the optimization in SLAM, because there are infinite solutions to the optimization. To overcome this problem, several things can be done. 
- In ORB-SLAM2, we need to __fix__ some poses every time when we want to perform optimization (local BA or global BA or pose graph optimization).
- In DSO, we stronly rely on the first estimate of poses, then when we perform optimization, 
  - we only optimize the poses and points in a small region. 
  - we remove the influence of the null space.

### How does DSO Use Null Space?

How can we remove the influence of null space? In other words, how can we remove the influence of null space in the increment $$\mathbf{x}$$ obtained by solving an optimization?

The idea is quite simple, we only remain the part of $$\mathbf{x}$$ which are orthogonal to the null space. Or, we can substract the projection of $$\mathbf{x}$$ in the null space.

How can we compute the projection matrix onto the null space?

To answer this question, we need to firstly understand how those DoF affect the increment. The question actaully becomes computation of derivatives of poses and a, b wrt. free variables. This could be done in a [numerical way](http://www.ceres-solver.org/numerical_derivatives.html#central-differences)

$$
\frac{\partial f}{\partial x} \approx \frac{f(x + h) - f(x - h)}{2h}
$$

In the implementation of DSO, the derivatives are computed as follows

```c++
void FrameHessian::setStateZero(const Vec10& state_zero) {
  this->state_zero = state_zero;

  for (int i = 0; i < 6; ++i) {
    Vec6 eps;
    eps.setZero();
    eps[i] = 1e-3;
    SE3 EepsP = Sophus::SE3::exp(eps);
    SE3 EepsM = Sophus::SE3::exp(-eps);
    SE3 w2c_leftEps_P_x0 =
        (get_worldToCam_evalPT() * EepsP) * get_worldToCam_evalPT().inverse();
    SE3 w2c_leftEps_M_x0 =
        (get_worldToCam_evalPT() * EepsM) * get_worldToCam_evalPT().inverse();
    nullspaces_pose.col(i) =
        (w2c_leftEps_P_x0.log() - w2c_leftEps_M_x0.log()) / (2e-3);
  }

  // scale change
  SE3 w2c_leftEps_P_x0 = (get_worldToCam_evalPT());
  w2c_leftEps_P_x0.translation() *= 1.00001;
  w2c_leftEps_P_x0 = w2c_leftEps_P_x0 * get_worldToCam_evalPT().inverse();
  SE3 w2c_leftEps_M_x0 = (get_worldToCam_evalPT());
  w2c_leftEps_M_x0.translation() /= 1.00001;
  w2c_leftEps_M_x0 = w2c_leftEps_M_x0 * get_worldToCam_evalPT().inverse();
  nullspaces_scale = (w2c_leftEps_P_x0.log() - w2c_leftEps_M_x0.log()) / (2e-3); // possibly 2e-5

  // photometric parameters
  nullspaces_affine.setZero();
  nullspaces_affine.topLeftCorner<2, 1>() = Vec2(1, 0);
  nullspaces_affine.topRightCorner<2, 1>() =
      Vec2(0, expf(aff_g2l_0().a) * ab_exposure);
};
```

Now that we have all derivatives denoted by the Jacobian matrix $$\mathbf{J}$$, what does $$\mathbf{J}$$ mean?

Assume we have computed an increment $$\Delta \mathbf{x}$$ by minimizng our energy function $$\mathbf{E}(\mathbf{x}_{k} + \Delta \mathbf{x})$$ using Gauss-Newton (or LM). Then, we can trust that the $$\Delta\mathbf{x} + \mathbf{J}^{T} \delta{\mathbf{y}}$$ ( where $$\delta{\mathbf{y}}$$ is a perturbation on the free variables pose, scale and photometric parameters) is also a solution to minimize our energy function, i.e., __both solutions lead to the same energy__. 

Let's come back to the original question - How can we compute the projection matrix onto the null space? As we can see, $$\mathbf{J}^{T} \delta{\mathbf{y}}$$ lies in the column space of $$\mathbf{J}^{T}$$. Let's denote $$\mathbf{J}^{T}$$ by $$\mathbf{N}$$.

The projection matrix onto the column space of $$\mathbf{N}$$ is then given by (Reference: [MIT Linear Algebra](https://ocw.mit.edu/courses/mathematics/18-06sc-linear-algebra-fall-2011/least-squares-determinants-and-eigenvalues/projections-onto-subspaces/))


$$
\mathbf{N}(\mathbf{N}^{T}\mathbf{N})^{-1}\mathbf{N}^{T}
$$

if $$\mathbf{N}^{T}\mathbf{N}$$ is invertible,

$$
\mathbf{N}\mathbf{N}^{+}
$$

if $$\mathbf{N}^{T}\mathbf{N}$$ is not invertible,

Note: if $$A=U \Sigma V^{\mathrm{T}}$$ (the SVD), then its pseudoinverse is $$A^{+}=V \Sigma^{+} U^{\mathrm{T}}$$.

Therefore, we can remove the influence of null space by computing 

$$
\mathbf{x} - \mathbf{N} \mathbf{N}^{+}\mathbf{x}
$$

```c++
void EnergyFunctional::orthogonalize(VecX* b, MatXX* H) {
  // Decide to which nullspaces to orthogonalize.
  std::vector<VecX> ns;
  ns.insert(ns.end(), lastNullspaces_pose.begin(), lastNullspaces_pose.end());
  ns.insert(ns.end(), lastNullspaces_scale.begin(), lastNullspaces_scale.end());

  // Make null space matrix
  MatXX N(ns[0].rows(), ns.size());
  for (unsigned int i = 0; i < ns.size(); ++i) {
    N.col(i) = ns[i].normalized();
  }

  // Compute Npi := N * (N' * N)^-1 = pseudo inverse of N.
  Eigen::JacobiSVD<MatXX> svdNN(N, Eigen::ComputeThinU | Eigen::ComputeThinV);

  VecX SNN = svdNN.singularValues();
  double minSv = 1e10, maxSv = 0;
  for (int i = 0; i < SNN.size(); ++i) {
    if (SNN[i] < minSv) {
      minSv = SNN[i];
    }
    if (SNN[i] > maxSv) {
      maxSv = SNN[i];
    }
  }
  for (int i = 0; i < SNN.size(); ++i) {
    if (SNN[i] > setting_solverModeDelta * maxSv) {
      SNN[i] = 1.0 / SNN[i];
    } else {
      SNN[i] = 0;
    }
  }

  MatXX Npi = svdNN.matrixU() * SNN.asDiagonal() *
              svdNN.matrixV().transpose();           // [dim] x 9.
  MatXX NNpiT = N * Npi.transpose();                 // [dim] x [dim].
  MatXX NNpiTS = 0.5 * (NNpiT + NNpiT.transpose());  // = N * (N' * N)^-1 * N'.

  if (b != nullptr) {
    *b -= NNpiTS * *b; // remove the influence of null space
  }
  if (H != nullptr) {
    *H -= NNpiTS * *H * NNpiTS;
  }
}
```