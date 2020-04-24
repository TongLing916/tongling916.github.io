---
layout:     post
title:      "DSO - First Estimates Jacobian"
date:       2020-4-24
author:     Tong
catalog: true
tags:
    - DSO
---

### Analysis and Improvement of the Consistency of Extended Kalman Filter Based SLAM [^Huang08]

#### Abstract

In this work, we study the inconsistency of EKF- based SLAM from the perspective of observability. We analytically prove that when the Jacobians of the state and measurement models are evaluated at the latest state estimates during every time step, the linearized error-state system model of the EKF- based SLAM has observable subspace of dimension higher than that of the actual, nonlinear, SLAM system. As a result, the covariance estimates of the EKF undergo reduction in directions of the state space where no information is available, which is a primary cause of inconsistency. To address this issue, a new "first estimates Jacobian" (FEJ) EKF is proposed, which is shown to perform better in terms of consistency. In the FEJ-EKF, the filter Jacobians are calculated using the first-ever available estimates for each state variable, which insures that the observable subspace of the error-state system model is of the same dimension as that of the underlying nonlinear SLAM system. The theoretical analysis is validated through extensive simulations.

#### Introduction

- The main contributions of this work are the following:
  - Through an observability analysis, we prove that the standard EKF-SLAM employs an error-state system model that has an unobservable subspace of dimension two, even though the underlying nonlinear system model has three unobservable degrees of freedom (corresponding to the position and orientation of the global reference frame). This is a primary cause of filter inconsistency.
  - We propose a new algorithm, termed First Estimates Jacobian (FEJ)-EKF, which improves the estimator’s consistency during SLAM. Specifically, we show analytically that when the EKF Jacobians are computed using the first-ever available estimates for each of the state variables, the error-state model has the _same_ observability properties as the underlying nonlinear model. As a result of these properties, the new FEJ-EKF outperforms, in terms of accuracy and consistency, alternative approaches to this problem [^Castellanos07].


### A First-Estimates Jacobian EKF for Improving SLAM Consistency [^Huang09]

#### Abstract 

In this work, we study the inconsistency of EKF-based SLAM from the perspective of observability. We analytically prove that when the Jacobians of the system and measurement models are evaluated at the latest state estimates during every time step, the linearized error-state system employed in the EKF has observable subspace of dimension higher than that of the actual, nonlinear, SLAM system. As a result, the covariance estimates of the EKF undergo reduction in directions of the state space where no information is available, which is a primary cause of the inconsistency. Furthermore, a new “First-Estimates Jacobian” (FEJ) EKF is proposed to improve the estimator’s consistency during SLAM. The proposed algorithm performs better in terms of consistency, because when the filter Jacobians are calculated using the first-ever available estimates for each state variable, the error-state system model has an observable subspace of the same dimension as the underlying nonlinear SLAM system. The theoretical analysis is validated through both simulations and experiments.

### Observability-based Rules for Designing Consistent EKF SLAM Estimators [^Huang10]

#### Abstract 

In this work, we study the inconsistency problem of extended Kalman filter (EKF)-based simultaneous localization and mapping (SLAM) from the perspective of observability. We analytically prove that when the Jacobians of the process and measurement models are evaluated at the latest state estimates during every time step, the linearized error-state system employed in the EKF has an observable subspace of dimension higher than that of the actual, non-linear, SLAM system. As a result, the covariance estimates of the EKF undergo reduction in directions of the state space where no information is available, which is a primary cause of the inconsistency. Based on these theoretical results, we propose a general framework for improving the consistency of EKF-based SLAM. In this framework, the EKF linearization points are selected in a way that ensures that the resulting linearized system model has an observable subspace of appropriate dimension. We describe two algorithms that are instances of this paradigm. In the first, termed observability constrained (OC)-EKF, the linearization points are selected so as to minimize their expected errors (i.e. the difference between the linearization point and the true state) under the observability constraints. In the second, the filter Jacobians are calculated using the first-ever available estimates for all state variables. This latter approach is termed first-estimates Jacobian (FEJ)-EKF. The proposed algorithms have been tested both in simulation and experimentally, and are shown to significantly outperform the standard EKF both in terms of accuracy and consistency.

### Example - Direct Sparse Odometry [^Engel18]

As stated in the paper, 
 - both $$\mathbf{J}_{\text{photo}}=\frac{\partial r_{k}\left((\delta+\boldsymbol{x}) \oplus \zeta_{0}\right)}{\partial \delta_{\text {photo }}}$$ ($$a_i, b_i, a_j, b_j$$) and $$\mathbf{J}_{\text{geo}} = \frac{\partial \mathbf{p}^{\prime}\left((\boldsymbol{\delta}+\boldsymbol{x}) \oplus \zeta_{0}\right)}{\partial \boldsymbol{\delta}_{\mathrm{geo}}}$$ (poses, intrinsic parameters and inverse depths) are evaluated at $$\mathbf{x} = 0$$ (__first estimate__).
 - $$\mathbf{J}_{\text{geo}}$$ is assumed to be the same for all residuals belonging to the same point, and evaluated only for the center pixel.
 -  $$\mathbf{J}_{\text{I}} = \frac{\partial I_{j}}{\partial \mathbf{p}^{\prime}}$$ is evaluated at the __current value__ for $$\mathbf{x}$$, i.e., at the same point as the residual $$r_{k}$$.

The corresponding implementation are

```c++
double PointFrameResidual::linearize(CalibHessian* const HCalib) {
  state_NewEnergyWithOutlier = -1;

  if (state_state == ResState::OOB) {
    state_NewState = ResState::OOB;
    return state_energy;
  }

  FrameFramePrecalc* precalc = &(host->targetPrecalc[target->idx]);
  const Eigen::Vector3f* dIl = target->dI;  // [intensity gx gy]

  // K * R * K^{-1}: from host to target
  // current estimate
  const Mat33f& PRE_KRKiTll = precalc->PRE_KRKiTll;
  const Vec3f& PRE_KtTll = precalc->PRE_KtTll;

  // Rth_0: rotation from host to target
  // first estimate
  const Mat33f& PRE_RTll_0 = precalc->PRE_RTll_0;

  // tth_0: translation from host to target
  // first estimate
  const Vec3f& PRE_tTll_0 = precalc->PRE_tTll_0;

  const float* const color = point->color;
  const float* const weights = point->weights;

  // ATTENTION: FIRST ESTIMATE
  const Vec2f affLL = precalc->PRE_aff_mode;
  const float b0 = precalc->PRE_b0_mode;

  Vec6f d_xi_x, d_xi_y;
  Vec4f d_C_x, d_C_y;
  float d_d_x, d_d_y;
  {
    // inverse depth target / inverse depth host
    float drescale;

    // pixel inverse depth wrt. target
    float new_idepth;

    // pixel coordinates in normalized plane target
    float u, v;

    // pixel coordinates in image target
    float Ku, Kv;

    // pixel coordinates in normalized plane host
    Vec3f KliP;

    // All estimates including idepth_zero_scaled are the first estimates!
    if (!projectPoint(point->u, point->v, point->idepth_zero_scaled, 0, 0,
                      HCalib, PRE_RTll_0, PRE_tTll_0, &drescale, &u, &v, &Ku,
                      &Kv, &KliP, &new_idepth)) {
      // if point is out of boundary of image 2
      state_NewState = ResState::OOB;
      return state_energy;
    }

    centerProjectedTo = Vec3f(Ku, Kv, new_idepth);  // [Ku Kv inv_d]

    // d(pixel coordinates in image target) / d(inverse depth wrt. host)
    d_d_x = drescale * (PRE_tTll_0[0] - PRE_tTll_0[2] * u) * SCALE_IDEPTH *
            HCalib->fxl();
    d_d_y = drescale * (PRE_tTll_0[1] - PRE_tTll_0[2] * v) * SCALE_IDEPTH *
            HCalib->fyl();

    // d(pixel coordinates in image target) / d(fx, fy, cx, cy)
    d_C_x[2] = drescale * (PRE_RTll_0(2, 0) * u - PRE_RTll_0(0, 0));
    d_C_x[3] = HCalib->fxl() * drescale *
               (PRE_RTll_0(2, 1) * u - PRE_RTll_0(0, 1)) * HCalib->fyli();
    d_C_x[0] = KliP[0] * d_C_x[2];
    d_C_x[1] = KliP[1] * d_C_x[3];

    d_C_y[2] = HCalib->fyl() * drescale *
               (PRE_RTll_0(2, 0) * v - PRE_RTll_0(1, 0)) * HCalib->fxli();
    d_C_y[3] = drescale * (PRE_RTll_0(2, 1) * v - PRE_RTll_0(1, 1));
    d_C_y[0] = KliP[0] * d_C_y[2];
    d_C_y[1] = KliP[1] * d_C_y[3];

    d_C_x[0] = (d_C_x[0] + u) * SCALE_F;
    d_C_x[1] *= SCALE_F;
    d_C_x[2] = (d_C_x[2] + 1) * SCALE_C;
    d_C_x[3] *= SCALE_C;

    d_C_y[0] *= SCALE_F;
    d_C_y[1] = (d_C_y[1] + v) * SCALE_F;
    d_C_y[2] *= SCALE_C;
    d_C_y[3] = (d_C_y[3] + 1) * SCALE_C;

    // d(pixel cooridinates in image target) / d(relative pose Tth)
    d_xi_x[0] = new_idepth * HCalib->fxl();
    d_xi_x[1] = 0;
    d_xi_x[2] = -new_idepth * u * HCalib->fxl();
    d_xi_x[3] = -u * v * HCalib->fxl();
    d_xi_x[4] = (1 + u * u) * HCalib->fxl();
    d_xi_x[5] = -v * HCalib->fxl();

    d_xi_y[0] = 0;
    d_xi_y[1] = new_idepth * HCalib->fyl();
    d_xi_y[2] = -new_idepth * v * HCalib->fyl();
    d_xi_y[3] = -(1 + v * v) * HCalib->fyl();
    d_xi_y[4] = u * v * HCalib->fyl();
    d_xi_y[5] = u * HCalib->fyl();
  }

  // ATTENTION: These derivatives are computed using FIRST ESTIMATE
  {
    J->Jpdxi[0] = d_xi_x;
    J->Jpdxi[1] = d_xi_y;

    J->Jpdc[0] = d_C_x;
    J->Jpdc[1] = d_C_y;

    J->Jpdd[0] = d_d_x;
    J->Jpdd[1] = d_d_y;
  }

  float JIdxJIdx_00 = 0, JIdxJIdx_11 = 0, JIdxJIdx_10 = 0;
  float JabJIdx_00 = 0, JabJIdx_01 = 0, JabJIdx_10 = 0, JabJIdx_11 = 0;
  float JabJab_00 = 0, JabJab_01 = 0, JabJab_11 = 0;

  float wJI2_sum = 0;
  float energyLeft = 0;  // total energy of this point (and the whole pattern)
  for (int idx = 0; idx < patternNum; ++idx) {
    float Ku, Kv;
    if (!projectPoint(point->u + patternP[idx][0], point->v + patternP[idx][1],
                      point->idepth_scaled, PRE_KRKiTll, PRE_KtTll, &Ku, &Kv)) {
      state_NewState = ResState::OOB;
      return state_energy;
    }

    projectedTo[idx][0] = Ku;
    projectedTo[idx][1] = Kv;

    // [intensity gx gy]
    Vec3f hitColor = (getInterpolatedElement33(dIl, Ku, Kv, wG[0]));
    float residual = hitColor[0] - (affLL[0] * color[idx] + affLL[1]);

    float drdA = (color[idx] - b0);
    if (!std::isfinite(hitColor[0])) {
      state_NewState = ResState::OOB;
      return state_energy;
    }

    float w = sqrtf(
        setting_outlierTHSumComponent /
        (setting_outlierTHSumComponent + hitColor.tail<2>().squaredNorm()));
    w = 0.5f * (w + weights[idx]);

    float hw = fabsf(residual) < setting_huberTH ? 1 : setting_huberTH /
                                                           fabsf(residual);
    energyLeft += w * w * hw * residual * residual * (2 - hw);

    {
      if (hw < 1) {
        hw = sqrtf(hw);
      }
      hw = hw * w;

      hitColor[1] *= hw;
      hitColor[2] *= hw;

      J->resF[idx] = residual * hw;

      // ATTENTION: These two derivatives are computed using CURRENT ESTIMATE
      J->JIdx[0][idx] = hitColor[1];
      J->JIdx[1][idx] = hitColor[2];

      // ATTENTION: These two derivatives are computed using FIRST ESTIMATE
      J->JabF[0][idx] = drdA * hw;
      J->JabF[1][idx] = hw;

      JIdxJIdx_00 += hitColor[1] * hitColor[1];
      JIdxJIdx_11 += hitColor[2] * hitColor[2];
      JIdxJIdx_10 += hitColor[1] * hitColor[2];

      JabJIdx_00 += drdA * hw * hitColor[1];
      JabJIdx_01 += drdA * hw * hitColor[2];
      JabJIdx_10 += hw * hitColor[1];
      JabJIdx_11 += hw * hitColor[2];

      JabJab_00 += drdA * drdA * hw * hw;
      JabJab_01 += drdA * hw * hw;
      JabJab_11 += hw * hw;

      wJI2_sum +=
          hw * hw * (hitColor[1] * hitColor[1] + hitColor[2] * hitColor[2]);

      if (setting_affineOptModeA < 0) {
        J->JabF[0][idx] = 0;
      }
      if (setting_affineOptModeB < 0) {
        J->JabF[1][idx] = 0;
      }
    }
  }

  J->JIdx2(0, 0) = JIdxJIdx_00;
  J->JIdx2(0, 1) = JIdxJIdx_10;
  J->JIdx2(1, 0) = JIdxJIdx_10;
  J->JIdx2(1, 1) = JIdxJIdx_11;
  J->JabJIdx(0, 0) = JabJIdx_00;
  J->JabJIdx(0, 1) = JabJIdx_01;
  J->JabJIdx(1, 0) = JabJIdx_10;
  J->JabJIdx(1, 1) = JabJIdx_11;
  J->Jab2(0, 0) = JabJab_00;
  J->Jab2(0, 1) = JabJab_01;
  J->Jab2(1, 0) = JabJab_01;
  J->Jab2(1, 1) = JabJab_11;

  state_NewEnergyWithOutlier = energyLeft;

  if (energyLeft >
          std::max<float>(host->frameEnergyTH, target->frameEnergyTH) ||
      wJI2_sum < 2) {
    // if the residual is too large, set it as outlier and return the energy
    // threshold
    energyLeft = std::max<float>(host->frameEnergyTH, target->frameEnergyTH);
    state_NewState = ResState::OUTLIER;
  } else {
    state_NewState = ResState::IN;
  }

  state_NewEnergy = energyLeft;  // new energy will be set in applyRes()
  return energyLeft;
}
```

### Literature

[^Huang08]: Huang, G. P., et al. “Analysis and Improvement of the Consistency of Extended Kalman Filter Based SLAM.” 2008 IEEE International Conference on Robotics and Automation, 2008, pp. 473–479.

[^Huang09]: Huang, Guoquan P., Anastasios I. Mourikis, and Stergios I. Roumeliotis. "A first-estimates Jacobian EKF for improving SLAM consistency." Experimental Robotics. Springer, Berlin, Heidelberg, 2009.

[^Huang10]: Huang, Guoquan P., et al. “Observability-Based Rules for Designing Consistent EKF SLAM Estimators.” The International Journal of Robotics Research, vol. 29, no. 5, 2010, pp. 502–528.


[^Engel18]: Engel, Jakob, Vladlen Koltun, and Daniel Cremers. "Direct Sparse Odometry." IEEE Transactions on Pattern Analysis and Machine Intelligence 40.3 (2018): 611-625.

[^Castellanos07]: J. Castellanos, R. Martinez-Cantin, J. Tardos, and J. Neira, “Robocentric map joining: Improving the consistency of EKF-SLAM,” Robotics and Autonomous Systems, vol. 55, pp. 21–29, 2007.
