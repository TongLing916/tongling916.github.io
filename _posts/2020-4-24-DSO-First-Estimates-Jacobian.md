---
layout:     post
title:      "First Estimates Jacobian"
date:       2020-4-24
author:     Tong
catalog: true
tags:
    - SLAM
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

### Literature

[^Huang08]: Huang, G. P., et al. “Analysis and Improvement of the Consistency of Extended Kalman Filter Based SLAM.” 2008 IEEE International Conference on Robotics and Automation, 2008, pp. 473–479.

[^Huang09]: Huang, Guoquan P., Anastasios I. Mourikis, and Stergios I. Roumeliotis. "A first-estimates Jacobian EKF for improving SLAM consistency." Experimental Robotics. Springer, Berlin, Heidelberg, 2009.

[^Huang10]: Huang, Guoquan P., et al. “Observability-Based Rules for Designing Consistent EKF SLAM Estimators.” The International Journal of Robotics Research, vol. 29, no. 5, 2010, pp. 502–528.


[^Engel18]: Engel, Jakob, Vladlen Koltun, and Daniel Cremers. "Direct Sparse Odometry." IEEE Transactions on Pattern Analysis and Machine Intelligence 40.3 (2018): 611-625.

[^Castellanos07]: J. Castellanos, R. Martinez-Cantin, J. Tardos, and J. Neira, “Robocentric map joining: Improving the consistency of EKF-SLAM,” Robotics and Autonomous Systems, vol. 55, pp. 21–29, 2007.
