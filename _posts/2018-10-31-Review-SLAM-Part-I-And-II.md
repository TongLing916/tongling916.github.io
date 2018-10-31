---
layout:     post
title:      "Review - Simultaneous Localisation and Mapping (SLAM): Part I and II"
date:       2018-10-31
author:     Tong
catalog: true
tags:
   - Review
---

## [Part I - The Essential Algorithms][paper-part-1]

$$\quad$$ SLAM is the process by which a mobile robot cna build a map of an environment and at the same time use this map to compute its own location.
Part I of this tutorial describes the probablistic form of the SLAM problem, essential solution methods and significant implementations. Part II of 
this tutorial will be concerned with recent advances in computational methods and new formulations of the SLAM for large scale and complex environments.

### 1. Introduction

### 2. History of the SLAM Problem

$$\quad$$ The correlations between landmarks were actually the critical part of the problem and the more these correlations grew, the better the solution. 

### 3. Formulation and Structure of the SLAM problem

#### 3.1 Preliminaries

#### 3.2 Probablistic SLAM

#### 3.3 Structure of Probablistic SLAM

$$\quad$$ The most important insight in SLAM was to realize that the correlations between landmark estimates increase _monotonically_ as 
more and more observations are made.

### 4. Solutions to the SLAM Problem

#### 4.1 EKF-SLAM

__Convergence:__ In the EKF-SLAM algorithm, convergence of the map is manifest in the monotonic convergence of the determinant of the map 
covariance matrix $$P_{mm,k}$$, and all landmark variances converge toward a lower bound determined by initial uncertainties in robot position 
and observations.

__Computational Effort:__ Computation grows quadraticallly with the number of landmarks.

__Data Association:__ The standard formulation of the EKF-SLAM solution specially fragile to incorrect association of observations to landmarks. 
The "loop-closure" problem, when a robot returns to re-observe landmarks after a large traverse, is especially difficult. The association problem is 
compounded in environments where landmarks are not simple points and indeed look different from different view-points.

__Non-linearity:__ Non-linearity can be a significant problem in EKF-SLAM and leads to inevitable, and sometimes dramatic, inconsistency in solutions. 
Convergence and consistency can only be guaranteed in the linear case.

#### 4.2 Rao-Blackwellised Filter

$$\quad$$ Fast-SLAM with its basis on recursive Monte Carlo sampling, or particle filtering, was the first to directly represent the non-linear 
process model and non-Gaussian pose distribution. 

$$\quad$$ When conditioned on the trajectory, the map landmarks become independent. This is a key property of FastSLAM, and the reason for its speed: 
the map is represented as a set of independent Gaussians, with linear complexity, rather than a joint map covariance with quadratic complexity.

$$\quad$$ There are two versions of FastSLAM. They differ only in terms of the form of their proposal distribution and, consequently in their 
importance weight.

### 5. Implementationsof SLAM

### 6. Conclusions

## [Part II][paper-part-2]

$$\quad$$ Focus on three key areas: computational complexity, data association, and environment representation.

### 1. Computational Complexity

$$\quad$$ The direct approach to reducing computational complexity involves exploting the structure of the SLAM problem in re-formulating 
the essential time- and observation-update equations to limit required computation. 

$$\quad$$ The time-update computation can be limited using _state-augmentation_ methods. 

$$\quad$$ The observation-update computation can be limited using a _partitioned form_ of the update equations. 

$$\quad$$ Re-formulation of the standard state-space SLAM representation into information form allows _sparsification_ of the resulting information 
matrix to be exploited in reducing computation. 

$$\quad$$ _Submapping_ methods exploit the idea that a map can be broken up into regions with local coordinate systems and arranged in a hierarchical 
manner. Updates can occur in a local frame with periodic interframe updates.


#### 1.1 State Augmentation

#### 1.2 Partitioned Updates

$$\quad$$ The submap method has a number of advantages. First, the number of landmarks that must be updated at any on time is limitted to only 
those that are described in the local submap coordinate frame. Thus, the observation-rate update is independent of local estimates. The full update, 
and the propagation of local estimates, can be carried out as a background task at a much lower update rate while still permitting observation-rate 
global localization. A second advantage is that there is lower uncertainty in a locally referenced frame, so approximations due to linearization 
are reduced. Finally, submap registration can use batch-validation gating, thereby improving association robustness. 

#### 1.3 Sparsification

$$\quad$$ The key to exact sparsification of the information form of the SLAM problem is to notice that state augmentation is a sparse operation.

#### 1.4 Global Submaps

#### 1.5 Relative Submaps 

### 2. Data Association

#### 2.1. Batch Validation

#### 2.2. Appearance Signatures

#### 2.3 Multihypothesis Data Association

### 3. Environment Representation

#### 3.1 Partial Observability and Delayed Mapping

#### 3.2 Nongeometric Landmarks

#### 3.3 3D SLAM

#### 3.4 Trajectory-Oriented SLAM

#### 3.5 Embedded Auxiliary Information

#### 3.6 Dynamic Environments

### 4. SLAM: Where to Next? 

[paper-part-1]: https://people.eecs.berkeley.edu/~pabbeel/cs287-fa09/readings/Durrant-Whyte_Bailey_SLAM-tutorial-I.pdf
[paper-part-2]: https://ieeexplore.ieee.org/document/1678144
