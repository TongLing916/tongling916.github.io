---
layout:     post
title:      "Review - Simultaneous Localisation and Mapping (SLAM): Part I and II"
date:       2018-10-31
author:     Tong
catalog: true
tags:
   - SLAM
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

### 1. Computational Complexity

### 2. State Augmentation

### 3. Partitioned Updates

### 4. Sparsification

### 5. Global Submaps

### 6. Relative Submaps 

### 7. Data Association

### 8. Batch Validation

### 9. Appearance Signatures

### 10. Multihypothesis Data Association

### 11. Environment Representation

### 12. Partial Observability and Delayed Mapping

### 13. Nongeometric Landmarks

### 14. 3D SLAM

### 15. Trajectory-Oriented SLAM

### 16. Embedded Auxiliary Information

### 17. Dynamic Environments

### 18. SLAM: Where to Next? 

[paper-part-1]: https://people.eecs.berkeley.edu/~pabbeel/cs287-fa09/readings/Durrant-Whyte_Bailey_SLAM-tutorial-I.pdf
[paper-part-2]: https://ieeexplore.ieee.org/document/1678144
