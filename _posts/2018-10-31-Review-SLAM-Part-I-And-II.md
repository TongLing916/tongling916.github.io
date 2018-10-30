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

$$\quad$$ 

### 2. History of the SLAM Problem

### 3. Formulation and Structure of the SLAM problem

#### 3.1 Preliminaries

#### 3.2 Probablistic SLAM

#### 3.3 Structure of Probablistic SLAM

### 4. Solutions to the SLAM Problem

#### 4.1 EKF-SLAM

#### 4.2 Rao-Blackwellised Filter

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
