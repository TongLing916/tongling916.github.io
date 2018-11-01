---
layout:     post
title:      "Bundle Adjustment"
date:       2018-11-1
author:     Tong
catalog: true
tags:
    - Algorithm
---

> 1. [Bundle Adjustment - A Modern Synthesis][paper-ba] <br>
> 2. [A chinese explanation][blog-ba] 

### 1 Introduction

$$\quad$$ __Bundle adjustment__ is the problem of refining a visual reconstruction to produce _jointly optimal_ 3D structure and viewing parameter (camera pose and/or calibration) estimates. _Optimal_ 
means that the parameter estimates are found by minimizing some cost function that quantifies the model fitting error, and _jointly_ that the solution is simultaneously optimal with respect to both 
structure and camera variations. The name refers to the "bundle" of light rays leaving each 3D feature and converging on each camera centre, which are "adjusted" optimally with respect to both 
features and camera positions.

$$\quad$$ There are a number of __misconceptions__: <br>
1. Optimization / bundle adjustment is slow.
2. Only linear algebra is required.
3. Any sequence can be used.
4. Point $$P$$ is reconstructed accurately.

$$\quad$$ Why should you use bundle adjustment? <br>
1. Flexibility
2. Accuracy
3. Efficiency

### 2 Projection Model and Problem Parametrization

#### 2.1 The Projection Model

$$\quad$$ One of the great strengths of adjustment computations is their ability to take such complex and heterogeneous models in their stride. Almost any _predictive parametric_ models 
can be handled, i.e. any model that _predicts_ the values of some known measurements or descriptors on the basis of some continuous _parametric_ representation of the world, which is to 
be estimated from the measurements.

$$\quad$$ For each observation $$\underline x_{ip}$$ , we assume that we have a __predictive model__ $$x_{ip} = x(C_c,P_i,X_p)$$ based on the parameters, that can be used to 
derive a __feature prediction error__:
$$
\triangle x_{ip}(C_c,P_i,X_p) \equiv \underline x_{ip} - x(C_c,P_i,X_p)   \quad\quad\quad (1)
$$
<br> where $$X_p$$ are individual static 3D features, $$P_i$$ are internal calibration parameters, and $$C_c$$ are calibration parameters.

<br> To estimate the unknown 3D feature and camera parameters from the observations, and hence reconstruct the scene, we minimize some measure of their total prediction error. Bundle adjustment is 
the model refinement part of this.

#### 2.2 Bundle Parametrization



### 3 Error Modeling


#### 3.1 Desiderata for the Cost Function



#### 3.2 Nonlinear Least Squares



#### 3.3 Robustified Least Squares



#### 3.4 Intensity-based methods



#### 3.5 Implicit models 



### 4 Basic Numerical Optimization



#### 4.1 Second Order Methods



#### 4.2 Step Control 



#### 4.3 Gaus-Newton and Least Squares



#### 4.4 Constrained Problems



#### 4.5 General Implementations Issues



### 5 Network Structure



### 6 Implementation Strategy 1: Second Order Adjustment Methods


#### 6.1 The Schur Complement and the Reduced Bundle System



#### 6.2 Triangular decompositions



#### 6.3 Sparse factorization 



##### 6.3.1 Pattern matrices



##### 6.3.2 Top-down Ordering Methods



#### 6.3.3 Bottom-up Ordering Methods



### 7 Implementation Strategy 2: First Order Adjustment Methods



#### 7.1 First Order Iterations


#### 7.2 Why Are First Order Methods Slow?



#### 7.3 Preconditioning



#### 7.4 Experiments



### 8 Implementation Strategy 3: Updating and Recursion

#### 8.1 Updating rules



#### 8.2 Recursive Methods and Reduction



### 9 Gauge Freedom

#### 9.1 General Formulation



#### 9.2 Gauge constraints



#### 9.3 Inner Constraints



#### 9.4 Free Networks



#### 9.5 Implementation of Gauge Constraints



### 10 Quality Control



#### 10.1 Cost Perturbations



#### 10.2 Inner Reliability and Outlier Detection   



#### 10.3 Outer Reliability



#### 10.4 Sensitivity Analysis



#### 10.5 Model Selection



### 11 Network Design



### 12 Summary and Recommendations
 






[paper-ba]: https://hal.inria.fr/inria-00548290/document
[blog-ba]: https://blog.csdn.net/OptSolution/article/details/64442962