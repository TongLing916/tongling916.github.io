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

$$\quad$$ The most suitable parametrizations for optimization are as uniform, finite and well-behaved as possible _near the current state estimate_. Ideally, they should be locally close to 
linaer in terms of their effect on the chosen error model, so that the cost function is locally quadratic.

$$\quad$$ State updates should be evaluated using a stable __local__ parametrization based on increments from the current estimate. As examples we consider 3D points and rotations.

__3D points:__ Projectively, _infinity is just like any other place._ You should use a homogeneous parametrization $$(X Y Z W)^\intercal$$for distant points, e.g. with spherical 
normlization $$\sum X_i^2 = 1$$.

__Rotations:__ Rotations should be parametrized using either quaternions subject to $$\left \| q \right \|^2 = 1$$, or local perturbations $$R \: \delta R$$ or $$\delta R \: R$$ of an existing rotation 
$$R$$, where $$\delta R$$ can be any well-behaved 3 parameter small rotation approximation.

__Sate updates:__ We assume that we can locally linearize the state manifold, locally resolving any internal constraints and freedoms that it may be subject to, to produce an unconstrained 
vector $$\delta x$$ parametrizing the possible local state displacement.

### 3 Error Modeling

$$\quad$$ The cost funtion $$f(x)$$ quantifies the total prediction (image reprojection) error of the model parametrized by the combined scene and camera parameter $$x$$. Robust statistically-based error 
metrics based on total (inlier + outlier) log likelihoods should be used, to correctly allow for the presence of outliers.

$$\quad$$ Bundle adjustment is essentially a parameter estimation problem. We will consider only __optimal point estimators__, whose output is by definition the single parameter vector that 
minimizes a predefined __cost function__ designed to measure how well the model fits the observations and background knowledge.

$$\quad$$ The __diespersion matrix:___ the inverse of the cost function Hessian $$H = \frac{\mathrm{d^2} f}{\mathrm{d} x^2}$$ at the minimum.

#### 3.1 Desiderata for the Cost Function

$$\quad$$ One of the great strengths of adjustment computations is their ability to combine information from disparate sources.

$$\quad$$ Accurate noise modelling is just as critical to successful estimation as accurate geometric modelling. The most important mismodelling is failure to take account of the possiblity of 
__outliers__ (aberrant data values, caused e.g. by blunders such as incorrect feature correspondences). _ML estimation is naturally robuts_: there is no need to robustify it so long as realistic 
error distributions were used in the first place. A distribution that models both inliers and outliers is called a __total distribution__. There is no need to separate the two classes, as ML 
estimation does not care about the distinction.

$$\quad$$ In summary, if there is a possiblity of outliers, non-robust distribution models such as Gaussians should be replaced with more realistic long-tailed ones. Poor robustness is due 
entirely to unrealistic distributional assumptions: the maximum likelihood framework itself is naturally robust provided that the total observation distribution including both inliers and 
outliers is modelled. 

$$\quad$$ In fact, real observations can seldom be cleanly divided into inliers and outliers. There is a hard core of outliers such as feature correspondences errors, but there is also a grey 
area of features that for some reason (a specularity, a shadow, poor focus, motion blur...) were not as accurately located as other features, without clearly being outliers.

#### 3.2 Nonlinear Least Squares

$$\quad$$ Nonlinear least squares takes as estimates the parameter values that minimize the __weighted Sum of Squared Error (SSE)__ cost function: <br>
$$
f(x) = \frac{1}{2} \sum_{i}\Delta z_i(x)^\intercal W_i \Delta z_i(x), \quad \Delta z_i(x) \equiv \underline z_i - z_i(x)  \quad (2)
$$
<br> Here, $$\Delta z_i(x)$$ is the feature prediction error and $$W_i$$ is an arbitrary symmetric positive definite (SPD) __weight matrix__. Modulo normalization terms independent of $$x$$, the 
weighted SSE cost function coincides with the negative log likelihood for observations $$\underline z_i$$ perturbed by Gaussian noise of mean zero and covariance $$W_i^{-1}$$. So for least squares 
to have a useful statistical interpretation, the $$W_i$$ should be chosen to approximate the inverse measurement covariance of $$\underline z_i$$ (See Probablistic Robotics 3.4 The Information 
Filter). 

_[What is the tail of a probablity distribution?][quora-tail-distribution]_

#### 3.3 Robustified Least Squares

$$\quad$$ The main problem with least squares is its high sensitivity to outliers. This happens because the Gaussian has extremely small tails compared to most real measurement error distributions.

$$\quad$$ When we say _observation_ we mean _irreducible group of observations treated as a unit by the robustifying model_. I.e., our observations need not be scalars, but they must be units, 
probabilistically independent of one another irrespective of whether they are inliers or outliers.

$$\quad$$ As usual, each independent observation $$\underline z_i$$ contributes an independent term $$f_i(x|\underline z_i)$$ to the total cost function. One very natural family are the __radia 
distributions__, which have negative log likelihoods of the form: <br>
$$
f(x) = \frac{1}{2} \rho _i(\Delta z_i(x)^\intercal W_i \Delta z_i(x))  \quad \quad (3)
$$
<br> Here, $$\rho _i (s)$$ can be any increasing function with $$\rho _i (0) = 0$$ and $$\frac{\mathrm{d} }{\mathrm{d} x} \rho _i (0) = 1$$. Weighted SSE has $$\rho _i (s) = s$$. The dispersion matrix 
$$W_i^{-1}$$ determines the spatial spread of $$\underline z_i$$, and up to its covariance (if this is finite).

> [What does the inverse of covariance matrix say about data?][quora-dispersion-matrix]

#### 3.4 Intensity-based methods

$$\quad$$ In intensity-based matching of image patches, the observables are image gray-scales or colors $$I$$ rather than feature coordinates $$u$$, and the error model is based on intensity residuals. 

$$\quad$$ Feature detectors are optimized  for detection not localization. To localize a detected feature accurately we need to match (some function of) the image intesities in its region against 
either an idealized template or another image of the feature, using an appropriate geometric deformation model, etc. 

#### 3.5 Implicit models 

$$\quad$$ Sometimes observations are most naturally expressed in terms of an implicit observation-constraining model $$h(x,z) = 0$$, rather than an explicit observation-predicting one $$z = z(x)$$. There 
are two ways to handle implicit models: nuisance parameters and reduction.

__Nuisance parameters:__ In this approach, the model is made explicit by adding additional 'nuisance' parameters representing something equivalent to model-consistent estimates of th unknown noise free 
observations, i.e. to $$z$$ with $$h(x,z) = 0$$. The most direct way to do this is to include the entire parameter vector $$z$$ as nuisance parameters, so that we have to solve a constrained optimization 
problem on the extended parameter space $$(x,z)$$, minimizing $$f(\underline z - z)$$ over $$(x,z)$$ subject to $$h(x,z) = 0$$. <br> The advantage of the nuisance parameter approach is that it gives 
the exact optimal parameter estimate for $$x$$, and jointly, optimal $$x$$-consistent estimates for the noise free observations $$z$$.

__Reduction:__ Alternatively, we can regard $$h(x,\underline z)$$ rather than $$\underline z$$ as the observation vector, and hence fit the parameters to the explicit log likelihood model 
for $$h(x,\underline z)$$.

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
[quora-tail-distribution]: https://www.quora.com/What-is-the-tail-of-a-probability-distribution
[quora-dispersion-matrix]: https://stats.stackexchange.com/questions/73463/what-does-the-inverse-of-covariance-matrix-say-about-data-intuitively