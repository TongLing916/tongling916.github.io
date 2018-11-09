---
layout: post
title: "Chapter 4: Estimation-2D Projective Transformations"
date:       2018-11-09
author:     Tong
catalog: true
tags:
    - MVG
---

### 4.1 The Direct Linear Transformation (DLT) algorithm

1. $$\begin{bmatrix}0^\intercal & -\omega _i^\prime x_i^\intercal & y_i^\prime x_i^\intercal\\ \omega _i^\prime x_i^\intercal & 0^\intercal & -x_i^\prime x_i^\intercal\\ -y_i^\prime x_i^\intercal & x_i^\prime x_i^\intercal & 0^\intercal \end{bmatrix} \begin{pmatrix}h^1\\ h^2 \\ h^3 \end{pmatrix} = 0    \quad \quad (4.1)$$

2. $$h = \begin{pmatrix} h^1\\ h^2 \\ h^3 \end{pmatrix}, H = \begin{bmatrix} h_1 & h_2 & h_3 \\ h_4 & h_5 & h_6 \\ h_7 & h_8 & h_9 \end{bmatrix}    \quad \quad$$ (4.2)

3. $$\begin{align}
\begin{bmatrix}
0^\intercal & -\omega _i^\prime x_i^\intercal & y_i^\prime x_i^\intercal \\ 
\omega _i^\prime x_i^\intercal & 0^\intercal & -x_i^\prime x_i^\intercal 
\end{bmatrix}
\begin{pmatrix} h^1\\ h^2 \\ h^3 \end{pmatrix} & = 0    \\
A_i h & = 0
\end{align} \quad \quad (4.3) $$

4. Algorithm 4.1. The basic DLT for H (but see algorithm 4.2 which includes normalization <br>
> $$ \underline{Objective} $$ <br>
> Given $$n \geq 4$$ 2D to 2D point correspondences $$\left \{ x_i \leftrightarrow x_i^\prime \right \}$$, determine the 2D homography matrix H such that $$x_i^\prime = Hx_i$$.<br>
> $$ \underline{Algorithm} $$ <br>
> (i) For each correspondence $$ x_i \leftrightarrow x_i^\prime $$ compute the matrix $$A_i$$ from(4.1). Only the first two rows end need be used in general.   <br>
> (ii) Assemble the $$n \: 2 \times 9$$ matrices $$A_i$$ into a single $$2n \times 9$$ matrix A.<br>
> (iii) Obtain the SVD of A. The unit singular vector corresponding to the smallest singular value is the solution $$h$$. Sepcifically, if $$A = UDV^\intercal$$ with $$D$$ diagonal with positive diagonal entries, arranged in descending order down the diagonal, then $$h$$ is the last colomn of $$V$$.<br>
> (iv) The matrix $$H$$ is determined from $$h$$ as in (4.2).

5. A situation where a configuration does not determine a unique solution for a particular class of transformation is termed _degenerate_.


### 4.2 Different cost functions

0. __Notation.__ Vectors $$x$$ represent the _measured_ image coordinates; $$\hat{x}$$ represent estimated values of the points and $$\bar{x}$$ represent true values of the points.

1. Algebraic distance: 
$$
d_{alg}(x_i^\prime,Hx_i)^2 = \left \| \epsilon _i \right \|^2 = \left \| \begin{bmatrix}
0^\intercal & -\omega _i^\prime x_i^\intercal & y_i^\prime x_i^\intercal\\ \omega _i^\prime x_i^\intercal & 0^\intercal & -x_i^\prime x_i^\intercal
\end{bmatrix} h \right \|    \quad \quad (4.4)
$$

2. Geometric distance - error in one image:
$$
\sum_{i} d(x_i^\prime,H \bar{x_i})^2     \quad \quad (4.6)
$$

3. Geometric distance - symmetric transfer error:  
$$
\sum_{i} d(x_i,H^{-1} x_i^\prime)^2 + d(x_i^\prime,H x_i)^2    \quad \quad (4.7)
$$

4. Reprojection error - both images:
$$
\sum_{i} d(x_i,\hat{x_i})^2 + d(x_i^\prime,\hat{x_i}^\prime)^2 \;\; \; \;   subject\;  to \;  \hat{x_i}^\prime = \hat{H}\hat{x_i}\quad \quad (4.8)
$$

![](http://www.robots.ox.ac.uk/~vgg/hzbook/hzbook2/WebPage/pngfiles/estimationfigs-error.png)

### 4.3 Statistical cost functions and Maximum Likelihood estimation

1. The _Maximum Likelihood estimation_ is equivalent to minimizing the geometric error function / reprojection error function / Mahalanobis distance.

### 4.4 Transformation invariance and normalization

1. __Reuslt 4.4.__ Let $$T^\prime$$ be a similarity transformation with sclae factor $$s$$, and let $$T$$ be an arbitrary projective transformation. 
Further, suppose $$H$$ is any 2D homography and let $$\tilde{H}$$ be defined by $$\tilde{H} = T^\prime H T^{-1}$$. Then $$\left \| \tilde{A} \tilde{h} \right \| = s\left \| Ah \right \|$$ 
where $$h$$ and $$\tilde{h}$$ are the vectors fo entries of $$H$$ and $$\tilde{H}$$.

2. Data normalization is an essential step in the DLT algorithm. It must not be considered optional.

3. Algorithm 4.2. The normalized DLT for 2D homographies.

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/post-normalized-DLT.PNG)


### 4.5 Iterative minimization methods

1. The technique of iterative minization generally consists of five steps: <br>
(i) __Cost function.__ A cost function is chosen as the basis for minimization.<br>
(ii) __Parameterization.__ The transformation (or other entity) to be computed is expressed in terms of a finite number of parameters. It is not 
general neccessary that this is a minimum set of parameters, and there are in fact often advantages to over-parametrization.<br>
(iii) __Function specification.__ A function must be specified that expresses the cost in terms of the set of parameters.<br>
(iv) __Initialization.__ A suitable initial parameter estimate is computed. This will generally be done using a linear algorithm such as the DLT algorithm.<br>
(v) __Iteration.__ Starting from the initial solution, the parameters are iteratively refined with the goal of minimizing the cost function.

2. The Gold Standard algorithm and variations for estimating $$H$$ from image correspondences. The Gold Standard algorithm is preferred to the Sampson 
method for 2D homography computation.

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/post-gold-standard.PNG)

### 4.6 Experimental comparison of the algorithms

### 4.7 Robust estimation 

1. Algorithm 4.4. The _RANSAC_ robust estimation algorithm. A minimum of $$s$$ data points are required to instantiate the free parameters of the model.

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/post-ransac-robust-estimation.PNG)

2. Algorithm 4.5. Adaptive algorithm for determining the number of RANSAC samples.

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/post-adaptive-ransac.PNG)

### 4.8 Automatic computation of a homography

1. Algorithm 4.6. Automatic estimation of a homography between two images using RANSAC.

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/post-automatic-estimation.PNG)
 
### 4.9 Closure

1. The Gold Standard Algorithm for estimating an affine homography $$H_A$$ from image correspondences.

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/post-gold-standard-affine.PNG)

































