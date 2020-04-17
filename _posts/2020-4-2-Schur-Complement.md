---
layout:     post
title:      "Schur Complement"
date:       2020-4-2
author:     Tong
catalog: true
tags:
    - SLAM
---

### Foreknowledge

- For a diagonal matrix

$$A=\left(\begin{array}{cccc}
a_{11} & 0 & \cdots & 0 \\
0 & a_{22} & \cdots & 0 \\
\vdots & \vdots & & \vdots \\
0 & 0 & \cdots & a_{n n}
\end{array}\right)$$,

its inverse is 

$$A^{-1}=\left(\begin{array}{cccc}
1 / a_{11} & 0 & \cdots & 0 \\
0 & 1 / a_{22} & \cdots & 0 \\
\vdots & \vdots & \vdots \\
0 & 0 & \cdots & 1 / a_{n n}
\end{array}\right)$$

### Schur Complement [^Barfoot17]

#### Problem Formulation

In Newton's or Gauss-Newton method, we often need to solve the following equation

$$\underbrace{\left[\begin{array}{ll}
\mathbf{H}_{11} & \mathbf{H}_{12} \\
\mathbf{H}_{12}^{T} & \mathbf{H}_{22}
\end{array}\right]}_{\mathbf{H}} \underbrace{\left[\begin{array}{l}
\delta \mathbf{x}_{1} \\
\delta \mathbf{x}_{2}
\end{array}\right]}_{\delta \mathbf{x}}=\underbrace{-\left[\begin{array}{l}
\mathbf{b}_{1} \\
\mathbf{b}_{2}
\end{array}\right]}_{-\mathbf{b}}$$

|Variable|Meaning|
|---|---|
|$$\mathbf{H}$$|Hessian matrix|
|$$\delta \mathbf{x}$$|Increment of variables to optimize|
|$$\mathbf{x}_{1}$$|`K` variables to optimize|
|$$\mathbf{x}_{2}$$|`M` variables to optimize|

#### Technique

If $$\mathbf{H}_{22}$$ has some sparse structure (diagonal matrix) like below, 

![](https://cdn.mathpix.com/snip/images/PsleFG3qjwvIi4xNH6qIO7jG1vqZIb3Flt6z1gPaADY.original.fullsize.png)

we can exploit this sparsity by premultiplying both sides by
$$\left[\begin{array}{cc}
\mathbf{I} & -\mathbf{H}_{12} \mathbf{H}_{22}^{-1} \\
\mathbf{0} & \mathbf{I}
\end{array}\right]$$, then we have 

$$\left[\begin{array}{cc}
\mathbf{H}_{11}-\mathbf{H}_{12} \mathbf{H}_{22}^{-1} \mathbf{H}_{12}^{T} & \mathbf{0} \\
\mathbf{H}_{12}^{T} & \mathbf{H}_{22}
\end{array}\right]\left[\begin{array}{c}
\delta \mathbf{x}_{1} \\
\delta \mathbf{x}_{2}
\end{array}\right]=-\left[\begin{array}{c}
\mathbf{b}_{1}-\mathbf{H}_{12} \mathbf{H}_{22}^{-1} \mathbf{b}_{2} \\
\mathbf{b}_{2}
\end{array}\right]$$

As a result, we can solve $$\delta \mathbf{x}_{1}$$ with 
$$(\mathbf{H}_{11}-\mathbf{H}_{12} \mathbf{H}_{22}^{-1} \mathbf{H}_{12}^{T})\delta \mathbf{x}_{1} = -(\mathbf{b}_{1}-\mathbf{H}_{12} \mathbf{H}_{22}^{-1} \mathbf{b}_{2})
$$ first, then substitute it into 
the second equation 
$$\mathbf{H}_{12}^{T} \delta \mathbf{x}_{1} + \mathbf{H}_{22} \delta \mathbf{x}_{2} = -\mathbf{b}_{2}
$$ and solve $$\delta \mathbf{x}_{2} = -\mathbf{H}_{22}^{-1}(\mathbf{b}_{2} + \mathbf{H}^T_{12}\delta\mathbf{x}_{1})$$.

$$\delta \mathbf{x}_{2} = -\mathbf{H}_{22}^{-1}(\mathbf{b}_{2} + \mathbf{H}^T_{12}\delta\mathbf{x}_{1})$$

This procedure brings the complexity of each solve down from $$O((K + M)^3)$$ without sparsity to $$O((K^3 + K^2M))$$ with sparsity, which is most beneficial when
$$K \ll  M$$.

To speed up solving $$\delta \mathbf{x}_{1}$$, we can use some decomposition techniques introduced soon.

#### Gauss-Newton

As explained in this [Themenrunde](https://github.com/TongLing916/tongling916.github.io/blob/master/documents/Themenrunde_LDSO.pdf), the variables used in Gauss-Newton are computed as follows (Suppose weight matrix is an identity matrix)

$$\mathbf{H} = \mathbf{J}^T\mathbf{J}$$

$$\mathbf{b} = \mathbf{J}^T\mathbf{e}$$

|Variable | Meaning|
|---|---|
|$$\mathbf{J}$$|Jacobian matrix|
|$$\mathbf{e}$$|residual vector|

### Decomposition 

- Cholesky Decomposition [^Barfoot17]

- $$LDL^T$$ Decomposition [^Hartley04]

### [Example - DSO](http://www.lingtong.de/2020/04/17/DSO-Schur-Complement/) [^Engel18]


### Literature

[^Engel18]: Engel, Jakob, Vladlen Koltun, and Daniel Cremers. "Direct Sparse Odometry." IEEE Transactions on Pattern Analysis and Machine Intelligence 40.3 (2018): 611-625.

[^Barfoot17]: Timothy D Barfoot. State Estimation for Robotics. Cambridge University Press, 2017.

[^Hartley04]: R. Hartley and A. Zisserman, Multiple View Geometry in Computer Vision, 2nd ed. Cambridge University Press, 2004.