---
layout:     post
title:      "Schur Complement"
date:       2020-4-2
author:     Tong
catalog: true
tags:
    - SLAM
---

### Schur Complement [^Barfoot17]

#### Probelm Formulation

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

### Decomposition 

- Cholesky Decomposition [^Barfoot17]

- $LDL^T$ Decomposition

### Example - DSO [^Engel18]

### Literature

[^Engel18]: Engel, Jakob, Vladlen Koltun, and Daniel Cremers. "Direct Sparse Odometry." IEEE Transactions on Pattern Analysis and Machine Intelligence 40.3 (2018): 611-625.

[^Barfoot17]: Timothy D Barfoot. State Estimation for Robotics. Cambridge University Press, 2017.