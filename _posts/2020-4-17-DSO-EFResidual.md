---
layout:     post
title:      "DSO - EFResidual"
date:       2020-4-17
author:     Tong
catalog: true
tags:
    - DSO
---

> [Direct sparse odometry](https://vision.in.tum.de/research/vslam/dso)

### Variable

| Variable | Formula                                                                                                                                                                                                                                                                      | Description |
| -------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------- |
| `JpJdF`  | $$\begin{bmatrix} (\frac{\partial r_{ji}}{\partial \boldsymbol{\xi}_{ji}})^T\frac{\partial r_{ji}}{\partial \rho_{i}} \\ (\frac{\partial r_{ji}}{\partial \begin{bmatrix} -e^{a_{ji}} \\ -b_{ji}\end{bmatrix}})^{T}\frac{\partial r_{ji}}{\partial \rho_{i}} \end{bmatrix}$$ |             |

### Function

### void fixLinearizationF()

```c++
/** \brief Fix a point's residual */
void fixLinearizationF(EnergyFunctional* ef);
```

- This function is usually called when we want to remove some point, so we need to fix the residual (not update it anymore).
  - The formula to compute the residual is $$r_{ji} = r_{ji}^{0} + \begin{bmatrix} \frac{\partial r_{ji}}{\partial \mathbf{p}_{j}} & \frac{\partial r_{ji}}{\partial \mathbf{a}_{ji}}\end{bmatrix} \begin{bmatrix}\delta{\mathbf{p}_{j}} \\ \delta{\mathbf{a}_{ji}}\end{bmatrix}$$, where 

$$
\delta \mathbf{p}_{j} = \begin{bmatrix}
    \frac{\partial \mathbf{p}_{j}}{\partial \mathbf{C}} & \frac{\partial \mathbf{p}_{j}}{\partial \boldsymbol{\xi}_{ji}} & \frac{\partial \mathbf{p}_{j}}{\partial \rho_{i}}
\end{bmatrix}
\begin{bmatrix}
    \delta \mathbf{C} \\
    \delta \boldsymbol{\xi}_{ji} \\
    \delta \rho_{i} 
\end{bmatrix}
$$

$$
\delta{\boldsymbol{\xi}_{ji}} = \begin{bmatrix} \frac{\partial \boldsymbol{\xi}_{ji}}{\partial \boldsymbol{\xi}_{iw}} & \frac{\partial \boldsymbol{\xi}_{ji}}{\partial \boldsymbol{\xi}_{jw}}\end{bmatrix} \begin{bmatrix}\delta{\boldsymbol{\xi}_{iw}} \\  \delta{\boldsymbol{\xi}_{jw}}\end{bmatrix}
$$

$$\delta{\mathbf{a}_{ji}}  = \begin{bmatrix} \frac{\partial \mathbf{a}_{ji}}{\partial \mathbf{a}_{i}} & \frac{\partial \mathbf{a}_{ji}}{\partial \mathbf{a}_{j}}\end{bmatrix}\begin{bmatrix} \delta{\mathbf{a}_{i}} \\ \delta{\mathbf{a}_{j}}\end{bmatrix}$$