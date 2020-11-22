---
layout:     post
title:      "DSO - AccumulatorApprox"
date:       2020-4-17
author:     Tong
catalog: true
tags:
    - DSO
---

> [Direct sparse odometry](https://vision.in.tum.de/research/vslam/dso)

### Abstract

- This is a class to compute the `H` and `b` in the Gauss-Newton optimization. Related variables are
    - Intrinsic parameters ($$f_x, f_y, c_x, c_y$$)
    - Relative pose ($$\boldsymbol{\xi}_{ji}$$)
    - Relative photometric parameters ($$-e^{a_{ji}}, -b_{ji}$$)

### Variable

| Name | Type       | Formula                                                                                                               | Meaning                                                                                                                                                                                                                                                         |
| ---- | ---------- | --------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `H`  | `Mat1313f` | $$\begin{bmatrix}\mathbf{J}^{T} \mathbf{J} & \mathbf{b} \\ \mathbf{b}^{T} & \mathbf{e}^{T} \mathbf{e} \end{bmatrix}$$ | $$\mathbf{e} = \begin{bmatrix} r_{1} \\ \cdots \\ r_{n} \end{bmatrix}$$, $$\mathbf{J} = \frac{\partial \mathbf{e}}{\partial \begin{bmatrix} \mathbf{C} \\ \boldsymbol{\xi}_{ji} \\ \mathbf{a}_{ji} \end{bmatrix}}$$, $$\mathbf{b} = \mathbf{J}^{T} \mathbf{e}$$ |

#### Notes

1. In the real implementation, the matrix $$\mathbf{H} = \begin{bmatrix}\mathbf{J}^{T} \mathbf{J} & \mathbf{b} \\ \mathbf{b}^{T} & \mathbf{e}^{T} \mathbf{e} \end{bmatrix} = \begin{bmatrix}\mathbf{H}_{\text{tl}} & \mathbf{H}_{\text{tr}} \\ \mathbf{H}_{\text{tr}}^{T} & \mathbf{H}_{\text{br}} \end{bmatrix}$$ is divided into three parts.
   1. The first part is the __top left__ `10 x 10`. 
      - $$\mathbf{H}_{\text{tl}} = \mathbf{J}_{\text{geo}}^{T}\mathbf{J}_{\text{geo}}$$, where $$\mathbf{J}_{\text{geo}} = \frac{\partial \mathbf{e}}{\partial \begin{bmatrix} \mathbf{C} \\ \boldsymbol{\xi}_{ji}\end{bmatrix}}$$.
      - Due to the symmetric property, there are only $$10 + 9 + 8 + \cdots + 1 = 55$$ elements computed.
   2. The second part is the __top right__ `10 x 3`.
      - $$\mathbf{H}_{\text{tr}} = \begin{bmatrix}\mathbf{J}_{\text{geo}}^{T}\mathbf{J}_{\text{photo}} & \mathbf{J}_{\text{geo}}^{T} \mathbf{e}\end{bmatrix}$$, where $$\mathbf{J}_{\text{photo}} = \frac{\partial \mathbf{e}}{\partial \mathbf{a}_{ji}}$$.
   3. The third part is the __bottom right__ `3 x 3`.
      - $$\mathbf{H}_{\text{br}} = \begin{bmatrix}\mathbf{J}_{\text{photo}}^{T}\mathbf{J}_{\text{photo}} & \mathbf{J}_{\text{photo}}^{T} \mathbf{e} \\ (\mathbf{J}_{\text{photo}}^{T} \mathbf{e})^{T} & \mathbf{e}^{T} \mathbf{e} \end{bmatrix}$$.
      - Due to the symmetric property, there are only $$3 + 2 + 1 = 6$$ elements computed.

### Function

### void update()

```c++
/** \brief Compute the top left part of Hessian
 *
 *  Compute the Hessian ONLY related to intrinsic parameters and relative
 *  pose. Since it is a symmetric matrix, we only need to compute the upper
 *  right part of it.
*/
void update(const float* const x4, const float* const x6,
            const float* const y4, const float* const y6, const float a,
            const float b, const float c);
```

| Name | Formula                                                                                        | Meaning |
| ---- | ---------------------------------------------------------------------------------------------- | ------- |
| `x4` | $$\frac{\partial u_j}{\partial\begin{bmatrix} f_{x} \\ f_{y} \\ c_{x} \\ c_{y}\end{bmatrix}}$$ |         |
| `x6` | $$\frac{\partial u_j}{\partial\boldsymbol{\xi_{ji}}}$$                                         |         |
| `y4` | $$\frac{\partial v_j}{\partial\begin{bmatrix} f_{x} \\ f_{y} \\ c_{x} \\ c_{y}\end{bmatrix}}$$ |         |
| `y6` | $$\frac{\partial v_j}{\partial\boldsymbol{\xi_{ji}}}$$                                         |         |
| `a`  | $$\frac{\partial r_{ji}}{\partial u_j}\frac{\partial r_{ji}}{\partial u_j}$$                   |         |
| `b`  | $$\frac{\partial r_{ji}}{\partial u_j}\frac{\partial r_{ji}}{\partial v_j}$$                   |         |
| `c`  | $$\frac{\partial r_{ji}}{\partial v_j}\frac{\partial r_{ji}}{\partial v_j}$$                   |         |


### void updateTopRight()

```c++
void updateTopRight(const float* const x4, const float* const x6,
                    const float* const y4, const float* const y6,
                    const float TR00, const float TR10, const float TR01,
                    const float TR11, const float TR02, const float TR12);
```

| Name   | Formula                                                                                        | Meaning |
| ------ | ---------------------------------------------------------------------------------------------- | ------- |
| `x4`   | $$\frac{\partial u_j}{\partial\begin{bmatrix} f_{x} \\ f_{y} \\ c_{x} \\ c_{y}\end{bmatrix}}$$ |         |
| `x6`   | $$\frac{\partial u_j}{\partial\boldsymbol{\xi_{ji}}}$$                                         |         |
| `y4`   | $$\frac{\partial v_j}{\partial\begin{bmatrix} f_{x} \\ f_{y} \\ c_{x} \\ c_{y}\end{bmatrix}}$$ |         |
| `y6`   | $$\frac{\partial v_j}{\partial\boldsymbol{\xi_{ji}}}$$                                         |         |
| `TR00` | $$\frac{\partial r_{ji}}{\partial (-e^{a_{ji}})}\frac{\partial r_{ji}}{\partial u_{j}}$$       |         |
| `TR10` | $$\frac{\partial r_{ji}}{\partial (-e^{a_{ji}})}\frac{\partial r_{ji}}{\partial v_{j}}$$       |         |
| `TR01` | $$\frac{\partial r_{ji}}{\partial (-b_{ji})}\frac{\partial r_{ji}}{\partial u_{j}}$$           |         |
| `TR11` | $$\frac{\partial r_{ji}}{\partial (-b_{ji})}\frac{\partial r_{ji}}{\partial v_{ji}}$$          |         |
| `TR02` | $$r_{ji}\frac{\partial r_{ji}}{\partial u_{j}}$$                                               |         |
| `TR12` | $$r_{ji}\frac{\partial r_{ji}}{\partial v_{j}}$$                                               |         |


### void updateBotRight()

```c++
/** \brief Compute the bottom right part of Hessian
 *
 * Compute the Hessian and b ONLY related to photometric affine parameters
*/
void updateBotRight(const float a00, const float a01, const float a02,
                    const float a11, const float a12,
                    const float a22);
```

| Name  | Formula                                                                                          | Meaning |
| ----- | ------------------------------------------------------------------------------------------------ | ------- |
| `a00` | $$\frac{\partial r_{ji}}{\partial (-e^{a_{ji}})}\frac{\partial r_{ji}}{\partial (-e^{a_{ji}})}$$ |         |
| `a01` | $$\frac{\partial r_{ji}}{\partial (-e^{a_{ji}})}\frac{\partial r_{ji}}{\partial (-b_{ji})}$$     |         |
| `a02` | $$r_{ji}\frac{\partial r_{ji}}{\partial (-e^{a_{ji}})}$$                                         |         |
| `a11` | $$\frac{\partial r_{ji}}{\partial (-b_{ji})}\frac{\partial r_{ji}}{\partial (-b_{ji})}$$         |         |
| `a12` | $$r_{ji}\frac{\partial r_{ji}}{\partial (-b_{ji})}$$                                             |         |
| `a22` | $$r_{ji}^{2}$$                                                                                   |         |

