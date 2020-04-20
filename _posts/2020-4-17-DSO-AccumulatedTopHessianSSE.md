---
layout:     post
title:      "DSO - AccumulatedTopHessianSSE"
date:       2020-4-17
author:     Tong
catalog: true
tags:
    - DSO
---

> [Direct sparse odometry](https://vision.in.tum.de/research/vslam/dso)

### Variable

### Function

#### void addPoint()

```c++
template <int mode>
void addPoint(EFPoint* const p, const EnergyFunctional* const ef,
              const int tid = 0);
```
- This function has three modes
  - `active: mode = 0`
  - `marginalize: mode = 2`
  - `linearize (mode = 1)`: NOT USED, because there are NEVER linearized and active points available.

#### void stitchDoubleInternal()

```c++
void stitchDoubleInternal(MatXX* H, VecX* b, const EnergyFunctional* const EF,
                          bool usePrior, int min, int max, Vec10* stats,
                          int tid);
```

- Actually, this matrix computes [$$\mathbf{H}_{11}$$](http://www.lingtong.de/2020/04/17/DSO-Schur-Complement/#hessian) and [$$\mathbf{b}_{1}$$](http://www.lingtong.de/2020/04/17/DSO-Schur-Complement/#b), but there are some equations to mention,
  - $$\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}} = \frac{\partial r}{\partial \boldsymbol{\xi}_{ji}}\frac{\partial \boldsymbol{\xi}_{ji}}{\partial \boldsymbol{\xi}_{iw}}$$
  - $$\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}} = \frac{\partial r}{\partial \boldsymbol{\xi}_{ji}}\frac{\partial \boldsymbol{\xi}_{ji}}{\partial \boldsymbol{\xi}_{iw}}$$
  - 
  - $$(\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}})^{T}\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}} = (\frac{\partial \boldsymbol{\xi}_{ji}}{\partial \boldsymbol{\xi}_{iw}})^{T}((\frac{\partial r}{\partial \boldsymbol{\xi}_{ji}})^{T}\frac{\partial r}{\partial \boldsymbol{\xi}_{ji}})\frac{\partial \boldsymbol{\xi}_{ji}}{\partial \boldsymbol{\xi}_{iw}}$$
  - $$(\frac{\partial r}{\partial \boldsymbol{\xi}_{jw}})^{T}\frac{\partial r}{\partial \boldsymbol{\xi}_{jw}} = (\frac{\partial \boldsymbol{\xi}_{ji}}{\partial \boldsymbol{\xi}_{jw}})^{T}((\frac{\partial r}{\partial \boldsymbol{\xi}_{ji}})^{T}\frac{\partial r}{\partial \boldsymbol{\xi}_{ji}})\frac{\partial \boldsymbol{\xi}_{ji}}{\partial \boldsymbol{\xi}_{jw}}$$
  - $$\frac{\partial r}{\partial \mathbf{a}_{i}} = \frac{\partial r}{\partial\mathbf{a}_{ji}}\frac{\partial \mathbf{a}_{ji}}{\partial \mathbf{a}_{i}}$$
  - $$\frac{\partial r}{\partial \mathbf{a}_{j}} = \frac{\partial r}{\partial\mathbf{a}_{ji}}\frac{\partial \mathbf{a}_{ji}}{\partial \mathbf{a}_{j}}$$
- In the real implementation, we compute $$\mathbf{H}_{1}$$ and $$\mathbf{b}_{1}$$ block by block.
  - $$\begin{bmatrix}(\frac{\partial r}{\partial  \boldsymbol{\xi}_{iw}})^{T} \frac{\partial r}{\partial  \boldsymbol{\xi}_{iw}} & (\frac{\partial r}{\partial  \boldsymbol{\xi}_{iw}})^{T} \frac{\partial r}{\partial \mathbf{a}_{i}} \\ (\frac{\partial r}{\partial \mathbf{a}_{i}})^{T} \frac{\partial r}{\partial  \boldsymbol{\xi}_{iw}} & (\frac{\partial r}{\partial \mathbf{a}_{i}})^{T} \frac{\partial r}{\partial \mathbf{a}_{i}} \end{bmatrix} = \begin{bmatrix} (\frac{\partial \boldsymbol{\xi}_{ji}}{\partial \boldsymbol{\xi}_{iw}})^{T} & \\ & (\frac{\partial \mathbf{a}_{ji}}{\partial \mathbf{a}_{i}})^{T}\end{bmatrix} \begin{bmatrix}(\frac{\partial r}{\partial  \boldsymbol{\xi}_{ji}})^{T} \frac{\partial r}{\partial  \boldsymbol{\xi}_{ji}} & (\frac{\partial r}{\partial  \boldsymbol{\xi}_{ji}})^{T} \frac{\partial r}{\partial \mathbf{a}_{ji}} \\ (\frac{\partial r}{\partial \mathbf{a}_{ji}})^{T} \frac{\partial r}{\partial  \boldsymbol{\xi}_{ji}} & (\frac{\partial r}{\partial \mathbf{a}_{ji}})^{T} \frac{\partial r}{\partial \mathbf{a}_{ji}} \end{bmatrix}\begin{bmatrix} \frac{\partial \boldsymbol{\xi}_{ji}}{\partial \boldsymbol{\xi}_{iw}} & \\ & \frac{\partial \mathbf{a}_{ji}}{\partial \mathbf{a}_{i}}\end{bmatrix}$$
  - $$\begin{bmatrix}(\frac{\partial r}{\partial  \boldsymbol{\xi}_{iw}})^{T} \frac{\partial r}{\partial  \boldsymbol{\xi}_{jw}} & (\frac{\partial r}{\partial  \boldsymbol{\xi}_{iw}})^{T} \frac{\partial r}{\partial \mathbf{a}_{j}} \\ (\frac{\partial r}{\partial \mathbf{a}_{i}})^{T} \frac{\partial r}{\partial  \boldsymbol{\xi}_{jw}} & (\frac{\partial r}{\partial \mathbf{a}_{i}})^{T} \frac{\partial r}{\partial \mathbf{a}_{j}} \end{bmatrix} = \begin{bmatrix} (\frac{\partial \boldsymbol{\xi}_{ji}}{\partial \boldsymbol{\xi}_{iw}})^{T} & \\ & (\frac{\partial \mathbf{a}_{ji}}{\partial \mathbf{a}_{i}})^{T}\end{bmatrix} \begin{bmatrix}(\frac{\partial r}{\partial  \boldsymbol{\xi}_{ji}})^{T} \frac{\partial r}{\partial  \boldsymbol{\xi}_{ji}} & (\frac{\partial r}{\partial  \boldsymbol{\xi}_{ji}})^{T} \frac{\partial r}{\partial \mathbf{a}_{ji}} \\ (\frac{\partial r}{\partial \mathbf{a}_{ji}})^{T} \frac{\partial r}{\partial  \boldsymbol{\xi}_{ji}} & (\frac{\partial r}{\partial \mathbf{a}_{ji}})^{T} \frac{\partial r}{\partial \mathbf{a}_{ji}} \end{bmatrix}\begin{bmatrix} \frac{\partial \boldsymbol{\xi}_{ji}}{\partial \boldsymbol{\xi}_{jw}} & \\ & \frac{\partial \mathbf{a}_{ji}}{\partial \mathbf{a}_{j}}\end{bmatrix}$$
  - $$\begin{bmatrix}(\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}})^{T} \frac{\partial r}{\partial \mathbf{C}} \\ (\frac{\partial r}{\partial \mathbf{a}_{i}})^{T}\frac{\partial r}{\partial \mathbf{C}} \end{bmatrix} = \begin{bmatrix} (\frac{\partial \boldsymbol{\xi}_{ji}}{\partial \boldsymbol{\xi}_{iw}})^{T} & \\ & (\frac{\partial \mathbf{a}_{ji}}{\partial \mathbf{a}_{i}})^{T}\end{bmatrix} \begin{bmatrix}(\frac{\partial r}{\partial  \boldsymbol{\xi}_{ji}})^{T} \frac{\partial r}{\partial  \mathbf{C}} \\ (\frac{\partial r}{\partial \mathbf{a}_{ji}})^{T} \frac{\partial r}{\partial  \mathbf{C}} \end{bmatrix}$$
  - $$\begin{bmatrix}r(\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}})^{T} \\ r(\frac{\partial r}{\partial \mathbf{a}_{i}})^{T} \end{bmatrix} = \begin{bmatrix} (\frac{\partial \boldsymbol{\xi}_{ji}}{\partial \boldsymbol{\xi}_{iw}})^{T} & \\ & (\frac{\partial \mathbf{a}_{ji}}{\partial \mathbf{a}_{i}})^{T}\end{bmatrix}\begin{bmatrix}r(\frac{\partial r}{\partial \boldsymbol{\xi}_{ji}})^{T} \\ r(\frac{\partial r}{\partial \mathbf{a}_{ji}})^{T} \end{bmatrix}$$
  
#### void stitchDoubleMT()

```c++
void stitchDoubleMT(IndexThreadReduce<Vec10>* red, MatXX& H, VecX& b,
                    const EnergyFunctional* const EF, const bool usePrior,
                    const bool MT);
```

- Basically, this matrix computes [$$\mathbf{H}_{11}$$](http://www.lingtong.de/2020/04/17/DSO-Schur-Complement/#hessian) and [$$\mathbf{b}_{1}$$](http://www.lingtong.de/2020/04/17/DSO-Schur-Complement/#b). Most of work will be done in `void stitchDoubleInternal()`. After that, this function will copy some existing elements over diagonal (due to the symmetric property).