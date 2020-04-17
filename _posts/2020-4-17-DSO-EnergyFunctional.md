---
layout:     post
title:      "DSO - EnergyFunctional"
date:       2020-4-17
author:     Tong
catalog: true
tags:
    - DSO
---

> [Direct sparse odometry](https://vision.in.tum.de/research/vslam/dso)

| Variable | Formula | Description
| - | - | -|
| `adHost` | $$\begin{bmatrix} (\frac{\partial \boldsymbol{\xi}_{ji}}{\partial \boldsymbol{\xi}_{iw}})^{T} & \\ & (\frac{\partial \begin{bmatrix} -e^{a_{ji}} \\ -b_{ji}\end{bmatrix}}{\partial \mathbf{a}_{i}})^{T}\end{bmatrix}$$ | |
| `adTarget` | $$\begin{bmatrix} (\frac{\partial \boldsymbol{\xi}_{ji}}{\partial \boldsymbol{\xi}_{jw}})^{T} & \\ & (\frac{\partial \begin{bmatrix} -e^{a_{ji}} \\ -b_{ji}\end{bmatrix}}{\partial \mathbf{a}_{j}})^{T}\end{bmatrix}$$ | |