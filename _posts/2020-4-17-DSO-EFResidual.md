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

| Variable | Formula | Description
| - | - | -|
| `JpJdF` | $$\begin{bmatrix} (\frac{\partial r_{ji}}{\partial \boldsymbol{\xi}_{ji}})^T\frac{\partial r_{ji}}{\partial \rho_{i}} \\ (\frac{\partial r_{ji}}{\partial \begin{bmatrix} -e^{a_{ji}} \\ -b_{ji}\end{bmatrix}})^{T}\frac{\partial r_{ji}}{\partial \rho_{i}} \end{bmatrix}$$ | |