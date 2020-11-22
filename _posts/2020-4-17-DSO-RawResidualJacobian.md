---
layout:     post
title:      "DSO - RawResidualJacobian"
date:       2020-4-17
author:     Tong
catalog: true
tags:
    - DSO
---

> [Direct sparse odometry](https://vision.in.tum.de/research/vslam/dso)

| Variable used in `RawResidualJacobian` | Variable used here                                                                                                                                                                 | Meaning                                             |
| -------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------- |
| `resF`                                 |                                                                                                                                                                                    |                                                     |
| `Jpdxi`                                | $$\frac{\partial\mathbf{p}_j}{\partial\boldsymbol{\xi_{ji}}}$$                                                                                                                     | derivative of pixel `j` wrt. `relative pose`        |
| `Jpdc`                                 | $$\frac{\partial\mathbf{p}_j}{\partial\begin{bmatrix} f_{x} \\ f_{y} \\ c_{x} \\ c_{y}\end{bmatrix}} = \frac{\partial r_{ji}}{\partial \mathbf{C}}$$                               | derivative of pixel `j` wrt. `intrinsic parameters` |
| `Jpdd`                                 | $$\frac{\partial\mathbf{p}_j}{\partial \rho_{i}}$$                                                                                                                                 | derivative of pixel `j` wrt. `inverse depth i`      |
| `JIdx`                                 | $$\frac{\partial r_{ji}}{\partial\mathbf{p}_j}$$                                                                                                                                   | derivative of residual wrt. pixel `j`               |
| `JabF`                                 | $$\frac{\partial r_{ji}}{\partial \begin{bmatrix} -e^{a_{ji}} \\ -b_{ji}\end{bmatrix}} = \frac{\partial r_{ji}}{\partial \mathbf{a}_{ji}}$$                                        | derivative of residual wrt. `photometric parameters` |
| `JIdx2`                                | $$(\frac{\partial r_{ji}}{\partial\mathbf{p}_j})^{T}\frac{\partial r_{ji}}{\partial\mathbf{p}_j}$$                                                                                 |                                                     |
| `JabJIdx`                              | $$(\frac{\partial r_{ji}}{\partial \begin{bmatrix} -e^{a_{ji}} \\ -b_{ji}\end{bmatrix}})^{T}\frac{\partial r_{ji}}{\partial\mathbf{p}_j}$$                                         |                                                     |
| `Jab2`                                 | $$(\frac{\partial r_{ji}}{\partial \begin{bmatrix} -e^{a_{ji}} \\ -b_{ji}\end{bmatrix}})^{T}\frac{\partial r_{ji}}{\partial \begin{bmatrix} -e^{a_{ji}} \\ -b_{ji}\end{bmatrix}}$$ |                                                     |
