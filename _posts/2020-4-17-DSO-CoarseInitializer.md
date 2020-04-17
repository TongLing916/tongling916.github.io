---
layout:     post
title:      "DSO - CoraseInitializer"
date:       2020-4-17
author:     Tong
catalog: true
tags:
    - DSO
---

> [Direct sparse odometry](https://vision.in.tum.de/research/vslam/dso)

| Variable used in `CoarseInitializer` | Variable used here                                                            | Meaning                                       |
| - | - | - |
| `residual`                           | $$I_{j}\left[\mathbf{p}_{j}\right]-e^{a_{ji}}I_{i}[\mathbf{p}_{i}] - b_{ji}$$ | single residual                               |
| `hw`                                 | $$\omega_{h}$$                                                                | Huber weight                                  |
| `refToNew_aff_current`               | $$\begin{bmatrix}a_{ji} \\ b_{ji}\end{bmatrix}$$                              | photometric affine parameters                 |
| `r2new_aff`                          | $$\begin{bmatrix}e^{a_{ji}} \\ b_{ji}\end{bmatrix}$$                          | photometric affine parameters                 |
| `dp0`                                | $$\frac{\partial r_{ji}}{\partial t_{ji}^x}$$                                 | derivative wrt. `translation`                 |
| `dp1`                                | $$\frac{\partial r_{ji}}{\partial t_{ji}^y}$$                                 | derivative wrt. `translation`                 |
| `dp2`                                | $$\frac{\partial r_{ji}}{\partial t_{ji}^z}$$                                 | derivative wrt. `translation`                 |
| `dp3`                                | $$\frac{\partial r_{ji}}{\partial \phi_{ji}^x}$$                              | derivative wrt. `rotation`                    |
| `dp4`                                | $$\frac{\partial r_{ji}}{\partial \phi_{ji}^y}$$                              | derivative wrt. `rotation`                    |
| `dp5`                                | $$\frac{\partial r_{ji}}{\partial \phi_{ji}^z}$$                              | derivative wrt. `rotation`                    |
| `dp6`                                | $$\frac{\partial r_{ji}}{\partial a_{ji}}$$                                   | derivative wrt. photometric affine parameters |
| `dp7`                                | $$\frac{\partial r_{ji}}{\partial b_{ji}}$$                                   | derivative wrt. photometric affine parameters |
| `dd`                                 | $$\frac{\partial r_{ji}}{\partial \rho_{i}}$$                                 | derivative wrt. inverse depth i               |
| `r`                                  | $$r_{ji}$$                                                                    | single residual after applying Huber kernel   |