---
layout:     post
title:      "G2O - Nitty Gritty"
date:       2020-4-10
author:     Tong
catalog: true
tags:
    - SLAM
---

> [Github](https://github.com/RainerKuemmerle/g2o)

Because `g2o` updates pose / rotation using [__left multiplication__](https://github.com/raulmur/ORB_SLAM2/blob/master/Thirdparty/g2o/g2o/types/types_six_dof_expmap.h#L75) under Lie groups, we need to use the model of __left perturbations__ to calculate related Jacobians. Examples can be found [here](http://tongling916.github.io/2020/04/02/Jacobian-Matrices/).