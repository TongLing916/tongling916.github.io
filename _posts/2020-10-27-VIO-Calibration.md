---
layout:     post
title:      "VIO - Calibration"
date:       2020-10-27
author:     Tong
catalog: true
tags:
    - VIO
---

> [Kalibr](https://github.com/ethz-asl/kalibr/wiki)

### Goal

1. Extrinsics
2. Time delay

### [IMU Noise](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model)

### [Degenerate Motion](http://udel.edu/~pgeneva/downloads/papers/r05.pdf)[^Yang19]

1. Pure Translation (No Rotation)
2. One-Axis Rotation
3. Constant Local Angular and Linear Velocities
4. Constant Local Angular Velocity and Global Linear Acceleration

### [Intrinsic Calibration](http://www.roboticsproceedings.org/rss16/p026.pdf)[^Yang20]

### [VINS Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)[^Qin17]

1. Why don't we estimate extrinsic parameter $$\mathbf{p}_{bc}$$?


### Literature

[^Yang19]: Yang, Yulin; Geneva, Patrick; Eckenhoff, Kevin; Huang, Guoquan. "Degenerate Motion Analysis for Aided INS with Online Spatial and Temporal Sensor Calibration." 2019 RAL.

[^Yang20]: Yang, Yulin, et al. "Online IMU Intrinsic Calibration: Is It Necessary?." 2020 RSS.

[^Qin17]: Qin, Tong, and Shaojie Shen. "Robust initialization of monocular visual-inertial estimation on aerial robots." 2017 IROS.
