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