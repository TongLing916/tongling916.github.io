---
layout:     post
title:      "Kalman Filter"
date:       2019-8-24
author:     Tong
catalog: true
tags:
    - SLAM
---

### Kalman Filter

#### Introduction

$$
\begin{array}{lll}{1 :} & {\text { Algorithm Kalman filter }\left(\mu_{t-1}, u_{t}, z_{t}\right) :} \\ {2 :} & {\overline{\mu}_{t}=A_{t} \mu_{t-1}+B_{t} u_{t}} \\ {\text { 3: }} & {\overline{\Sigma}_{t}=A_{t} \Sigma_{t-1} A_{t}^{T}+R_{t}} \\ {4 :} & {K_{t}=\overline{\Sigma}_{t} C_{t}^{T}\left(C_{t} \overline{\Sigma}_{t} C_{t}^{T}+Q_{t}\right)^{-1}} \\ {5 :} & {\mu_{t}=\overline{\mu}_{t}+K_{t}\left(z_{t}-C_{t} \overline{\mu}_{t}\right)} \\ {6 :} & {\Sigma_{t}=\left(I-K_{t} C_{t}\right) \overline{\Sigma}_{t}} \\ {7} & {\text { return } \mu_{t}, \Sigma_{t}}\end{array}
$$

#### Implementation (Python)

#### Implementation (C++)


### Extended Kalman Filter

#### Introduction

#### Implementation (Python)

#### Implementation (C++)
