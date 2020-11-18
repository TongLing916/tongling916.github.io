---
layout:     post
title:      "VIO - Preintegration"
date:       2020-10-27
author:     Tong
catalog: true
tags:
    - VIO
---

### Motivation[^Forster16]

Once we have IMU measurements, we can integrate them to estimate the __orientation__, __velocity__, and __position__. The IMU measurements can be represented as 

$$
\tilde{\boldsymbol{\omega}}^{\text{b}} = \boldsymbol{\omega}^{\text{b}} + \mathbf{b}^{\text{g}} + \mathbf{n}^{\text{g}}, 
$$

$$
\tilde{\mathbf{a}}^{\text{b}} = \mathbf{q}_{\text{bw}}(\mathbf{a}^{\text{w}} + \mathbf{g}^{\text{w}}) + \mathbf{b}^{\text{a}} + \mathbf{n}^{\text{a}}.
$$

Ignoring the noises and biases (i.e., considering only nominal state[^Sola17]), we have 

$$
\mathbf{a}^{\text{b}} = \mathbf{q}_{\text{bw}}(\mathbf{a}^{\text{w}} + \mathbf{g}^{\text{w}})
$$

$$
\mathbf{a}^{\text{w}} = \mathbf{q}_{\text{wb}}\mathbf{a}^{\text{b}} - \mathbf{g}^{\text{w}}
$$

With the measurements from $$i$$ to $$j$$ (elapsed time: $$\Delta t$$), the orientation $$\mathbf{q}_{\text{wb}}$$, velocity $$\mathbf{v}^{\text{w}}$$, and position $$\mathbf{p}_{\text{wb}}$$ are integrated continuously using 

$$
\mathbf{q}_{\text{wb}_{j}} = \int_{t \in [i, j]} (\mathbf{q}_{\text{wb}_{t}} \otimes \begin{bmatrix}
    0 \\ \frac{1}{2} \boldsymbol{\omega}^{\text{b}_{t}}
\end{bmatrix}) dt
$$

$$
\begin{aligned}
    \mathbf{v}^{\text{w}}_{j} &= \mathbf{v}^{\text{w}}_{i} + \int_{t \in [i, j]} \mathbf{a}^{\text{w}_{t}} dt \\
    &= \mathbf{v}^{\text{w}}_{i} + \int_{t \in [i, j]} (\mathbf{q}_{\text{wb}_{t}}\mathbf{a}^{\text{b}_{t}} - \mathbf{g}^{\text{w}}) dt \\
    &= \mathbf{v}^{\text{w}}_{i} - \mathbf{g}^{\text{w}} \Delta t  + \int_{t \in [i, j]} (\mathbf{q}_{\text{wb}_{t}}\mathbf{a}^{\text{b}_{t}})dt 
\end{aligned}
$$

$$
\begin{aligned}
    \mathbf{p}_{\text{wb}_{j}} &= \mathbf{p}_{\text{wb}_{i}} + \mathbf{v}^{\text{w}}_{i} \Delta t + \int\int_{t \in [i, j]}(\mathbf{q}_{\text{wb}_{t}}\mathbf{a}^{\text{b}_{t}} - \mathbf{g}^{\text{w}}) dt^{2} \\
    &= \mathbf{p}_{\text{wb}_{i}} + \mathbf{v}^{\text{w}}_{i} \Delta t - \frac{1}{2} \mathbf{g}^{\text{w}} \Delta t^{2} + \int\int_{t \in [i, j]}(\mathbf{q}_{\text{wb}_{t}}\mathbf{a}^{\text{b}_{t}}) dt^{2}
\end{aligned}
$$

However, the __problem__ in the above integration is that every time we update the orientation $$\mathbf{q}_{\text{wb}_{t}}$$, the velocity $$\mathbf{v}^{\text{w}}_{\text{j}}$$ and position $$\mathbf{p}_{\text{wb}}$$ will be affected. That means that we need to reintegrate the velocity and position, which is time-consuming.

To overcome this problem, we can reformulate the above integration into __preintegration__ with a very simple formula, i.e., 

$$
\color{red}{\mathbf{q}_{\text{wb}_{t}} = \mathbf{q}_{\text{wb}_{i}} \otimes \mathbf{q}_{\text{b}_{i} \text{b}_{t}}}.
$$

__Why is this formula useful and effective?__

Our previous integrations are converted into the following ones, 

$$
\begin{aligned}
    \mathbf{q}_{\text{wb}_{j}} &= \int_{t \in [i, j]} (\mathbf{q}_{\text{wb}_{t}} \otimes 
    \begin{bmatrix}
        0 \\ \frac{1}{2} \boldsymbol{\omega}^{\text{b}_{t}}
    \end{bmatrix}) dt \\
    &= \int_{t \in [i, j]} (\mathbf{q}_{\text{wb}_{i}} \otimes \mathbf{q}_{\text{b}_{i} \text{b}_{t}} \otimes 
    \begin{bmatrix}
        0 \\ \frac{1}{2} \boldsymbol{\omega}^{\text{b}_{t}}
    \end{bmatrix}) dt \\
    &= \mathbf{q}_{\text{wb}_{i}} \otimes \int_{t \in [i, j]} (\mathbf{q}_{\text{b}_{i} \text{b}_{t}} \otimes 
    \begin{bmatrix}
        0 \\ \frac{1}{2} \boldsymbol{\omega}^{\text{b}_{t}}
    \end{bmatrix}) dt \\
    &= \mathbf{q}_{\text{wb}_{i}} \otimes \color{red}{\mathbf{q}_{\text{b}_{i} \text{b}_{j}}}
\end{aligned}
$$

$$
\begin{aligned}
    \mathbf{v}^{\text{w}}_{j} &= \mathbf{v}^{\text{w}}_{i} - \mathbf{g}^{\text{w}} \Delta t  + \int_{t \in [i, j]} (\mathbf{q}_{\text{wb}_{t}}\mathbf{a}^{\text{b}_{t}})dt \\
    &= \mathbf{v}^{\text{w}}_{i} - \mathbf{g}^{\text{w}} \Delta t  + \int_{t \in [i, j]} (\mathbf{q}_{\text{wb}_{i}} \otimes \mathbf{q}_{\text{b}_{i} \text{b}_{t}}\mathbf{a}^{\text{b}_{t}})dt \\
    &= \mathbf{v}^{\text{w}}_{i} - \mathbf{g}^{\text{w}} \Delta t  + \mathbf{q}_{\text{wb}_{i}} \int_{t \in [i, j]} (\mathbf{q}_{\text{b}_{i} \text{b}_{t}}\mathbf{a}^{\text{b}_{t}})dt \\
    &= \mathbf{v}^{\text{w}}_{i} - \mathbf{g}^{\text{w}} \Delta t  + \mathbf{q}_{\text{wb}_{i}} \color{red}{\boldsymbol{\beta}_{\text{b}_{i} \text{b}_{j}}}
\end{aligned}
$$

$$
\begin{aligned}
    \mathbf{p}_{\text{wb}_{j}} &= \mathbf{p}_{\text{wb}_{i}} + \mathbf{v}^{\text{w}}_{i} \Delta t - \frac{1}{2} \mathbf{g}^{\text{w}} \Delta t^{2} + \int\int_{t \in [i, j]}(\mathbf{q}_{\text{wb}_{t}}\mathbf{a}^{\text{b}_{t}}) dt^{2} \\
    &= \mathbf{p}_{\text{wb}_{i}} + \mathbf{v}^{\text{w}}_{i} \Delta t - \frac{1}{2} \mathbf{g}^{\text{w}} \Delta t^{2} + \int\int_{t \in [i, j]}(\mathbf{q}_{\text{wb}_{i}} \otimes \mathbf{q}_{\text{b}_{i} \text{b}_{t}}\mathbf{a}^{\text{b}_{t}}) dt^{2} \\
    &= \mathbf{p}_{\text{wb}_{i}} + \mathbf{v}^{\text{w}}_{i} \Delta t - \frac{1}{2} \mathbf{g}^{\text{w}} \Delta t^{2} + \mathbf{q}_{\text{wb}_{i}} \int\int_{t \in [i, j]}(\mathbf{q}_{\text{b}_{i} \text{b}_{t}}\mathbf{a}^{\text{b}_{t}}) dt^{2} \\
    &= \mathbf{p}_{\text{wb}_{i}} + \mathbf{v}^{\text{w}}_{i} \Delta t - \frac{1}{2} \mathbf{g}^{\text{w}} \Delta t^{2} + \mathbf{q}_{\text{wb}_{i}} \color{red}{\boldsymbol{\alpha}_{\text{b}_{i} \text{b}_{j}}}
\end{aligned}
$$

The preintegration $$\mathbf{q}_{\text{b}_{i} \text{b}_{j}}$$, $$\boldsymbol{\beta}_{\text{b}_{i} \text{b}_{j}}$$, and $$\boldsymbol{\alpha}_{\text{b}_{i} \text{b}_{j}}$$ are only related to IMU measurements, which __do not change__ with the update of orientation.

As a summary, we have

$$
\begin{bmatrix}
    \mathbf{p}_{\text{wb}_{j}} \\
    \mathbf{q}_{\text{wb}_{j}} \\
    \mathbf{v}^{\text{w}}_{j} \\
    \mathbf{b}^{\text{a}}_{j} \\
    \mathbf{b}^{\text{g}}_{j}
\end{bmatrix} =
\begin{bmatrix}
    \mathbf{p}_{\text{wb}_{i}} + \mathbf{v}^{\text{w}}_{i} \Delta t - \frac{1}{2} \mathbf{g}^{\text{w}} \Delta t^{2} + \mathbf{q}_{\text{wb}_{i}} \color{red}{\boldsymbol{\alpha}_{\text{b}_{i} \text{b}_{j}}} \\ 
    \mathbf{q}_{\text{wb}_{i}} \otimes \color{red}{\mathbf{q}_{\text{b}_{i} \text{b}_{j}}} \\
    \mathbf{v}^{\text{w}}_{i} - \mathbf{g}^{\text{w}} \Delta t  + \mathbf{q}_{\text{wb}_{i}} \color{red}{\boldsymbol{\beta}_{\text{b}_{i} \text{b}_{j}}} \\
    \mathbf{b}^{\text{a}}_{i} \\
    \mathbf{b}^{\text{g}}_{i}
\end{bmatrix}
$$

Besides, the preintegration residual can be represented by 
$$
\mathbf{r}_{\mathcal{B}}(\mathbf{x}_{i}, \mathbf{x}_{j})= 
\begin{bmatrix}
   \mathbf{r}_{\text{p}} \\
   \mathbf{r}_{\text{q}} \\
   \mathbf{r}_{\text{v}} \\
   \mathbf{r}_{\text{ba}} \\
   \mathbf{r}_{\text{bg}} 
\end{bmatrix} = 
\begin{bmatrix}
   \mathbf{q}_{\text{b}_{i}\text{w}}(\mathbf{p}_{\text{wb}_{j}} - \mathbf{p}_{\text{wb}_{i}} - \mathbf{v}_{\text{wb}_{i}} \Delta t + \frac{1}{2}\mathbf{g}^{\text{w}}\Delta t^{2}) - \hat{\boldsymbol{\alpha}}_{\text{b}_{i}\text{b}_{j}} \\
   2 [\hat{\mathbf{q}}_{\text{b}_{j}\text{b}_{i}} \otimes (\mathbf{q}_{\text{b}_{i}\text{w}} \otimes \mathbf{q}_{\text{wb}_{j}})]_{\text{xyz}} \\ 
   \mathbf{q}_{\text{b}_{i}\text{w}} (\mathbf{v}_{\text{wb}_{j}} - \mathbf{v}_{\text{wb}_{i}} + \mathbf{g}^{\text{w}} \Delta t) - \hat{\boldsymbol{\beta}}_{\text{b}_{i}\text{b}_{j}} \\
   \mathbf{b}^{\text{a}}_{j} - \mathbf{b}^{\text{a}}_{i} \\
   \mathbf{b}^{\text{g}}_{j} - \mathbf{b}^{\text{g}}_{i}
\end{bmatrix}.
$$

### Mid-Propagation

Now, we are concerned about the computation of preintegration values. 

### Error Propagation 

### Discussion

1. How do we estimate the initial covariance?
2. When do we need preintegration? (VINS-Mono vs. MSCKF)

### Literature

[^Forster16]: Forster, Christian, et al. "On-Manifold Preintegration for Real-Time Visual--Inertial Odometry." IEEE Transactions on Robotics 33.1 (2016): 1-21.

[^Sola17]: Sola, Joan. "Quaternion kinematics for the error-state Kalman filter." arXiv preprint arXiv:1711.02508 (2017).