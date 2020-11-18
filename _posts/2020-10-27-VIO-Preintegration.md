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
\tilde{\boldsymbol{\omega}}^{\text{b}} = \boldsymbol{\omega}^{\text{b}} + \mathbf{b}^{\text{g}} + \mathbf{n}_{\mathbf{b}^\text{g}}, 
$$

$$
\tilde{\mathbf{a}}^{\text{b}} = \mathbf{q}_{\text{bw}}(\mathbf{a}^{\text{w}} + \mathbf{g}^{\text{w}}) + \mathbf{b}^{\text{a}} + \mathbf{n}_{\mathbf{b}^\text{a}}.
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

__Why is this equation useful and effective?__

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
\end{aligned},
$$

where 

$$
\mathbf{q}_{\text{b}_{i} \text{b}_{j}} = \int_{t \in [i, j]} (\mathbf{q}_{\text{b}_{i} \text{b}_{t}} \otimes 
    \begin{bmatrix}
        0 \\ \frac{1}{2} \boldsymbol{\omega}^{\text{b}_{t}}
    \end{bmatrix}) dt
$$

$$
\boldsymbol{\beta}_{\text{b}_{i} \text{b}_{j}} = \int_{t \in [i, j]} (\mathbf{q}_{\text{b}_{i} \text{b}_{t}}\mathbf{a}^{\text{b}_{t}})dt
$$

$$
\boldsymbol{\alpha}_{\text{b}_{i} \text{b}_{j}} = \int\int_{t \in [i, j]}(\mathbf{q}_{\text{b}_{i} \text{b}_{t}}\mathbf{a}^{\text{b}_{t}}) dt^{2}
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

Besides, the [preintegration residual](https://tongling916.github.io/2020/10/26/VIO-Optimization/#inertial-residual) can be represented by 
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
   \mathbf{q}_{\text{b}_{i}\text{w}}(\mathbf{p}_{\text{wb}_{j}} - \mathbf{p}_{\text{wb}_{i}} - \mathbf{v}_{\text{wb}_{i}} \Delta t + \frac{1}{2}\mathbf{g}^{\text{w}}\Delta t^{2}) - \color{red}{\hat{\boldsymbol{\alpha}}_{\text{b}_{i}\text{b}_{j}}} \\
   2 [\color{red}{\hat{\mathbf{q}}_{\text{b}_{j}\text{b}_{i}}} \otimes (\mathbf{q}_{\text{b}_{i}\text{w}} \otimes \mathbf{q}_{\text{wb}_{j}})]_{\text{xyz}} \\ 
   \mathbf{q}_{\text{b}_{i}\text{w}} (\mathbf{v}_{\text{wb}_{j}} - \mathbf{v}_{\text{wb}_{i}} + \mathbf{g}^{\text{w}} \Delta t) - \color{red}{\hat{\boldsymbol{\beta}}_{\text{b}_{i}\text{b}_{j}}} \\
   \mathbf{b}^{\text{a}}_{j} - \mathbf{b}^{\text{a}}_{i} \\
   \mathbf{b}^{\text{g}}_{j} - \mathbf{b}^{\text{g}}_{i}
\end{bmatrix}.
$$

### Preintegration

Now, we are concerned about the computation of preintegration values. Due to the fact that we can only obtain discrete IMU measurements, we need to find a way to propagte measurements from timestamp $$i$$ to $$j$$ (Note that there could be multiple measurements between $$i$$ and $$j$$).

The authors of VINS-Mono[^Qin17] use mid-point propagation[^Sola17] for the computation.

For example, assumed that we need to propagate IMU measurements from $$k$$ to $$k+1$$ (elapsed time: $$\delta t$$) into preintegration, we compute the values as follows,

$$
\begin{aligned}
    \bar{\boldsymbol{\omega}}^{\text{b}_{k}} &= \frac{1}{2}(\boldsymbol{\omega}^{\text{b}_{k}} + \boldsymbol{\omega}^{\text{b}_{k+1}}) \\
    &= \frac{1}{2}((\tilde{\boldsymbol{\omega}}
    ^{\text{b}_{k}} - \mathbf{b}^{\text{g}}_{k}) + (\tilde{\boldsymbol{\omega}}
    ^{\text{b}_{k+1}} - \mathbf{b}^{\text{g}}_{k})) \\
    &= \frac{1}{2}(\tilde{\boldsymbol{\omega}}
    ^{\text{b}_{k}} + \tilde{\boldsymbol{\omega}}
    ^{\text{b}_{k+1}}) - \mathbf{b}^{\text{g}}_{k}
\end{aligned}
$$

$$
\color{red}{\mathbf{q}_{\text{b}_{i}\text{b}_{k+1}}} = \mathbf{q}_{\text{b}_{i}\text{b}_{k}} \otimes \begin{bmatrix}
    1 \\ \frac{1}{2}\bar{\boldsymbol{\omega}}^{\text{b}_{k}} \delta t
\end{bmatrix}
$$

$$
\begin{aligned}
    \bar{\mathbf{a}}^{\text{b}_{i}} &= \frac{1}{2}(\mathbf{q}_{\text{b}_{i}\text{b}_{k}}\mathbf{a}^{\text{b}_{k}} + \mathbf{q}_{\text{b}_{i}\text{b}_{k+1}}\mathbf{a}^{\text{b}_{k+1}}) \\
    &= \frac{1}{2}(\mathbf{q}_{\text{b}_{i}\text{b}_{k}}(\tilde{\mathbf{a}}^{\text{b}_{k}} - \mathbf{b}^{\text{a}}_{k}) + \mathbf{q}_{\text{b}_{i}\text{b}_{k+1}}(\tilde{\mathbf{a}}^{\text{b}_{k+1}} - \mathbf{b}^{\text{a}}_{k}))
\end{aligned}
$$

$$
\color{red}{\boldsymbol{\beta}_{\text{b}_{i}\text{b}_{k+1}}} = \boldsymbol{\beta}_{\text{b}_{i}\text{b}_{k}} + \bar{\mathbf{a}}^{\text{b}_{i}} \delta t
$$

$$
\color{red}{\boldsymbol{\alpha}_{\text{b}_{i}\text{b}_{k+1}}} = \boldsymbol{\alpha}_{\text{b}_{i}\text{b}_{k}} +\boldsymbol{\beta}_{\text{b}_{i}\text{b}_{k}} \delta t + \frac{1}{2}\bar{\mathbf{a}}^{\text{b}_{i}} \delta t^2
$$

Note that although there are probably multiple IMU measurements between $$i$$ and $$j$$, we actually __always__ use $$\mathbf{b}^{\text{a}}_{i}$$ and $$\mathbf{b}^{\text{g}}_{i}$$ to propagate the measurements. 

### Error Propagation

__Where does error come from?__
1. Last state. (Error in last state is propagated into the current state)
2. Noise. (Measurement noise, random walk noise in IMU)

__Why do we need to propagte error?__
1. Computation of covariance, whose inverse is used as information matrix of preintegration residual in optimization. 
2. Computation of preintegration residual. Because we also optimize biases, the change of biases could also affect the IMU preintegration values $$\hat{\boldsymbol{\alpha}}_{\text{b}_{i}\text{b}_{j}}$$, $$\hat{\mathbf{q}}_{\text{b}_{j}\text{b}_{i}}$$, and $$\hat{\boldsymbol{\beta}}_{\text{b}_{i}\text{b}_{j}}$$. 

__How do we propagte error?__

Assumed that we have
$$\dot{\delta \mathbf{x}} = \mathbf{A}\delta\mathbf{x} + \mathbf{B}\mathbf{n},$$ 
we have then 

$$
\begin{aligned}
    \delta \mathbf{x}_{k+1} &= \delta \mathbf{x}_{k} + \dot{\delta \mathbf{x}}_{k} \delta t \\
    &= \delta \mathbf{x}_{k} + (\mathbf{A}_{k}\delta\mathbf{x}_{k} + \mathbf{B}_{k}\mathbf{n}_{k}) \delta t \\
    &= (\mathbf{I} + \mathbf{A}_{k} \delta t)\delta\mathbf{x}_{k} + (\mathbf{B}_{k}\delta t) \mathbf{n}_{k} \\
    &= \mathbf{F}_{k}\delta\mathbf{x}_{k} + \mathbf{G}_{k}\mathbf{n}_{k}
\end{aligned}
$$

Therefore, the jacobian and covariance propagation are 

$$
\delta \mathbf{x}_{k+2} = \color{red}{(\mathbf{F}_{k+1}\mathbf{F}_{k})}\delta\mathbf{x}_{k} + \dots
$$

$$
\boldsymbol{\Sigma}_{k+1} = \mathbf{F}_{k}\boldsymbol{\Sigma}_{k} \mathbf{F}_{k}^{\text{T}} + \mathbf{G}_{k}\boldsymbol{\Sigma}_{\text{n}}\mathbf{G}_{k}^{\text{T}}
$$

As a summary, we have

$$
\begin{bmatrix}
    \delta\boldsymbol{\alpha}_{\text{b}_{i}\text{b}_{k+1}} \\
    \delta\boldsymbol{\theta}_{\text{b}_{i}\text{b}_{k+1}} \\
    \delta\boldsymbol{\beta}_{\text{b}_{i}\text{b}_{k+1}} \\
    \delta\mathbf{b}^{\text{a}}_{k+1} \\
    \delta\mathbf{b}^{\text{g}}_{k+1}
\end{bmatrix} =
\mathbf{F}
\begin{bmatrix}
    \delta\boldsymbol{\alpha}_{\text{b}_{i}\text{b}_{k}} \\
    \delta\boldsymbol{\theta}_{\text{b}_{i}\text{b}_{k}} \\
    \delta\boldsymbol{\beta}_{\text{b}_{i}\text{b}_{k}} \\
    \delta\mathbf{b}^{\text{a}}_{k} \\
    \delta\mathbf{b}^{\text{g}}_{k}
\end{bmatrix} +
\mathbf{G}
\begin{bmatrix}
    \mathbf{n}^{\text{a}}_{k} \\
    \mathbf{n}^{\text{g}}_{k} \\
    \mathbf{n}^{\text{a}}_{k+1} \\
    \mathbf{n}^{\text{g}}_{k+1} \\
    \mathbf{n}_{\mathbf{b}^\text{a}_{k}} \\
    \mathbf{n}_{\mathbf{b}^\text{g}_{k}}
\end{bmatrix},
$$

where 

$$
\mathbf{F}_{15\times15} =
\begin{bmatrix}
    \mathbf{f}_{11} & \mathbf{f}_{12} & \mathbf{f}_{13} & \mathbf{f}_{14} & \mathbf{f}_{15} \\
    \mathbf{0} & \mathbf{f}_{22} & \mathbf{0} & \mathbf{0} & \mathbf{f}_{25} \\
    \mathbf{0} & \mathbf{f}_{32} & \mathbf{f}_{33} & \mathbf{f}_{34} & \mathbf{f}_{35} \\
    \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{f}_{44} & \mathbf{0} \\
    \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{f}_{55}
\end{bmatrix}
$$

$$
\begin{aligned}
    \mathbf{f}_{11} &= \frac{\partial \delta\boldsymbol{\alpha}_{\text{b}_{i}\text{b}_{k+1}}}{\partial \delta\boldsymbol{\alpha}_{\text{b}_{i}\text{b}_{k}}} = \mathbf{I}_{3}\\
    \mathbf{f}_{12} &= \frac{\partial \delta\boldsymbol{\alpha}_{\text{b}_{i}\text{b}_{k+1}}}{\partial \delta\boldsymbol{\theta}_{\text{b}_{i}\text{b}_{k}}} = -\frac{1}{4}(\mathbf{R}_{\text{b}_{i}\text{b}_{k}}[\tilde{\mathbf{a}}^{\text{b}_{k}} - \mathbf{b}^{\text{a}}_{k}]_{\times}\delta t^{2} + \mathbf{R}_{\text{b}_{i}\text{b}_{k+1}}[\tilde{\mathbf{a}}^{\text{b}_{k+1}} - \mathbf{b}^{\text{a}}_{k}]_{\times}(\mathbf{I}_{3} - [\bar{\boldsymbol{\omega}}^{\text{b}_{k}}]_{\times}\delta t)\delta t^{2})\\
    \mathbf{f}_{13} &= \frac{\partial \delta\boldsymbol{\alpha}_{\text{b}_{i}\text{b}_{k+1}}}{\partial \delta\boldsymbol{\beta}_{\text{b}_{i}\text{b}_{k}}} = \mathbf{I}_{3}\delta t\\
    \mathbf{f}_{14} &= \frac{\partial \delta\boldsymbol{\alpha}_{\text{b}_{i}\text{b}_{k+1}}}{\partial \delta\mathbf{b}^{\text{a}}_{k}} = -\frac{1}{4}(\mathbf{R}_{\text{b}_{i}\text{b}_{k}} + \mathbf{R}_{\text{b}_{i}\text{b}_{k+1}})\delta t^{2}\\
    \mathbf{f}_{15} &= \frac{\partial \delta\boldsymbol{\alpha}_{\text{b}_{i}\text{b}_{k+1}}}{\partial \delta\mathbf{b}^{\text{g}}_{k+1}} = -\frac{1}{4}(\mathbf{R}_{\text{b}_{i}\text{b}_{k+1}}[\tilde{\mathbf{a}}^{\text{b}_{k+1}} - \mathbf{b}^{\text{a}}_{k}]_{\times}\delta t^{2})(-\delta t)\\
    \mathbf{f}_{22} &= \frac{\partial \delta\boldsymbol{\theta}_{\text{b}_{i}\text{b}_{k+1}}}{\partial \delta\boldsymbol{\theta}_{\text{b}_{i}\text{b}_{k}}} = \mathbf{I}_{3} - [\bar{\boldsymbol{\omega}}^{\text{b}_{k}}]_{\times}\delta t\\
    \mathbf{f}_{25} &= \frac{\partial \delta\boldsymbol{\theta}_{\text{b}_{i}\text{b}_{k+1}}}{\partial \delta\mathbf{b}^{\text{g}}_{k+1}} = -\mathbf{I}_{3}\delta t\\
    \mathbf{f}_{32} &= \frac{\partial \delta\boldsymbol{\beta}_{\text{b}_{i}\text{b}_{k+1}}}{\partial \delta\boldsymbol{\theta}_{\text{b}_{i}\text{b}_{k}}} = -\frac{1}{2}(\mathbf{R}_{\text{b}_{i}\text{b}_{k}}[\tilde{\mathbf{a}}^{\text{b}_{k}} - \mathbf{b}^{\text{a}}_{k}]_{\times}\delta t + \mathbf{R}_{\text{b}_{i}\text{b}_{k+1}}[\tilde{\mathbf{a}}^{\text{b}_{k+1}} - \mathbf{b}^{\text{a}}_{k}]_{\times}(\mathbf{I}_{3} - [\bar{\boldsymbol{\omega}}^{\text{b}_{k}}]_{\times}\delta t)\delta t)\\
    \mathbf{f}_{33} &= \frac{\partial \delta\boldsymbol{\beta}_{\text{b}_{i}\text{b}_{k+1}}}{\partial \delta\boldsymbol{\beta}_{\text{b}_{i}\text{b}_{k}}} = \mathbf{I}_{3}\\
    \mathbf{f}_{34} &= \frac{\partial \delta\boldsymbol{\beta}_{\text{b}_{i}\text{b}_{k+1}}}{\partial \delta\mathbf{b}^{\text{a}}_{k}} = -\frac{1}{2}(\mathbf{R}_{\text{b}_{i}\text{b}_{k}} + \mathbf{R}_{\text{b}_{i}\text{b}_{k+1}})\delta t\\
    \mathbf{f}_{35} &= \frac{\partial \delta\boldsymbol{\beta}_{\text{b}_{i}\text{b}_{k+1}}}{\partial \delta\mathbf{b}^{\text{g}}_{k+1}} = -\frac{1}{2}(\mathbf{R}_{\text{b}_{i}\text{b}_{k+1}}[\tilde{\mathbf{a}}^{\text{b}_{k+1}} - \mathbf{b}^{\text{a}}_{k}]_{\times}\delta t)(-\delta t)\\
    \mathbf{f}_{44} &= \frac{\partial \delta\mathbf{b}^{\text{a}}_{k+1}}{\partial \delta\mathbf{b}^{\text{a}}_{k}} = \mathbf{I}_{3}\\
    \mathbf{f}_{55} &= \frac{\partial \delta\mathbf{b}^{\text{g}}_{k+1}}{\partial \delta\mathbf{b}^{\text{g}}_{k+1}} = \mathbf{I}_{3}
\end{aligned}
$$

$$
\mathbf{G}_{15\times18} = 
\begin{bmatrix}
    \mathbf{g}_{11} & \mathbf{g}_{12} & \mathbf{g}_{13} & \mathbf{g}_{14} & \mathbf{0} & \mathbf{0} \\
    \mathbf{0} & \mathbf{g}_{22} & \mathbf{0} & \mathbf{g}_{24} & \mathbf{0} & \mathbf{0} \\
    \mathbf{g}_{31} & \mathbf{g}_{32} & \mathbf{g}_{33} & \mathbf{g}_{34} & \mathbf{0} & \mathbf{0} \\
    \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{g}_{45} & \mathbf{0} \\
    \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{g}_{56}
\end{bmatrix}
$$

$$
\begin{aligned}
    \mathbf{g}_{11} &= \frac{\partial \delta\boldsymbol{\alpha}_{\text{b}_{i}\text{b}_{k+1}}}{\partial \mathbf{n}^{\text{a}}_{k}} = \frac{1}{4}\mathbf{R}_{\text{b}_{i}\text{b}_{k}}\delta t^{2}\\
    \mathbf{g}_{12} &= \frac{\partial \delta\boldsymbol{\alpha}_{\text{b}_{i}\text{b}_{k+1}}}{\partial \mathbf{n}^{\text{g}}_{k}} = -\frac{1}{4}(\mathbf{R}_{\text{b}_{i}\text{b}_{k+1}}[\tilde{\mathbf{a}}^{\text{b}_{k+1}} - \mathbf{b}^{\text{a}}_{k}]_{\times}\delta t^{2})(\frac{1}{2}\delta t)\\
    \mathbf{g}_{13} &= \frac{\partial \delta\boldsymbol{\alpha}_{\text{b}_{i}\text{b}_{k+1}}}{\partial \mathbf{n}^{\text{a}}_{k+1}} = \frac{1}{4}\mathbf{R}_{\text{b}_{i}\text{b}_{k+1}}\delta t^{2}\\
    \mathbf{g}_{14} &= \frac{\partial \delta\boldsymbol{\alpha}_{\text{b}_{i}\text{b}_{k+1}}}{\partial \mathbf{n}^{\text{g}}_{k+1}} = \mathbf{g}_{12}\\
    \mathbf{g}_{22} &= \frac{\partial \delta\boldsymbol{\theta}_{\text{b}_{i}\text{b}_{k+1}}}{\partial \mathbf{n}^{\text{g}}_{k}} = \frac{1}{2}\mathbf{I}_{3}\delta t\\
    \mathbf{g}_{24} &= \frac{\partial \delta\boldsymbol{\theta}_{\text{b}_{i}\text{b}_{k+1}}}{\partial \mathbf{n}^{\text{g}}_{k+1}} = \frac{1}{2}\mathbf{I}_{3}\delta t\\
    \mathbf{g}_{31} &= \frac{\partial \delta\boldsymbol{\beta}_{\text{b}_{i}\text{b}_{k+1}}}{\partial \mathbf{n}^{\text{a}}_{k}} = \frac{1}{2}\mathbf{R}_{\text{b}_{i}\text{b}_{k}}\delta t\\
    \mathbf{g}_{32} &= \frac{\partial \delta\boldsymbol{\beta}_{\text{b}_{i}\text{b}_{k+1}}}{\partial \mathbf{n}^{\text{g}}_{k}} = -\frac{1}{2}(\mathbf{R}_{\text{b}_{i}\text{b}_{k+1}}[\tilde{\mathbf{a}}^{\text{b}_{k+1}} - \mathbf{b}^{\text{a}}_{k}]_{\times}\delta t)(\frac{1}{2}\delta t)\\
    \mathbf{g}_{33} &= \frac{\partial \delta\boldsymbol{\beta}_{\text{b}_{i}\text{b}_{k+1}}}{\partial \mathbf{n}^{\text{a}}_{k+1}} = \frac{1}{2}\mathbf{R}_{\text{b}_{i}\text{b}_{k+1}}\delta t\\
    \mathbf{g}_{34} &= \frac{\partial \delta\boldsymbol{\beta}_{\text{b}_{i}\text{b}_{k+1}}}{\partial \mathbf{n}^{\text{g}}_{k+1}} = \mathbf{g}_{32}\\
    \mathbf{g}_{45} &= \frac{\partial \delta\mathbf{b}^{\text{a}}_{k+1}}{\partial \mathbf{n}_{\mathbf{b}^\text{a}_{k}}} = \mathbf{I}_{3}\delta t\\
    \mathbf{g}_{56} &= \frac{\partial \delta\mathbf{b}^{\text{g}}_{k+1}}{\partial \mathbf{n}_{\mathbf{b}^\text{g}_{k}}} = \mathbf{I}_{3}\delta t
\end{aligned}
$$

__How do we choose initialize $$F$$ and $$G$$?__

- $$F$$ is initialized as an identity.
- $$G$$ is a zero matrix.

__How does the update of biases affect preintegration values?__

During optimization, we update states (position, orientation, velocity, biases) after each iteration. That means that we need to compute a new residual at the new state using the formula

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
   \mathbf{q}_{\text{b}_{i}\text{w}}(\mathbf{p}_{\text{wb}_{j}} - \mathbf{p}_{\text{wb}_{i}} - \mathbf{v}_{\text{wb}_{i}} \Delta t + \frac{1}{2}\mathbf{g}^{\text{w}}\Delta t^{2}) - \color{red}{\hat{\boldsymbol{\alpha}}_{\text{b}_{i}\text{b}_{j}}} \\
   2 [\color{red}{\hat{\mathbf{q}}_{\text{b}_{j}\text{b}_{i}}} \otimes (\mathbf{q}_{\text{b}_{i}\text{w}} \otimes \mathbf{q}_{\text{wb}_{j}})]_{\text{xyz}} \\ 
   \mathbf{q}_{\text{b}_{i}\text{w}} (\mathbf{v}_{\text{wb}_{j}} - \mathbf{v}_{\text{wb}_{i}} + \mathbf{g}^{\text{w}} \Delta t) - \color{red}{\hat{\boldsymbol{\beta}}_{\text{b}_{i}\text{b}_{j}}} \\
   \mathbf{b}^{\text{a}}_{j} - \mathbf{b}^{\text{a}}_{i} \\
   \mathbf{b}^{\text{g}}_{j} - \mathbf{b}^{\text{g}}_{i}
\end{bmatrix}.
$$

However, the preintegrations are also affected by the update of biases (see [mid-point propagation](https://tongling916.github.io/2020/10/27/VIO-Preintegration/#preintegration)). To avoid computation, we can update these preintegrations using the following equations, i.e.,

$$
\begin{aligned}
\hat{\boldsymbol{\alpha}}^{\text{new}}_{\text{b}_{i}\text{b}_{j}} &= \hat{\boldsymbol{\alpha}}^{\text{old}}_{\text{b}_{i}\text{b}_{j}} + \delta \boldsymbol{\alpha}_{\text{b}_{i}\text{b}_{j}}\\ &= \hat{\boldsymbol{\alpha}}^{\text{old}}_{\text{b}_{i}\text{b}_{j}}+\frac{\partial \delta\boldsymbol{\alpha}_{\text{b}_{i}\text{b}_{j}}}{\partial \delta\mathbf{b}^{\text{a}}_{i}} \delta \mathbf{b}_{i}^{\text{a}}+\frac{\partial \delta\boldsymbol{\alpha}_{\text{b}_{i}\text{b}_{j}}}{\partial \delta\mathbf{b}^{\text{g}}_{i}} \delta \mathbf{b}_{i}^{\text{g}} 
\end{aligned}
$$

$$
\begin{aligned}
\hat{\boldsymbol{\beta}}^{\text{new}}_{\text{b}_{i}\text{b}_{j}} &= \hat{\boldsymbol{\beta}}^{\text{old}}_{\text{b}_{i}\text{b}_{j}} + \delta \boldsymbol{\beta}_{\text{b}_{i}\text{b}_{j}}\\ &= \hat{\boldsymbol{\beta}}^{\text{old}}_{\text{b}_{i}\text{b}_{j}}+\frac{\partial \delta\boldsymbol{\beta}_{\text{b}_{i}\text{b}_{j}}}{\partial \delta\mathbf{b}^{\text{a}}_{i}} \delta \mathbf{b}_{i}^{\text{a}}+\frac{\partial \delta\boldsymbol{\beta}_{\text{b}_{i}\text{b}_{j}}}{\partial \delta\mathbf{b}^{\text{g}}_{i}} \delta \mathbf{b}_{i}^{\text{g}} 
\end{aligned}
$$

$$
\begin{aligned}
\hat{\mathbf{q}}^{\text{new}}_{\text{b}_{i}\text{b}_{j}} &= \hat{\mathbf{q}}^{\text{old}}_{\text{b}_{i}\text{b}_{j}} \otimes \begin{bmatrix}
   1 \\ \frac{1}{2} \delta\boldsymbol{\theta}_{\text{b}_{i}\text{b}_{j}}
   \end{bmatrix} \\ &= \hat{\mathbf{q}}^{\text{old}}_{\text{b}_{i}\text{b}_{j}} \otimes 
   \begin{bmatrix}
        1 \\ \frac{1}{2} \frac{\partial \delta\boldsymbol{\theta}_{\text{b}_{i}\text{b}_{j}}}{\partial \delta\mathbf{b}^{\text{g}}_{i}} \delta \mathbf{b}^{\text{g}}_{i} 
    \end{bmatrix}
\end{aligned}
$$

### Discussion

1. When do we need preintegration? (VINS-Mono vs. MSCKF)

### Literature

[^Forster16]: Forster, Christian, et al. "On-Manifold Preintegration for Real-Time Visual--Inertial Odometry." IEEE Transactions on Robotics 33.1 (2016): 1-21.

[^Sola17]: Sola, Joan. "Quaternion kinematics for the error-state Kalman filter." arXiv preprint arXiv:1711.02508 (2017).

[^Qin17]: Qin, Tong, and Shaojie Shen. "Robust initialization of monocular visual-inertial estimation on aerial robots." 2017 IROS.
