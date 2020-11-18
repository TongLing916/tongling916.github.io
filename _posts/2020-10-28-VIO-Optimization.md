---
layout:     post
title:      "VIO - Optimization"
date:       2020-10-26
author:     Tong
catalog: true
tags:
    - VIO
---

### Convention

#### Basic 

$$
\boldsymbol{\phi}^{\wedge}=[\begin{array}{c}
\phi_{1} \\
\phi_{2} \\
\phi_{3}
\end{array}]^{\wedge}=[\begin{array}{ccc}
0 & -\phi_{3} & \phi_{2} \\
\phi_{3} & 0 & -\phi_{1} \\
-\phi_{2} & \phi_{1} & 0
\end{array}] \in \mathbb{R}^{3 \times 3}, \quad \boldsymbol{\phi} \in \mathbb{R}^{3}
$$

$$
\mathbf{a}^{\wedge} \mathbf{b} = -\mathbf{b}^{\wedge}\mathbf{a}
$$

#### Quaternion

$$
\mathbf{q} = 
\begin{bmatrix}
   q_w \\
   q_x \\
   q_y \\
   q_z
\end{bmatrix} = 
\begin{bmatrix}
   q_w \\
   \mathbf{q}_{v}
\end{bmatrix}
$$

$$
\mathbf{q}_{1} \otimes \mathbf{q}_{2} = 
[\mathbf{q}_{1}]_{\text{L}} \mathbf{q}_{2} = 
[\mathbf{q}_{2}]_{\text{R}} \mathbf{q}_{1}
$$

$$
\mathbf{q}_{1} \otimes \mathbf{x} \otimes \mathbf{q}_{2} = 
[\mathbf{q}_{1}]_{\text{L}} [\mathbf{q}_{2}]_{\text{R}}\mathbf{x} = 
[\mathbf{q}_{2}]_{\text{R}} [\mathbf{q}_{1}]_{\text{L}} \mathbf{x} 
$$

$$
[\mathbf{q}]_{\text{L}} = q_w \mathbf{I} +
\begin{bmatrix}
   0 & -\mathbf{q}_{v}^{\text{T}} \\
   \mathbf{q}_{v} & \mathbf{q}_{v}^{\wedge}
\end{bmatrix}
\qquad 
[\mathbf{q}]_{\text{R}} = q_w \mathbf{I} +
\begin{bmatrix}
   0 & -\mathbf{q}_{v}^{\text{T}} \\
   \mathbf{q}_{v} & -\mathbf{q}_{v}^{\wedge}
\end{bmatrix}
$$

#### Rotation Update

With a __small__ rotation increment $$\mathbf{\theta}\in \mathbb{R}^{3}$$, we can udpate the rotation as follows,

$$
\mathbf{R}_{\text{wb}} \leftarrow \mathbf{R}_{\text{wb}} \exp(\mathbf{\theta}^{\wedge}),
$$

$$
\mathbf{q}_{\text{wb}} \leftarrow \mathbf{q}_{\text{wb}} \otimes 
\begin{bmatrix}
   1 \\
   \frac{1}{2} \mathbf{\theta}   
\end{bmatrix}.
$$

__DO NOT FORGET TO NORMALIZE THE QUATERNION.__


Similarly, with an angular velocity $$\mathbf{\omega}\in \mathbb{R}^{3}$$ we have time derivatives of rotation as follows,

$$
\dot{\mathbf{R}}_{\text{wb}} = \mathbf{R}_{\text{wb}}\mathbf{\omega}^{\wedge},
$$

$$
\dot{\mathbf{q}}_{\text{wb}} = \mathbf{q}_{\text{wb}} \otimes 
\begin{bmatrix}
   0 \\
   \frac{1}{2} \mathbf{\omega}   
\end{bmatrix}.
$$

We can observe an implicit conversion between 
$$\exp(\mathbf{\theta}^{\wedge})$$
 and 
$$\begin{bmatrix}1 \\ \frac{1}{2} \mathbf{\theta}\end{bmatrix}$$
, and conversion between
$$\mathbf{\omega}^{\wedge}$$
 and 
$$\begin{bmatrix}
   0 \\
   \frac{1}{2} \mathbf{\omega}   
\end{bmatrix}$$
.


### Cost function[^Qin17]

$$
\begin{aligned} 
   \mathcal{X} &=\left[\mathbf{x}_{0}, \mathbf{x}_{1}, \ldots \mathbf{x}_{n}, \mathbf{x}_{\text{bc}}, \lambda_{0}, \lambda_{1}, \ldots \lambda_{m}\right] \\ 
   \mathbf{x}_{k} &=\left[\mathbf{p}_{\text{wb}_{k}}, \mathbf{v}_{\text{wb}_{k}}, \mathbf{q}_{\text{wb}_{k}}, \mathbf{b}^{\text{a}}_{k}, \mathbf{b}^{\text{g}}_{k}\right], k \in[0, n] \\ 
   \mathbf{x}_{\text{bc}} &=\left[\mathbf{p}_{\text{bc}}, \mathbf{q}_{\text{bc}}\right] 
\end{aligned}
$$

$$
\min_{\mathcal{X}}
\{
   \|\underbrace{\mathbf{r}_{p}-\mathbf{H}_{p} \mathcal{X}}_{\text{prior}} \|^{2}+
   \sum_{k \in \mathcal{B}}\|\underbrace{\mathbf{r}_{\mathcal{B}}(\hat{\mathbf{z}}_{b_{k+1}}^{b_{k}}, \mathcal{X})}_{\text{inertial}} \|_{\mathbf{P}_{b_{k+1}}^{b_{k}}}^{2}+
   \sum_{(l, j) \in \mathcal{C}} \rho(\| \underbrace{\mathbf{r}_{\mathcal{C}}(\hat{\mathbf{z}}_{l}^{c_{j}}, \mathcal{X})}_{\text{visual}} \|_{\mathbf{P}_{l}^{c_{j}}}^{2})
\}
$$

$$
\rho(s)=\left\{\begin{array}{ll}s & s \leq 1 \\ 2 \sqrt{s}-1 & s>1\end{array}\right.
$$

### Discussion

We need to correct preintegrated p, q, v using first-order approximation, namely, 
$$
\begin{aligned}
\boldsymbol{\alpha}_{b_{i} b_{j}} &=\boldsymbol{\alpha}_{b_{i} b_{j}}+\mathbf{J}_{b_{i}^{\text{a}}}^{\alpha} \delta \mathbf{b}_{i}^{a}+\mathbf{J}_{b_{i}^{g}}^{\alpha} \delta \mathbf{b}_{i}^{g} \\
\boldsymbol{\beta}_{b_{i} b_{j}} &=\boldsymbol{\beta}_{b_{i} b_{j}}+\mathbf{J}_{b_{i}^{\text{a}}}^{\beta} \delta \mathbf{b}_{i}^{a}+\mathbf{J}_{b_{i}^{g}}^{\beta} \delta \mathbf{b}_{i}^{g} \\
\mathbf{q}_{b_{i} b_{j}} &=\mathbf{q}_{b_{i} b_{j}} \otimes \begin{bmatrix}
   1 \\
\frac{1}{2} \mathbf{J}_{b_{i}^{g}}^{q} \delta \mathbf{b}_{i}^{g}
\end{bmatrix}
\end{aligned}.
$$

### Prior Residual

### Inertial Residual

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
\end{bmatrix},
$$
where 
$$\hat{\boldsymbol{\alpha}}_{\text{b}_{i}\text{b}_{j}},$$
$$\hat{\mathbf{q}}_{\text{b}_{j}\text{b}_{i}},$$
 and 
$$\hat{\boldsymbol{\beta}}_{\text{b}_{i}\text{b}_{j}}$$ 
are our preintegrated inertial measurements between two image frames $$i$$ and $$j$$.

#### Position 1

$$
\begin{aligned}
   \frac{\partial \mathbf{r}_{\text{p}}}{\partial \mathbf{p}_{\text{wb}_{i}}} &= \frac{\partial (\mathbf{q}_{\text{b}_{i}\text{w}}(\mathbf{p}_{\text{wb}_{j}} - \mathbf{p}_{\text{wb}_{i}} - \mathbf{v}_{\text{wb}_{i}} \Delta t + \frac{1}{2}\mathbf{g}^{\text{w}}\Delta t^{2}) - \hat{\boldsymbol{\alpha}}_{\text{b}_{i}\text{b}_{j}})}{\partial \mathbf{p}_{\text{wb}_{i}}} \\ &= -\mathbf{q}_{\text{b}_{i}\text{w}} \\ &= -\mathbf{R}_{\text{b}_{i}\text{w}} 
\end{aligned}
$$


#### Rotation 1

Define 
$$
\mathbf{C}_{1} = \mathbf{p}_{\text{wb}_{j}} - \mathbf{p}_{\text{wb}_{i}} - \mathbf{v}_{\text{wb}_{i}} \Delta t + \frac{1}{2}\mathbf{g}^{\text{w}}\Delta t^{2},
$$
we have
$$
\begin{aligned}
   \frac{\partial \mathbf{r}_{\text{p}}}{\partial \mathbf{q}_{\text{wb}_{i}}} &= \frac{\partial (\mathbf{q}_{\text{b}_{i}\text{w}}\mathbf{C}_{1} - \hat{\boldsymbol{\alpha}}_{\text{b}_{i}\text{b}_{j}})}{\partial \mathbf{q}_{\text{wb}_{i}}} \\ 
   &= \lim_{\delta \boldsymbol{\theta} \rightarrow 0} \frac{( (\mathbf{R}_{\text{wb}_{i}} \exp(\delta \boldsymbol{\theta}^{\wedge}))^{-1}\mathbf{C}_{1} - \hat{\boldsymbol{\alpha}}_{\text{b}_{i}\text{b}_{j}}) - (\mathbf{R}_{\text{b}_{i}\text{w}}\mathbf{C}_{1} - \hat{\boldsymbol{\alpha}}_{\text{b}_{i}\text{b}_{j}}) }{\delta \boldsymbol{\theta}} \\
   &= \lim_{\delta \boldsymbol{\theta} \rightarrow 0} \frac{\exp(-\delta \boldsymbol{\theta}^{\wedge})\mathbf{R}_{\text{b}_{i}\text{w}} \mathbf{C}_{1} - \mathbf{R}_{\text{b}_{i}\text{w}}\mathbf{C}_{1}}{\delta \boldsymbol{\theta}} \\
   &= \lim_{\delta \boldsymbol{\theta} \rightarrow 0} \frac{(\mathbf{I}-\delta \boldsymbol{\theta}^{\wedge})\mathbf{R}_{\text{b}_{i}\text{w}} \mathbf{C}_{1} - \mathbf{R}_{\text{b}_{i}\text{w}}\mathbf{C}_{1}}{\delta \boldsymbol{\theta}} \\
   &= \lim_{\delta \boldsymbol{\theta} \rightarrow 0} \frac{-\delta \boldsymbol{\theta}^{\wedge}\mathbf{R}_{\text{b}_{i}\text{w}} \mathbf{C}_{1}}{\delta \boldsymbol{\theta}} \\
   &= \lim_{\delta \boldsymbol{\theta} \rightarrow 0} \frac{(\mathbf{R}_{\text{b}_{i}\text{w}} \mathbf{C}_{1})^{\wedge}\delta \boldsymbol{\theta}}{\delta \boldsymbol{\theta}} \\
   &= (\mathbf{R}_{\text{b}_{i}\text{w}} \mathbf{C}_{1})^{\wedge}
\end{aligned}
$$

$$
\begin{aligned}
   \frac{\partial \mathbf{r}_{\text{q}}}{\partial \mathbf{q}_{\text{wb}_{i}}} &= \frac{\partial (2 [\hat{\mathbf{q}}_{\text{b}_{j}\text{b}_{i}} \otimes (\mathbf{q}_{\text{b}_{i}\text{w}} \otimes \mathbf{q}_{\text{wb}_{j}})]_{\text{xyz}})}{\partial \mathbf{q}_{\text{wb}_{i}}} \\ 
   &= \lim_{\delta \boldsymbol{\theta} \rightarrow 0} 2 \frac{ ([\hat{\mathbf{q}}_{\text{b}_{j}\text{b}_{i}} \otimes ((\mathbf{q}_{\text{wb}_{i}} \otimes \begin{bmatrix}
      1 \\ \frac{1}{2}\delta \boldsymbol{\theta}
   \end{bmatrix})^{-1} \otimes \mathbf{q}_{\text{wb}_{j}})]_{\text{xyz}}) - ([\hat{\mathbf{q}}_{\text{b}_{j}\text{b}_{i}} \otimes (\mathbf{q}_{\text{b}_{i}\text{w}} \otimes \mathbf{q}_{\text{wb}_{j}})]_{\text{xyz}}) }{\delta \boldsymbol{\theta}} \\
   &= \lim_{\delta \boldsymbol{\theta} \rightarrow 0} 2 \frac{ ([\hat{\mathbf{q}}_{\text{b}_{j}\text{b}_{i}} \otimes \begin{bmatrix}
      1 \\ -\frac{1}{2}\delta \boldsymbol{\theta}
   \end{bmatrix} \otimes \mathbf{q}_{\text{b}_{i}\text{w}} \otimes \mathbf{q}_{\text{wb}_{j}}]_{\text{xyz}}) - ([\hat{\mathbf{q}}_{\text{b}_{j}\text{b}_{i}} \otimes (\mathbf{q}_{\text{b}_{i}\text{w}} \otimes \mathbf{q}_{\text{wb}_{j}})]_{\text{xyz}}) }{\delta \boldsymbol{\theta}} \\
   &= \lim_{\delta \boldsymbol{\theta} \rightarrow 0} 2 \frac{ ([\hat{\mathbf{q}}_{\text{b}_{j}\text{b}_{i}} \otimes \begin{bmatrix}
      1 \\ -\frac{1}{2}\delta \boldsymbol{\theta}
   \end{bmatrix} \otimes \mathbf{q}_{\text{b}_{i}\text{b}_{j}}]_{\text{xyz}}) - ([\hat{\mathbf{q}}_{\text{b}_{j}\text{b}_{i}} \otimes \mathbf{q}_{\text{b}_{i}\text{b}_{j}}]_{\text{xyz}}) }{\delta \boldsymbol{\theta}} \\
   &= \lim_{\delta \boldsymbol{\theta} \rightarrow 0} 2 \frac{ [\hat{\mathbf{q}}_{\text{b}_{j}\text{b}_{i}} \otimes \begin{bmatrix}
      0 \\ -\frac{1}{2}\delta \boldsymbol{\theta}
   \end{bmatrix} \otimes \mathbf{q}_{\text{b}_{i}\text{b}_{j}}]_{\text{xyz}}}{\delta \boldsymbol{\theta}} \\
   &= \lim_{\delta \boldsymbol{\theta} \rightarrow 0} 2 \frac{ [ [\hat{\mathbf{q}}_{\text{b}_{j}\text{b}_{i}}]_{\text{L}} [\mathbf{q}_{\text{b}_{i}\text{b}_{j}}]_{\text{R}} \begin{bmatrix}
      0 \\ -\frac{1}{2}\delta \boldsymbol{\theta}
   \end{bmatrix} ]_{\text{xyz}}}{\delta \boldsymbol{\theta}} \\
   &= \lim_{\delta \boldsymbol{\theta} \rightarrow 0} 2 \frac{ \begin{bmatrix}
      \mathbf{0} & \mathbf{I}
   \end{bmatrix} [\hat{\mathbf{q}}_{\text{b}_{j}\text{b}_{i}}]_{\text{L}} [\mathbf{q}_{\text{b}_{i}\text{b}_{j}}]_{\text{R}} \begin{bmatrix}
      0 \\ -\frac{1}{2}\delta \boldsymbol{\theta}
   \end{bmatrix}}{\delta \boldsymbol{\theta}} \\
   &= \begin{bmatrix}
      \mathbf{0} & \mathbf{I}
   \end{bmatrix} [\hat{\mathbf{q}}_{\text{b}_{j}\text{b}_{i}}]_{\text{L}} [\mathbf{q}_{\text{b}_{i}\text{b}_{j}}]_{\text{R}} \begin{bmatrix}
      \mathbf{0}^{\text{T}} \\ -\mathbf{I}
   \end{bmatrix}
\end{aligned}.
$$

Note that here $$\frac{\partial \mathbf{r}_{\text{q}}}{\partial \mathbf{q}_{\text{wb}_{i}}}$$ is __different__ from the implementation in VINS-Mono (
   $$\frac{\partial \mathbf{r}_{\text{q}}}{\partial \mathbf{q}_{\text{wb}_{i}}} = \begin{bmatrix}
      \mathbf{0} & \mathbf{I}
   \end{bmatrix} [\hat{\mathbf{q}}_{\text{b}_{j}\text{b}_{i}}]_{\text{R}} [\mathbf{q}_{\text{b}_{i}\text{b}_{j}}]_{\text{L}} \begin{bmatrix}
      \mathbf{0}^{\text{T}} \\ -\mathbf{I}
   \end{bmatrix}$$
   ). (I wonder why...)

Define 
$$
\mathbf{C}_{2} = \mathbf{v}_{\text{wb}_{j}} - \mathbf{v}_{\text{wb}_{i}} + \mathbf{g}^{\text{w}} \Delta t,
$$

$$
\begin{aligned}
   \frac{\partial \mathbf{r}_{\text{v}}}{\partial \mathbf{q}_{\text{wb}_{i}}} &= \frac{\partial (\mathbf{q}_{\text{b}_{i}\text{w}}\mathbf{C}_{2} - \hat{\boldsymbol{\beta}}_{\text{b}_{i}\text{b}_{j}})}{\partial \mathbf{q}_{\text{wb}_{i}}} \\ 
   &= \lim_{\delta \boldsymbol{\theta} \rightarrow 0} \frac{( (\mathbf{R}_{\text{wb}_{i}} \exp(\delta \boldsymbol{\theta}^{\wedge}))^{-1}\mathbf{C}_{2} - \hat{\boldsymbol{\alpha}}_{\text{b}_{i}\text{b}_{j}}) - (\mathbf{R}_{\text{b}_{i}\text{w}}\mathbf{C}_{2} - \hat{\boldsymbol{\alpha}}_{\text{b}_{i}\text{b}_{j}}) }{\delta \boldsymbol{\theta}} \\
   &= \lim_{\delta \boldsymbol{\theta} \rightarrow 0} \frac{\exp(-\delta \boldsymbol{\theta}^{\wedge})\mathbf{R}_{\text{b}_{i}\text{w}} \mathbf{C}_{2} - \mathbf{R}_{\text{b}_{i}\text{w}}\mathbf{C}_{2}}{\delta \boldsymbol{\theta}} \\
   &= \lim_{\delta \boldsymbol{\theta} \rightarrow 0} \frac{(\mathbf{I}-\delta \boldsymbol{\theta}^{\wedge})\mathbf{R}_{\text{b}_{i}\text{w}} \mathbf{C}_{2} - \mathbf{R}_{\text{b}_{i}\text{w}}\mathbf{C}_{2}}{\delta \boldsymbol{\theta}} \\
   &= \lim_{\delta \boldsymbol{\theta} \rightarrow 0} \frac{-\delta \boldsymbol{\theta}^{\wedge}\mathbf{R}_{\text{b}_{i}\text{w}} \mathbf{C}_{2}}{\delta \boldsymbol{\theta}} \\
   &= \lim_{\delta \boldsymbol{\theta} \rightarrow 0} \frac{(\mathbf{R}_{\text{b}_{i}\text{w}} \mathbf{C}_{2})^{\wedge}\delta \boldsymbol{\theta}}{\delta \boldsymbol{\theta}} \\
   &= (\mathbf{R}_{\text{b}_{i}\text{w}} \mathbf{C}_{2})^{\wedge}
\end{aligned}
$$

#### Velocity 1

$$
\begin{aligned}
   \frac{\partial \mathbf{r}_{\text{p}}}{\partial \mathbf{v}_{\text{wb}_{i}}} &= \frac{\partial (\mathbf{q}_{\text{b}_{i}\text{w}}(\mathbf{p}_{\text{wb}_{j}} - \mathbf{p}_{\text{wb}_{i}} - \mathbf{v}_{\text{wb}_{i}} \Delta t + \frac{1}{2}\mathbf{g}^{\text{w}}\Delta t^{2}) - \hat{\boldsymbol{\alpha}}_{\text{b}_{i}\text{b}_{j}})}{\partial \mathbf{v}_{\text{wb}_{i}}} \\
   &= -\mathbf{R}_{\text{b}_{i}\text{w}}\Delta t
\end{aligned}
$$

$$
\begin{aligned}
   \frac{\partial \mathbf{r}_{\text{v}}}{\partial \mathbf{v}_{\text{wb}_{i}}} &= \frac{\partial (\mathbf{q}_{\text{b}_{i}\text{w}} (\mathbf{v}_{\text{wb}_{j}} - \mathbf{v}_{\text{wb}_{i}} + \mathbf{g}^{\text{w}} \Delta t) - \hat{\boldsymbol{\beta}}_{\text{b}_{i}\text{b}_{j}})}{\partial \mathbf{v}_{\text{wb}_{i}}} \\
   &= -\mathbf{R}_{\text{b}_{i}\text{w}}
\end{aligned}
$$

#### Bias accelerometer 1

$$
\begin{aligned}
   \frac{\partial \mathbf{r}_{\text{p}}}{\partial \mathbf{b}^{\text{a}}_{i}} = \frac{\partial (-\hat{\boldsymbol{\alpha}}_{\text{b}_{i}\text{b}_{j}})}{\partial \mathbf{b}^{\text{a}}_{i}}=-\mathbf{J}_{b_{i}^{\text{a}}}^{\alpha}
\end{aligned}
$$

$$
\begin{aligned}
   \frac{\partial \mathbf{r}_{\text{v}}}{\partial \mathbf{b}^{\text{a}}_{i}} = \frac{\partial (-\hat{\boldsymbol{\beta}}_{\text{b}_{i}\text{b}_{j}})}{\partial \mathbf{b}^{\text{a}}_{i}}=-\mathbf{J}_{b_{i}^{\text{a}}}^{\beta}
\end{aligned}
$$

$$
\begin{aligned}
   \frac{\partial \mathbf{r}_{\text{ba}}}{\partial \mathbf{b}^{\text{a}}_{i}} = -\mathbf{I}
\end{aligned}
$$

#### Bias gyroscope 1

$$
\begin{aligned}
   \frac{\partial \mathbf{r}_{\text{p}}}{\partial \mathbf{b}^{\text{g}}_{i}} = \frac{\partial (-\hat{\boldsymbol{\alpha}}_{\text{b}_{i}\text{b}_{j}})}{\partial \mathbf{b}^{\text{g}}_{i}}=-\mathbf{J}_{b_{i}^{\text{g}}}^{\alpha}
\end{aligned}
$$

$$
\begin{aligned}
   \frac{\partial \mathbf{r}_{\text{q}}}{\partial \mathbf{b}^{\text{g}}_{i}} &= \frac{\partial (2 [\hat{\mathbf{q}}_{\text{b}_{j}\text{b}_{i}} \otimes (\mathbf{q}_{\text{b}_{i}\text{w}} \otimes \mathbf{q}_{\text{wb}_{j}})]_{\text{xyz}})}{\partial \mathbf{b}^{\text{g}}_{i}} \\
   &= \lim_{ \delta \mathbf{b}_{i}^{g} \rightarrow 0} 2 \frac{ ([\hat{\mathbf{q}}_{\text{b}_{j}\text{b}_{i}} \otimes \begin{bmatrix}
      1 \\ \frac{1}{2} \mathbf{J}_{b_{i}^{g}}^{q} \delta \mathbf{b}_{i}^{g}
   \end{bmatrix} \otimes \mathbf{q}_{\text{b}_{i}\text{w}} \otimes \mathbf{q}_{\text{wb}_{j}}]_{\text{xyz}}) - ([\hat{\mathbf{q}}_{\text{b}_{j}\text{b}_{i}} \otimes (\mathbf{q}_{\text{b}_{i}\text{w}} \otimes \mathbf{q}_{\text{wb}_{j}})]_{\text{xyz}}) }{ \delta \mathbf{b}_{i}^{g}} \\
   &= \lim_{ \delta \mathbf{b}_{i}^{g} \rightarrow 0} 2 \frac{ [\hat{\mathbf{q}}_{\text{b}_{j}\text{b}_{i}} \otimes \begin{bmatrix}
      0 \\ \frac{1}{2} \mathbf{J}_{b_{i}^{g}}^{q} \delta \mathbf{b}_{i}^{g}
   \end{bmatrix} \otimes \mathbf{q}_{\text{b}_{i}\text{w}} \otimes \mathbf{q}_{\text{wb}_{j}}]_{\text{xyz}} }{ \delta \mathbf{b}_{i}^{g}} \\
   &= \lim_{ \delta \mathbf{b}_{i}^{g} \rightarrow 0} 2 \frac{ \begin{bmatrix}
      \mathbf{0} & \mathbf{I}
   \end{bmatrix}[\hat{\mathbf{q}}_{\text{b}_{j}\text{b}_{i}}]_{\text{L}} [\mathbf{q}_{\text{b}_{j}\text{b}_{i}}]_{\text{R}} \begin{bmatrix}
      0 \\ \frac{1}{2} \mathbf{J}_{b_{i}^{g}}^{q} \delta \mathbf{b}_{i}^{g}
   \end{bmatrix} }{ \delta \mathbf{b}_{i}^{g}} \\
   &= \begin{bmatrix}
      \mathbf{0} & \mathbf{I}
   \end{bmatrix}[\hat{\mathbf{q}}_{\text{b}_{j}\text{b}_{i}}]_{\text{L}} [\mathbf{q}_{\text{b}_{j}\text{b}_{i}}]_{\text{R}} \begin{bmatrix}
      \mathbf{0}^{\text{T}} \\ \mathbf{J}_{b_{i}^{g}}^{q} 
   \end{bmatrix} \\
\end{aligned}
$$

Note that here $$\frac{\partial \mathbf{r}_{\text{q}}}{\partial \mathbf{b}^{\text{g}}_{i}}$$ is __different__ from the implementation in VINS-Mono (
   $$\frac{\partial \mathbf{r}_{\text{q}}}{\partial \mathbf{b}^{\text{g}}_{i}} = -\begin{bmatrix}
      \mathbf{0} & \mathbf{I}
   \end{bmatrix}[\mathbf{q}_{\text{b}_{j}w}\otimes \mathbf{q}_{\text{wb}_{i}}\otimes \hat{\mathbf{q}}_{\text{b}_{i}\text{b}_{j}}]_{\text{L}}\begin{bmatrix}
      \mathbf{0}^{\text{T}} \\ \mathbf{J}_{b_{i}^{g}}^{q} 
   \end{bmatrix} $$
   ). (I wonder why...)


$$
\begin{aligned}
   \frac{\partial \mathbf{r}_{\text{v}}}{\partial \mathbf{b}^{\text{g}}_{i}} = \frac{\partial (-\hat{\boldsymbol{\beta}}_{\text{b}_{i}\text{b}_{j}})}{\partial \mathbf{b}^{\text{g}}_{i}}=-\mathbf{J}_{b_{i}^{\text{g}}}^{\beta}
\end{aligned}
$$

$$
\begin{aligned}
   \frac{\partial \mathbf{r}_{\text{bg}}}{\partial \mathbf{b}^{\text{g}}_{i}} = -\mathbf{I}
\end{aligned}
$$

#### Position 2

$$
\begin{aligned}
   \frac{\partial \mathbf{r}_{\text{p}}}{\partial \mathbf{p}_{\text{wb}_{j}}} &= \frac{\partial (\mathbf{q}_{\text{b}_{i}\text{w}}(\mathbf{p}_{\text{wb}_{j}} - \mathbf{p}_{\text{wb}_{j}} - \mathbf{v}_{\text{wb}_{i}} \Delta t + \frac{1}{2}\mathbf{g}^{\text{w}}\Delta t^{2}) - \hat{\boldsymbol{\alpha}}_{\text{b}_{i}\text{b}_{j}})}{\partial \mathbf{p}_{\text{wb}_{j}}} \\ &= \mathbf{q}_{\text{b}_{i}\text{w}} \\ &= \mathbf{R}_{\text{b}_{i}\text{w}} 
\end{aligned}
$$


#### Rotation 2

$$
\begin{aligned}
   \frac{\partial \mathbf{r}_{\text{q}}}{\partial \mathbf{q}_{\text{wb}_{j}}} &= \frac{\partial (2 [\hat{\mathbf{q}}_{\text{b}_{j}\text{b}_{i}} \otimes (\mathbf{q}_{\text{b}_{i}\text{w}} \otimes \mathbf{q}_{\text{wb}_{j}})]_{\text{xyz}})}{\partial \mathbf{q}_{\text{wb}_{j}}} \\ 
   &= \lim_{\delta \boldsymbol{\theta} \rightarrow 0} 2 \frac{ ([\hat{\mathbf{q}}_{\text{b}_{j}\text{b}_{i}} \otimes (\mathbf{q}_{\text{b}_{i}\text{w}} \otimes \mathbf{q}_{\text{wb}_{j}} \otimes \begin{bmatrix}
      1 \\ \frac{1}{2}\delta \boldsymbol{\theta}
   \end{bmatrix})]_{\text{xyz}}) - ([\hat{\mathbf{q}}_{\text{b}_{j}\text{b}_{i}} \otimes (\mathbf{q}_{\text{b}_{i}\text{w}} \otimes \mathbf{q}_{\text{wb}_{j}})]_{\text{xyz}}) }{\delta \boldsymbol{\theta}} \\
   &= \lim_{\delta \boldsymbol{\theta} \rightarrow 0} 2 \frac{ [\hat{\mathbf{q}}_{\text{b}_{j}\text{b}_{i}} \otimes \mathbf{q}_{\text{b}_{i}\text{w}} \otimes \mathbf{q}_{\text{wb}_{j}} \otimes \begin{bmatrix}
      0 \\ \frac{1}{2}\delta \boldsymbol{\theta}
   \end{bmatrix}]_{\text{xyz}} }{\delta \boldsymbol{\theta}} \\
   &= \lim_{\delta \boldsymbol{\theta} \rightarrow 0} 2 \frac{ [[\hat{\mathbf{q}}_{\text{b}_{j}\text{b}_{i}} \otimes \mathbf{q}_{\text{b}_{i}\text{w}} \otimes \mathbf{q}_{\text{wb}_{j}}]_{\text{L}} \begin{bmatrix}
      0 \\ \frac{1}{2}\delta \boldsymbol{\theta}
   \end{bmatrix}]_{\text{xyz}} }{\delta \boldsymbol{\theta}} \\
   &= \lim_{\delta \boldsymbol{\theta} \rightarrow 0} 2 \frac{ \begin{bmatrix}
      \mathbf{0} & \mathbf{I}
   \end{bmatrix}[\hat{\mathbf{q}}_{\text{b}_{j}\text{b}_{i}} \otimes \mathbf{q}_{\text{b}_{i}\text{w}} \otimes \mathbf{q}_{\text{wb}_{j}}]_{\text{L}} \begin{bmatrix}
      0 \\ \frac{1}{2}\delta \boldsymbol{\theta}
   \end{bmatrix} }{\delta \boldsymbol{\theta}} \\
   &= \begin{bmatrix}
      \mathbf{0} & \mathbf{I}
   \end{bmatrix}[\hat{\mathbf{q}}_{\text{b}_{j}\text{b}_{i}} \otimes \mathbf{q}_{\text{b}_{i}\text{w}} \otimes \mathbf{q}_{\text{wb}_{j}}]_{\text{L}}\begin{bmatrix}
      \mathbf{0}^{\text{T}} \\ \mathbf{I}
   \end{bmatrix} 
\end{aligned}.
$$

#### Velocity 2

$$
\begin{aligned}
   \frac{\partial \mathbf{r}_{\text{v}}}{\partial \mathbf{v}_{\text{wb}_{j}}} &= \frac{\partial (\mathbf{q}_{\text{b}_{i}\text{w}} (\mathbf{v}_{\text{wb}_{j}} - \mathbf{v}_{\text{wb}_{i}} + \mathbf{g}^{\text{w}} \Delta t) - \hat{\boldsymbol{\beta}}_{\text{b}_{i}\text{b}_{j}})}{\partial \mathbf{v}_{\text{wb}_{j}}} \\
   &= \mathbf{R}_{\text{b}_{i}\text{w}}
\end{aligned}
$$

#### Bias accelerometer 2

$$
\begin{aligned}
   \frac{\partial \mathbf{r}_{\text{ba}}}{\partial \mathbf{b}^{\text{a}}_{j}} = \mathbf{I}
\end{aligned}
$$


#### Bias gyroscope 2

$$
\begin{aligned}
   \frac{\partial \mathbf{r}_{\text{bg}}}{\partial \mathbf{b}^{\text{g}}_{j}} = \mathbf{I}
\end{aligned}
$$

### Visual Residual

### Discussion
   
1. How do we balance contributions (weights) of IMU, visual and prior residuals?

### Literature

[^Qin17]: Qin, Tong, and Shaojie Shen. "Robust initialization of monocular visual-inertial estimation on aerial robots." 2017 IROS.
