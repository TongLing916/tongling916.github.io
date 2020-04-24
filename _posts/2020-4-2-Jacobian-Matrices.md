---
layout:     post
title:      "Important Jacobians in SLAM"
date:       2020-4-2
author:     Tong
catalog: true
tags:
    - SLAM
---

### Convention

#### Lie Group - Lie Algebra

- Lie algebra / group is defined as follows. Note: `translation` in Lie Algebra is not equal to the translation $$\mathbf{t}$$ in Lie group.

$$
\boldsymbol{\xi}=\left[\begin{array}{c}
\text { translation } \\
\text { rotation }
\end{array}\right]
$$

$$
\mathbf{T}=\begin{bmatrix}
\mathbf{R} & \mathbf{t} \\
\mathbf{0}^T & 1
\end{bmatrix}
$$

$$
\boldsymbol{\xi}= \ln{(\mathbf{T}^{\vee})}
$$

$$
\mathbf{T}= \exp{(\boldsymbol{\xi}^{\wedge})}
$$

- Operators as introduced in sec. 7.1.8 [^Barfoot17]

$$
\mathbf{p}=\left[\begin{array}{c}
s x \\
s y \\
s z \\
s
\end{array}\right]=\left[\begin{array}{l}
\varepsilon \\
\eta
\end{array}\right]
$$

$$
\left[\begin{array}{l}
\varepsilon \\
\eta
\end{array}\right]^{\odot}=\left[\begin{array}{cc}
\eta \mathbf{I} & -\varepsilon^{\wedge} \\
0^{T} & 0
\end{array}\right]
$$

$$
\left[\begin{array}{l}
\varepsilon \\
\eta
\end{array}\right]^{\circledcirc}=\left[\begin{array}{cc}
0 & \varepsilon \\
-\varepsilon^{\wedge} & 0
\end{array}\right]
$$

$$
\boldsymbol{\xi}^{\wedge} \mathbf{p} \equiv \mathbf{p}^{\odot} \boldsymbol{\xi}, \quad \mathbf{p}^{T} \boldsymbol{\xi}^{\wedge} \equiv \boldsymbol{\xi}^{T} \mathbf{p}^{\circledcirc}
$$ 

$$
\boldsymbol{\phi}^{\wedge}=\left[\begin{array}{c}
\phi_{1} \\
\phi_{2} \\
\phi_{3}
\end{array}\right]^{\wedge}=\left[\begin{array}{ccc}
0 & -\phi_{3} & \phi_{2} \\
\phi_{3} & 0 & -\phi_{1} \\
-\phi_{2} & \phi_{1} & 0
\end{array}\right] \in \mathbb{R}^{3 \times 3}, \quad \boldsymbol{\phi} \in \mathbb{R}^{3}
$$

- Adjoint ($$6 \times 6$$)

$$\mathcal{T} = \text{Ad}(\mathbf{T}) = \begin{bmatrix}
\mathbf{R} & \mathbf{t}^{\wedge}\mathbf{R} \\
\mathbf{0} & \mathbf{R}
 \end{bmatrix}$$

$$\mathcal{T}^{-1} = \begin{bmatrix}
\mathbf{R}^{T} & -\mathbf{R}^{T}\mathbf{t}^{\wedge} \\
\mathbf{0} & \mathbf{R}^{T}
 \end{bmatrix}$$

$$\left(\mathbf{T}\mathbf{p}\right)^{\odot} \equiv \mathbf{T}\mathbf{p}^{\odot} \mathcal{T}^{-1}$$

$$\exp{((\mathcal{T}\boldsymbol{\xi})^{\wedge})} = \mathbf{T} \exp{(\boldsymbol{\xi}^{\wedge})} \mathbf{T}^{-1}$$

- Approximation of pose 

$$
\begin{aligned}
\mathbf{T}=& \exp \left(\boldsymbol{\xi}^{\wedge}\right) \\
=& \sum_{n=0}^{\infty} \frac{1}{n !}\left(\boldsymbol{\xi}^{\wedge}\right)^{n} \\
=&\mathbf{I}+\boldsymbol{\xi}^{\wedge}+\frac{1}{2 !}\left(\boldsymbol{\xi}^{\wedge}\right)^{2}+\frac{1}{3 !}\left(\boldsymbol{\xi}^{\wedge}\right)^{3}+\frac{1}{4 !}\left(\boldsymbol{\xi}^{\wedge}\right)^{4}+\frac{1}{5 !}\left(\boldsymbol{\xi}^{\wedge}\right)^{5}+\cdots \\
=& \mathbf{I}+\boldsymbol{\xi}^{\wedge}+\underbrace{\left(\frac{1}{2 !}-\frac{1}{4 !} \phi^{2}+\frac{1}{6 !} \phi^{4}-\frac{1}{8 !} \phi^{6}+\cdots\right)\left(\boldsymbol{\xi}^{\wedge}\right)^{2}}_{\frac{1-\cos \phi}{\phi^{2}}} \\
&+\underbrace{\left(\frac{1}{3 !}-\frac{1}{5 !} \phi^{2}+\frac{1}{7 !} \phi^{4}-\frac{1}{9 !} \phi^{6}+\cdots\right.}_{\frac{\phi-\sin \phi}{\phi^{3}}})\left(\boldsymbol{\xi}^{\wedge}\right)^{3} \\
&=\mathbf{I}+\boldsymbol{\xi}^{\wedge}+\left(\frac{1-\cos \phi}{\phi^{2}}\right)\left(\boldsymbol{\xi}^{\wedge}\right)^{2}+\left(\frac{\phi-\sin \phi}{\phi^{3}}\right)\left(\boldsymbol{\xi}^{\wedge}\right)^{3}
\end{aligned}
$$

- $$\mathbf{Sim}(3)$$

$$
\boldsymbol{\xi}^{s}=\left[\begin{array}{c}
\text { translation } \\
\text { rotation } \\
\text {scale}
\end{array}\right]
$$

$$
\mathbf{S} = \begin{bmatrix}
s\mathbf{R} & \mathbf{t} \\
\mathbf{0}^{T} & 1 
 \end{bmatrix}
$$

$$\mathcal{T}^s = \text{Ad}(\mathbf{S}) = \begin{bmatrix}
s\mathbf{R} & \mathbf{t}^{\wedge}\mathbf{R} & -\mathbf{t}\\
\mathbf{0} & \mathbf{R} & \mathbf{0} \\
\mathbf{0}^T & \mathbf{0}^T & 1
 \end{bmatrix}$$


| Variable                                | Meaning                 |
| --------------------------------------- | ----------------------- |
| $$\mathbf{p} \in \mathbb{R}^{4}$$       | homogeneous coordinates |
| $$\varepsilon \in \mathbb{R}^{3}$$      | $$3\times 1$$ vector    |
| $$\eta$$                                | scalar                  |
| $$\boldsymbol{\xi} \in \mathbb{R}^{6}$$ | lie algebra (e.g. pose) |

#### Huber Cost Function [^Hartley04]

$$
\begin{aligned}
C(\delta) &=\delta^{2} \text { for }|\delta|<b \\
&=2 b|\delta|-b^{2} \text { otherwise }
\end{aligned}
$$

where $$\delta$$ is the residual, $$b$$ is the threshold.

Through simple conversions, we can obtain a formulation which is more suitable for Jacobian computation,

$$
C(\delta) = \lambda (2 - \lambda) \delta^{2}
$$

where 

$$
\begin{aligned}
\lambda &= 1 \text { for }|\delta|<b \\
&= \frac{b}{\left | \delta \right |} \text { otherwise }
\end{aligned}
$$

In this way, the residual is converted from 
$$
\delta
$$ 
into 
$$
\omega_{h} \left | \delta \right |
$$ 

The coefficient $$\omega_{h} = \sqrt{\lambda (2 - \lambda)}$$ will be called __Huber weight__ in the following sections.


### Reprojection Error

The reprojection error is defined as follows

$$
\mathbf{r} = \mathbf{p}_{i} - \mathbf{K}\mathbf{T}_{iw}\mathbf{x} = \mathbf{p}_{i} - \mathbf{K}\mathbf{p}^{c}
$$

__Note__: In the cost function above, there is an implicit conversion to ensure the last element of homogeneous pixel coordinates to be one.

| Variable                                                                                | Meaning                                          |
| --------------------------------------------------------------------------------------- | ------------------------------------------------ |
| $$\mathbf{r}= \begin{bmatrix}\delta u \\ \delta v\end{bmatrix}$$                        | reprojection error                               |
| $$\mathbf{p}_{i}=\begin{bmatrix} u \\ v\end{bmatrix}$$                                  | corresponding pixel                              |
| $$\mathbf{K}=\begin{bmatrix}f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}$$ | calibration matrix                               |
| $$\mathbf{T}_{iw} = \begin{bmatrix}\mathbf{R} & \mathbf{t}\end{bmatrix}$$               | camera pose (__to optimize__)                    |
| $$\mathbf{x}$$                                                                          | 3D point (__to optimize__)                       |
| $$\mathbf{p}^{c} = \begin{bmatrix}x^{c} \\ y^{c} \\ z^{c}\end{bmatrix}$$                | 3D point in the camera $$i$$'s coordinate system |

#### 3D point

$$
\frac{\partial \mathbf{r}}{\partial \mathbf{x}} = \frac{\partial \mathbf{r}}{\partial \mathbf{p}^{c}} \frac{\partial \mathbf{p}^{c}}{\partial \mathbf{x}}
$$

Since
$$
\frac{1}{\rho} \begin{bmatrix}
    \hat{u} \\ \hat{v} \\ 1
\end{bmatrix} = \mathbf{K}\mathbf{p}^{c} = \begin{bmatrix}f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}\begin{bmatrix}x^{c} \\ y^{c} \\ z^{c}\end{bmatrix} = \begin{bmatrix}f_x x^{c} + c_x z^{c}  \\ f_y y^{c} + c_y z^{c} \\ z^{c}\end{bmatrix}
$$, where $$\rho$$ is the inverse depth of the 3D point, we have then 

$$
\mathbf{r} = \begin{bmatrix}u - f_x \frac{x^{c}}{z^{c}} - c_x \\ v - f_y \frac{y^{c}}{z^{c}} - c_y\end{bmatrix}
$$

Therefore,

$$
 \frac{\partial \mathbf{r}}{\partial \mathbf{p}^{c}} = \begin{bmatrix}
     -\frac{f_x}{z_c} & 0 & \frac{f_x x_c}{z_c^{2}} \\
     0 & -\frac{f_y}{z_c} & \frac{f_y y_c}{z_c^{2}}
 \end{bmatrix}
$$

Besides,

$$
\frac{\partial \mathbf{p}^{c}}{\partial \mathbf{x}} = \frac{\partial (\mathbf{R}\mathbf{x} + \mathbf{t})}{\partial \mathbf{x}} = \mathbf{R}
$$

All in all, 

$$
\frac{\partial \mathbf{r}}{\partial \mathbf{x}} = \frac{\partial \mathbf{r}}{\partial \mathbf{p}^{c}} \frac{\partial \mathbf{p}^{c}}{\partial \mathbf{x}} = \begin{bmatrix}
     -\frac{f_x}{z_c} & 0 & \frac{f_x x_c}{z_c^{2}} \\
     0 & -\frac{f_y}{z_c} & \frac{f_y y_c}{z_c^{2}}
 \end{bmatrix}\mathbf{R}
$$

#### Absolute camera pose

$$
\frac{\partial \mathbf{r}}{\partial \boldsymbol{\xi}_{iw}} = \frac{\partial \mathbf{r}}{\partial \mathbf{p}^{c}} \frac{\partial \mathbf{p}^{c}}{\partial \boldsymbol{\xi}_{iw}}
$$

$$
\begin{aligned}
\frac{\partial \mathbf{r}}{\partial \boldsymbol{\xi}_{iw}} &= \lim_{\delta \boldsymbol{\xi}_{iw} \rightarrow 0} \frac{\exp{(\delta \boldsymbol{\xi}_{iw})}^{\wedge}\mathbf{T}_{iw}\mathbf{x} - \mathbf{T}_{iw}\mathbf{x}}{\delta \boldsymbol{\xi}_{iw}} \\ 
&= \lim_{\delta \boldsymbol{\xi}_{iw} \rightarrow 0} \frac{(\mathbf{I} + \delta \boldsymbol{\xi}_{iw}^{\wedge})\mathbf{T}_{iw}\mathbf{x} - \mathbf{T}_{iw}\mathbf{x}}{\delta \boldsymbol{\xi}_{iw}} \\
&= \lim_{\delta \boldsymbol{\xi}_{iw} \rightarrow 0} \frac{\delta \boldsymbol{\xi}_{iw}^{\wedge}\mathbf{T}_{iw}\mathbf{x}}{\delta \boldsymbol{\xi}_{iw}} \\
&= \lim_{\delta \boldsymbol{\xi}_{iw} \rightarrow 0} \frac{(\mathbf{T}_{iw}\mathbf{x})^{\odot}\delta \boldsymbol{\xi}_{iw}}{\delta \boldsymbol{\xi}_{iw}} \\
&= (\mathbf{T}_{iw}\mathbf{x})^{\odot}
\end{aligned}
$$

### Photometric Error (3D Point)

#### Camera Pose

#### 3D Point

### Photometric Error (Inverse Depth)

#### Camera Pose

#### Inverse Depth

### [Photometric Error (DSO)](http://www.lingtong.de/2020/04/17/DSO-Jacobians/)

### Photometric Error (Plane)

#### Plane [a, b, c, d]

### Pose Graph Error

#### SE(3)

- Residual
$$\mathbf{r}_{ji} = \ln{(\bar{\mathbf{T}}_{ji} \mathbf{T}_{iw} \mathbf{T}_{jw}^{-1})^{\vee}}$$

| Variable  | Meaning    |
| ------------------- | -------------------------------------- |
| $$\mathbf{r}_{ji} \in \mathbb{R}^{6}$$ | single residual, `3` for translation, `3` for rotation |
|$$\bar{\mathbf{T}}_{ji}$$ |  measurement $$\mathbf{SE}(3)$$    |
| $$\mathbf{T}_{iw}, \mathbf{T}_{jw}$$    |    variables to optimize (Lie group) |
| $$\boldsymbol{\xi}_{iw}, \boldsymbol{\xi}_{jw}$$    |    variables to optimize (Lie algebra) |

#### Sim(3)

- Residual
$$\mathbf{r}_{ji} = \ln{(\bar{\mathbf{S}}_{ji} \mathbf{S}_{iw} \mathbf{S}_{jw}^{-1})^{\vee}}$$

| Variable  | Meaning    |
| ------------------- | -------------------------------------- |
| $$\mathbf{r}_{ji} \in \mathbb{R}^{7}$$ | single residual, `3` for translation, `3` for rotation, `1` for scale |
|$$\bar{\mathbf{S}}_{ji}$$ |  measurement $$\mathbf{Sim}(3)$$    |
| $$\mathbf{S}_{iw}, \mathbf{S}_{jw}$$    |    variables to optimize   (Lie group)  |
| $$\boldsymbol{\xi}_{iw}^{s}, \boldsymbol{\xi}_{jw}^{s}$$    |    variables to optimize   (Lie algebra)  |

- $$\frac{\partial \mathbf{r}_{ji}}{\partial \boldsymbol{\xi}_{iw}}$$

$$
\begin{aligned}
\frac{\partial  \mathbf{r}_{ji}}{\partial \boldsymbol{\xi}_{iw}^{s}} &= \lim_{\delta \boldsymbol{\xi}_{iw}^{s} \rightarrow 0} \frac{ \delta  \mathbf{r}_{ji}}{ \delta \boldsymbol{\xi}_{iw}^{s}} \\
&= \lim_{\delta \boldsymbol{\xi}_{iw}^{s} \rightarrow 0} \frac{ \ln{(\exp{(\delta \mathbf{r}_{ji})}^{\wedge})^{\vee}}}{ \delta \boldsymbol{\xi}_{iw}^{s}} \\
&= \lim_{\delta \boldsymbol{\xi}_{iw}^{s} \rightarrow 0} \frac{ \ln{(\bar{\mathbf{S}}_{ji} \exp{(\delta \boldsymbol{\xi}_{iw}^{s})^{\wedge}}\mathbf{S}_{iw} \mathbf{S}_{jw}^{-1}(\bar{\mathbf{S}}_{ji} \mathbf{S}_{iw} \mathbf{S}_{jw}^{-1})^{-1})^{\vee}}}{ \delta \boldsymbol{\xi}_{iw}^{s}} \\
&= \lim_{\delta \boldsymbol{\xi}_{iw}^{s} \rightarrow 0} \frac{ \ln{(\bar{\mathbf{S}}_{ji} \exp{(\delta \boldsymbol{\xi}_{iw}^{s})^{\wedge}}\mathbf{S}_{iw}\mathbf{S}_{jw}^{-1} \mathbf{S}_{jw} \mathbf{S}_{iw}^{-1}\bar{\mathbf{S}}_{ji}^{-1})^{\vee}}}{ \delta \boldsymbol{\xi}_{iw}^{s}} \\
&= \lim_{\delta \boldsymbol{\xi}_{iw}^{s} \rightarrow 0} \frac{ \ln{(\bar{\mathbf{S}}_{ji} \exp{(\delta \boldsymbol{\xi}_{iw}^{s})^{\wedge}}\bar{\mathbf{S}}_{ji}^{-1})}}{ \delta \boldsymbol{\xi}_{iw}^{s}} \\
&= \lim_{\delta \boldsymbol{\xi}_{iw}^{s} \rightarrow 0} \frac{ \ln{( \exp{(\mathcal{T}^{s}\delta \boldsymbol{\xi}_{iw}^{s})^{\wedge}})}^{\vee}}{ \delta \boldsymbol{\xi}_{iw}^{s}} \\
&= \lim_{\delta \boldsymbol{\xi}_{iw}^{s} \rightarrow 0} \frac{ \mathcal{T}^{s}\delta \boldsymbol{\xi}_{iw}^{s}}{ \delta \boldsymbol{\xi}_{iw}^{s}} \\
&= \mathcal{T}^{s} \\ 
&= \mathbf{Ad}(\bar{\mathbf{S}}_{ji})
\end{aligned}
$$

- $$\frac{\partial\mathbf{r}_{ji}}{\partial \boldsymbol{\xi}_{jw}^{s}}$$

$$
\begin{aligned}
\frac{\partial  \mathbf{r}_{ji}}{\partial \boldsymbol{\xi}_{jw}^{s}} &= \lim_{\delta \boldsymbol{\xi}_{jw}^{s} \rightarrow 0} \frac{ \delta  \mathbf{r}_{ji}}{ \delta \boldsymbol{\xi}_{jw}^{s}} \\
&= \lim_{\delta \boldsymbol{\xi}_{jw}^{s} \rightarrow 0} \frac{ \ln{(\exp{(\delta \mathbf{r}_{ji})}^{\wedge})^{\vee}}}{ \delta \boldsymbol{\xi}_{jw}^{s}} \\
&= \lim_{\delta \boldsymbol{\xi}_{jw}^{s} \rightarrow 0} \frac{ \ln{(\bar{\mathbf{S}}_{ji} \mathbf{S}_{iw} (\exp{(\delta \boldsymbol{\xi}_{jw}^{s})^{\wedge}}\mathbf{S}_{jw})^{-1}(\bar{\mathbf{S}}_{ji} \mathbf{S}_{iw} \mathbf{S}_{jw}^{-1})^{-1})^{\vee}}}{ \delta \boldsymbol{\xi}_{jw}^{s}} \\
&= \lim_{\delta \boldsymbol{\xi}_{jw}^{s} \rightarrow 0} \frac{ \ln{(\bar{\mathbf{S}}_{ji} \mathbf{S}_{iw}\mathbf{S}_{jw}^{-1} \exp{(-\delta \boldsymbol{\xi}_{jw}^{s})^{\wedge}}(\bar{\mathbf{S}}_{ji} \mathbf{S}_{iw} \mathbf{S}_{jw}^{-1})^{-1})^{\vee}}}{ \delta \boldsymbol{\xi}_{jw}^{s}} \\
&= \lim_{\delta \boldsymbol{\xi}_{jw}^{s} \rightarrow 0} \frac{ \ln{(\exp{(-\mathcal{T}^{s}\delta \boldsymbol{\xi}_{jw}^{s})^{\wedge}})^{\vee}}}{ \delta \boldsymbol{\xi}_{jw}^{s}} \\
&= \lim_{\delta \boldsymbol{\xi}_{jw}^{s} \rightarrow 0} \frac{-\mathcal{T}^{s}\delta \boldsymbol{\xi}_{jw}^{s}}{ \delta \boldsymbol{\xi}_{jw}^{s}} \\
&= -\mathcal{T}^{s} \\
&= -\mathbf{Ad}(\bar{\mathbf{S}}_{ji} \mathbf{S}_{iw}\mathbf{S}_{jw}^{-1})
\end{aligned}
$$


### Literature

[^Hartley04]: R. Hartley and A. Zisserman, Multiple View Geometry in Computer Vision, 2nd ed. Cambridge University Press, 2004.

[^Engel18]: Engel, Jakob, Vladlen Koltun, and Daniel Cremers. "Direct Sparse Odometry." IEEE Transactions on Pattern Analysis and Machine Intelligence 40.3 (2018): 611-625.

[^Barfoot17]: Timothy D Barfoot. State Estimation for Robotics. Cambridge University Press, 2017.