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

- Lie algebra is defined as follows

$$
\xi=\left[\begin{array}{c}
\text { translation } \\
\text { rotation }
\end{array}\right]
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

#### Camera Pose

#### 3D Point

### Photometric Error (3D Point)

#### Camera Pose

#### 3D Point

### Photometric Error (Inverse Depth)

#### Camera Pose

#### Inverse Depth

### Photometric Error (DSO[^Engel18])

$$
E_{\mathbf{p} j}:=\sum_{\mathbf{p} \in \mathcal{N}_{\mathrm{p}}} w_{\mathbf{p}}\left\|\left(I_{j}\left[\mathbf{p}^{\prime}\right]-b_{j}\right)-\frac{t_{j} e^{a_{j}}}{t_{i} e^{a_{i}}}\left(I_{i}[\mathbf{p}]-b_{i}\right)\right\|_{\gamma}
$$

| Variable                                                                                | Meaning                                                                    |
| --------------------------------------------------------------------------------------- | -------------------------------------------------------------------------- |
| $$\mathbf{p}$$                                                                          | pixel position                                                             |
| $$\mathbf{p}^{\prime}$$                                                                 | projected pixel position of $$\mathbf{p}$$                                 |
| $$\mathcal{N}_{\mathrm{p}}$$                                                            | residual pattern (8 pixels)                                                |
| $$t_j, t_i$$                                                                            | exposure time                                                              |
| $$I_j, I_i$$                                                                            | Intensity (or irradiance as said in DSO)                                   |
| $$w_{\mathbf{p}}:=\frac{c^{2}}{c^{2}+\left\|\nabla I_{i}(\mathbf{p})\right\|_{2}^{2}}$$ | gradient-dependent weighting                                               |
| $$a_i, b_i$$                                                                            | affine brightness transfer function $$e^{-a_{i}}\left(I_{i}-b_{i}\right)$$ |
| $$\|\cdot\|_{\gamma}$$                                                                  | Huber cost function                                                        |

In the following sections, we only consider the __residual of a single point__

$$
r_{ji} = w_{\mathbf{p}}\left\|\left(I_{j}\left[\mathbf{p}^{\prime}\right]-b_{j}\right)-\frac{t_{j} e^{a_{j}}}{t_{i} e^{a_{i}}}\left(I_{i}[\mathbf{p}]-b_{i}\right)\right\|_{\gamma}
$$

Actually, Jakob Engel has converted the above formulation into the following one which is easier for the optimization

$$
\begin{aligned}
r_{ji} &= \left\|I_{j}\left[\mathbf{p}_{j}\right]-e^{a_{ji}}I_{i}[\mathbf{p}_{i}] - b_{ji}\right\|_{\gamma} \\
&= \omega_{h} (I_{j}\left[\mathbf{p}_{j}\right]-e^{a_{ji}}I_{i}[\mathbf{p}_{i}] - b_{ji})
\end{aligned}
$$

| Variable                                              | Meaning                                      |
| ----------------------------------------------------- | -------------------------------------------- |
| $$e^{a_{ji}} = \frac{t_{j}}{t_{i}}e^{a_{j} - a_{i}}$$ | $$a_{ji}$$ is an affine tranfer parameter from $$i$$ to $$j$$ |
| $$b_{ji} = b_{j} - e^{a_{ji}}b_{i}$$                  | $$b_{ji}$$ is an affine tranfer parameter from $$i$$ to $$j$$ |
| $$\omega_{h}$$                                        | Huber weight, considered a constant          |

#### Preparation

Suppose camera $$i$$'s coordinate system is the same as the world coodinate system, then we have

$$
\mathbf{p}_{i} = \mathbf{K} \rho_{i} \begin{bmatrix}
\mathbf{I} &  \mathbf{0}
\end{bmatrix} \mathbf{p}^{w} = \mathbf{K} \rho_{i} \mathbf{p}^{w} = \mathbf{K} \rho_{i} \mathbf{p}^{c}_{i}
$$

$$
\mathbf{p}_{j} = \mathbf{K} \rho_{j} \begin{bmatrix}
\mathbf{R}_{ji} &  \mathbf{t}_{ji}
\end{bmatrix} \mathbf{p}^{w} = \mathbf{K} \rho_{j} (\mathbf{R}_{ji} \mathbf{p}^{w} + \mathbf{t}_{ji})
$$

Thus, replacing $$p^w$$ in the second formel with the variables in the first one gives us

$$
\begin{aligned}
\mathbf{p}_{j} &= \mathbf{K} \rho_{j} \left ( \frac{1}{\rho_{i}} \mathbf{R}_{ji}\mathbf{K}^{-1}\mathbf{p}_i + \mathbf{t}_{ji}\right )\\
&= \mathbf{K} \rho_{j} \left ( \mathbf{R}_{ji} \mathbf{p}^c_i + \mathbf{t}_{ji}\right ) \\
&= \mathbf{K} \rho_{j} \mathbf{p}^c_j \\
&= \mathbf{K} \mathbf{p}^n_j
\end{aligned}
$$


| Variable                                                                      | Meaning                                             |
| ----------------------------------------------------------------------------- | --------------------------------------------------- |
| $$\rho_{j}$$                                                                  | point inverse depth wrt. image $$j$$                |
| $$\mathbf{p}_{j}$$                                                            | pixel position in image $$j$$                       |
| $$\mathbf{p}^n_j = \mathbf{K}^{-1} \mathbf{p}_{j} = \rho_{j} \mathbf{p}^c_j$$ | point position in the normalized plane              |
| $$\mathbf{p}^c_j = \begin{bmatrix}x^c_j \\ y^c_j \\ z^c_j\end{bmatrix}$$      | point position in the camera $$j$$ coodinate system |
| $$\mathbf{p}^w$$                                                              | point position in the world coodinate system        |


#### Camera Pose

$$
\frac{\partial r_{ji}}{\partial \xi_{ji}} = \frac{\partial r_{ji}}{\partial \mathbf{p}_{j}} \frac{\partial \mathbf{p}_{j}}{\partial \xi_{ji}}
$$

The first part can be computed as follows

$$
\frac{\partial r_{ji}}{\partial \mathbf{p}_{j}} = \omega_{h} \frac{\partial I_{j}\left[\mathbf{p}_{j}\right]}{\partial \mathbf{p}_{j}} = \omega_{h} \begin{bmatrix}
g^j_x & g^j_y
\end{bmatrix}
$$

| Variable  | Meaning                                              |
| --------- | ---------------------------------------------------- |
| $$g^j_x$$ | gradient in image $$j$$ in x-direction (u-direction) |
| $$g^j_y$$ | gradient in image $$j$$ in y-direction (v-direction) |

For the second part, we have 

$$
\frac{\partial \mathbf{p}_{j}}{\partial \xi_{ji}} = \frac{\partial \mathbf{p}_{j}}{\partial \mathbf{p}^n_j} \frac{\partial \mathbf{p}^n_j}{\partial \xi_{ji}}
$$

According the [preparation](#preparation)
$$
\mathbf{p}_j = \mathbf{K} \mathbf{p}^n_j = \begin{bmatrix}
f_x x^n_j + c_x\\ 
f_y y^n_j + c_y\\ 
1
\end{bmatrix}
$$ we have

$$
\frac{\partial \mathbf{p}_{j}}{\partial \mathbf{p}^n_j} = \begin{bmatrix}
f_x & 0 & 0\\ 
0 & f_y & 0\\ 
0 & 0 & 0
\end{bmatrix}
$$

Then, we compute 

$$
\begin{aligned}
\frac{\partial \mathbf{p}^n_j}{\partial \xi_{ji}} &= \frac{\partial (\rho_{j}\mathbf{p}^c_j)}{\partial \xi_{ji}} \\
&= \frac{\partial \mathbf{p}^c_j}{\partial \xi_{ji}} \rho_{j} +\mathbf{p}^c_j \frac{\partial \rho_{j}}{\partial \xi_{ji}} 
\end{aligned}
$$

where 

$$
\begin{aligned}
\frac{\partial \mathbf{p}^c_j}{\partial \xi_{ji}} &= \frac{\partial (\mathbf{T}_{ji}\mathbf{p}^c_i)}{\partial \xi_{ji}} \\
&= \lim_{\delta \xi \rightarrow 0} \frac{\exp{(\delta\xi^{\wedge})}\exp{(\xi_{ji}^{\wedge})}\mathbf{p}^c_i -\exp{(\xi_{ji}^{\wedge})}\mathbf{p}^c_i }{\delta \xi} \\
&= \lim_{\delta \xi \rightarrow 0} \frac{(\mathbf{I} + \delta \xi^{\wedge})\exp{(\xi_{ji}^{\wedge})}\mathbf{p}^c_i -\exp{(\xi_{ji}^{\wedge})}\mathbf{p}^c_i }{\delta \xi}\\
&= \lim_{\delta \xi \rightarrow 0} \frac{\delta \xi^{\wedge}\exp{(\xi_{ji}^{\wedge})}\mathbf{p}^c_i }{\delta \xi}\\
&= \lim_{\delta \xi \rightarrow 0} \frac{(\exp{(\xi_{ji}^{\wedge})}\mathbf{p}^c_i)^{\odot}\delta \xi }{\delta \xi}\\
&= (\exp{(\xi_{ji}^{\wedge})}\mathbf{p}^c_i)^{\odot} \\
&= (\mathbf{p}^c_j)^{\odot} \\
&= \begin{bmatrix}
1 & 0 & 0 & 0 & z^c_j & -y^c_j\\ 
0 & 1 & 0 & -z^c_j & 0 & x^c_j\\ 
0 & 0 & 1 & y^c_j & -x^c_j & 0\\ 
0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix}
\end{aligned}
$$

$$
\begin{aligned}
\frac{\partial \rho_{j}}{\partial \xi_{ji}} &= \frac{\partial \frac{1}{z^c_j}}{\partial \xi_{ji}} \\
&= \frac{\partial \frac{1}{z^c_j}}{\partial z^c_j}\frac{\partial z^c_j}{\partial \xi_{ji}} \\
&= -\frac{1}{(z^c_j)^2}\begin{bmatrix}
0 & 0 & 1 & y^c_j & -x^c_j & 0 
\end{bmatrix} \\
&= \begin{bmatrix}
0 & 0 & -\rho^2_j & -\rho_{j} y^n_j & \rho_{j} x^n_j & 0 
\end{bmatrix}
\end{aligned}
$$

Thus, 

$$
\begin{aligned}
\frac{\partial \mathbf{p}^n_j}{\partial \xi_{ji}} &= \frac{\partial (\rho_{j}\mathbf{p}^c_j)}{\partial \xi_{ji}} \\
&= \frac{\partial \mathbf{p}^c_j}{\partial \xi_{ji}} \rho_{j} +\frac{\partial \rho_{j}}{\partial \xi_{ji}} \mathbf{p}^c_j \\
&= \begin{bmatrix}
\rho_{j} & 0 & 0 & 0 & 1 & -y^n_j\\ 
0 & \rho_{j} & 0 & -1 & 0 & x^n_j\\ 
0 & 0 & \rho_{j} & y^n_j & -x^n_j & 0
\end{bmatrix} + \begin{bmatrix}
0 & 0 & -\rho_{j}x^{n}_{j} & -x^{n}_{j}y^{n}_{j} & (x^n_j)^2 & 0\\ 
0 & 0 & -\rho_{j}y^{n}_{j}  & -(y^n_j)^2 & x^{n}_{j}y^{n}_{j} & 0\\ 
0 & 0 & -\rho_{j} & -y^n_j & x^n_j & 0
\end{bmatrix} \\
&= \begin{bmatrix}
\rho_{j} & 0 & -\rho_{j}x^{n}_{j} & -x^{n}_{j}y^{n}_{j} & 1 + (x^n_j)^2 & -y^n_j\\ 
0 & \rho_{j} & -\rho_{j}y^{n}_{j} & -1-(y^n_j)^2 & x^{n}_{j}y^{n}_{j} & x^n_j\\ 
0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix}
\end{aligned}
$$

Therefore, 

$$
\begin{aligned}
\frac{\partial r_{ji}}{\partial \xi_{ji}} &= \frac{\partial r_{ji}}{\partial \mathbf{p}_{j}} \frac{\partial \mathbf{p}_{j}}{\partial \xi_{ji}} \\
&= \frac{\partial r_{ji}}{\partial \mathbf{p}_{j}} \frac{\partial \mathbf{p}_{j}}{\partial \mathbf{p}^n_{j}} \frac{\partial \mathbf{p}^n_{j}}{\partial \xi_{ji}} \\
&= \omega_{h}
\begin{bmatrix}
g^j_x & g^j_y
\end{bmatrix} 
\begin{bmatrix}
f_x & 0 & 0\\ 
0 & f_y & 0
\end{bmatrix}
\begin{bmatrix}
\rho_{j} & 0 & -\rho_{j}x^{n}_{j} & -x^{n}_{j}y^{n}_{j} & 1 + (x^n_j)^2 & -y^n_j\\ 
0 & \rho_{j} & -\rho_{j}y^{n}_{j} & -1-(y^n_j)^2 & x^{n}_{j}y^{n}_{j} & x^n_j\\ 
0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix} \\ 
&=
\begin{bmatrix}
\omega_{h} g^j_x f_x & \omega_{h} g^j_y f_y & 0
\end{bmatrix}
\begin{bmatrix}
\rho_{j} & 0 & -\rho_{j}x^{n}_{j} & -x^{n}_{j}y^{n}_{j} & 1 + (x^n_j)^2 & -y^n_j\\ 
0 & \rho_{j} & -\rho_{j}y^{n}_{j} & -1-(y^n_j)^2 & x^{n}_{j}y^{n}_{j} & x^n_j\\ 
0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix} \\ 
&= \begin{bmatrix}
\rho_{j} m_x  \\ \rho_{j} m_y  \\ -\rho_{j} (m_x x^n_j + m_y y^n_j) \\ - m_x x^n_j y^n_j - m_y(1 + (y^n_j)^2)  \\ m_x(1+(x^n_j)^2) + m_y x^{n}_{j}y^{n}_{j}  \\ -m_x y^{n}_{j} + m_y x^{n}_{j}
\end{bmatrix}^T
\end{aligned} 
$$ where 
$$m_x = \omega_{h} g^j_x f_x, \quad m_y = \omega_{h} g^j_y f_y
$$

#### Affine Parameters

$$
\begin{aligned}
\frac{\partial r_{ji}}{\partial \begin{bmatrix} a_{ji} \\ b_{ji}\end{bmatrix}} &= \frac{\partial (\omega_{h} (I_{j}\left[\mathbf{p}_{j}\right]-e^{a_{ji}}I_{i}[\mathbf{p}_{i}] - b_{ji}))}{\partial \begin{bmatrix} a_{ji} \\ b_{ji}\end{bmatrix}} \\
&= \begin{bmatrix} -\omega_{h}I_{i}[\mathbf{p}_{i}]e^{a_{ji}} \\ -\omega_{h}\end{bmatrix}^T
\end{aligned}
$$

#### Inverse Depth

$$
\begin{aligned}
\frac{\partial r_{ji}}{\partial \rho_{i}} &= \frac{\partial r_{ji}}{\partial \mathbf{p}_{j}} \frac{\partial \mathbf{p}_{j}}{\partial \rho_{i}} \\ &= \frac{\partial r_{ji}}{\partial \mathbf{p}_{j}} \frac{\partial \mathbf{p}_{j}}{\partial \mathbf{p}^n_{j}}\frac{\partial \mathbf{p}^n_{j}}{\partial \rho_{i}}
\end{aligned}
$$

Therefore, we just need to compute the last part 
$$\frac{\partial \mathbf{p}^n_{j}}{\partial \rho_{i}}
$$

Due to the fact

$$\begin{aligned}
\mathbf{p}^n_{j} &= \rho_{j} \left ( \frac{1}{\rho_{i}} \mathbf{R}_{ji}\mathbf{K}^{-1}\mathbf{p}_i + \mathbf{t}_{ji}\right ) \\
&= \rho_{j} \left ( \rho^{-1}_{i} \mathbf{M} \mathbf{p}_i + \mathbf{t}_{ji}\right )
\end{aligned}
$$

| Variable                                                                 | Meaning                                |
| ------------------------------------------------------------------------ | -------------------------------------- |
| $$\mathbf{p}^n_{j} = \begin{bmatrix} x^n_j \\ y^n_j \\ 1 \end{bmatrix}$$ | point position in the normalized plane |
| $$\mathbf{M} = \mathbf{R}_{ji}\mathbf{K}^{-1}$$                          |                                        |
| $$\mathbf{t}_{ji} = \begin{bmatrix} t^x \\ t^y \\ t^z \end{bmatrix}$$    | translation from $$i$$ to $$j$$        |

Then, we have 

$$ \begin{bmatrix} x^n_j \\ y^n_j \\ 1 \end{bmatrix} =
\rho_{j} 
\begin{bmatrix}
\rho^{-1}_{i} \mathbf{M}^x \mathbf{p}_i + t^x \\
\rho^{-1}_{i} \mathbf{M}^y \mathbf{p}_i + t^y \\
\rho^{-1}_{i} \mathbf{M}^z \mathbf{p}_i + t^z
\end{bmatrix} 
$$

So

$$
\rho_{j} = \frac{1}{\rho^{-1}_{i} \mathbf{M}^z \mathbf{p}_i + t^z}
$$

$$
x^n_j = \frac{\rho^{-1}_{i} \mathbf{M}^x \mathbf{p}_i + t^x}{\rho^{-1}_{i} \mathbf{M}^z \mathbf{p}_i + t^z} = \frac{\mathbf{M}^x \mathbf{p}_i + t^x \rho_{i}}{\mathbf{M}^z \mathbf{p}_i + t^z \rho_{i}}
$$

$$
y^n_j = \frac{\rho^{-1}_{i} \mathbf{M}^y \mathbf{p}_i + t^y}{\rho^{-1}_{i} \mathbf{M}^z \mathbf{p}_i + t^z} = \frac{\mathbf{M}^y \mathbf{p}_i + t^y \rho_{i}}{\mathbf{M}^z \mathbf{p}_i + t^z \rho_{i}}
$$

Compute the parital derivatives 

$$\begin{aligned}
\frac{\partial x^n_j}{\partial \rho_i} &= \frac{t^x}{\mathbf{M}^z \mathbf{p}_i + t^z \rho_{i}} - (\mathbf{M}^x \mathbf{p}_i + t^x \rho_{i}) \frac{1}{(\mathbf{M}^z \mathbf{p}_i + t^z \rho_{i})^2} t^z \\ 
&= \frac{1}{\mathbf{M}^z \mathbf{p}_i + t^z \rho_{i}}(t^x - \frac{\mathbf{M}^x \mathbf{p}_i + t^x \rho_{i}}{\mathbf{M}^z \mathbf{p}_i + t^z \rho_{i}} t^z) \\
&= \frac{1}{\rho_{i}} \frac{1}{\rho^{-1}_{i} \mathbf{M}^z \mathbf{p}_i + t^z}(t^x - x^n_j t^z) \\ 
&= \frac{\rho_j}{\rho_i}(t^x - x^n_j t^z) 
\end{aligned}
$$

$$\begin{aligned}
\frac{\partial y^n_j}{\partial \rho_i} &= \frac{t^y}{\mathbf{M}^z \mathbf{p}_i + t^z \rho_{i}} - (\mathbf{M}^y \mathbf{p}_i + t^y \rho_{i}) \frac{1}{(\mathbf{M}^z \mathbf{p}_i + t^z \rho_{i})^2} t^z \\ 
&= \frac{1}{\mathbf{M}^z \mathbf{p}_i + t^z \rho_{i}}(t^y - \frac{\mathbf{M}^y \mathbf{p}_i + t^y \rho_{i}}{\mathbf{M}^z \mathbf{p}_i + t^z \rho_{i}} t^z) \\
&= \frac{1}{\rho_{i}} \frac{1}{\rho^{-1}_{i} \mathbf{M}^z \mathbf{p}_i + t^z}(t^y - y^n_j t^z) \\ 
&= \frac{\rho_j}{\rho_i}(t^y - y^n_j t^z) 
\end{aligned}
$$

Therefore,

$$\begin{aligned}
\frac{\partial r_{ji}}{\partial \rho_{i}} &= \frac{\partial r_{ji}}{\partial \mathbf{p}_{j}} \frac{\partial \mathbf{p}_{j}}{\partial \rho_{i}} \\ &= \frac{\partial r_{ji}}{\partial \mathbf{p}_{j}} \frac{\partial \mathbf{p}_{j}}{\partial \mathbf{p}^n_{j}}\frac{\partial \mathbf{p}^n_{j}}{\partial \rho_{i}} \\ 
&= \omega_{h}
\begin{bmatrix}
g^j_x & g^j_y
\end{bmatrix} 
\begin{bmatrix}
f_x & 0 & 0\\ 
0 & f_y & 0
\end{bmatrix}\begin{bmatrix}
\frac{\rho_j}{\rho_i}(t^x - x^n_j t^z)  \\ 
\frac{\rho_j}{\rho_i}(t^y - y^n_j t^z) \\
0
\end{bmatrix} \\ 
&= 
m_x \frac{\rho_j}{\rho_i}(t^x - x^n_j t^z) +
m_y \frac{\rho_j}{\rho_i}(t^y - y^n_j t^z)
\end{aligned}
$$ 

where 
$$m_x = \omega_{h} g^j_x f_x, \quad m_y = \omega_{h} g^j_y f_y
$$

#### Codes

| Variable used in codes          | Variable used here      | Meaning      |
| -------- | ----------------------------- |-------- |
|`residual` | $$I_{j}\left[\mathbf{p}_{j}\right]-e^{a_{ji}}I_{i}[\mathbf{p}_{i}] - b_{ji}$$| 单个点的误差 |
| `hw` | $$\omega_{h}$$ |Huber weight |
| `refToNew_aff_current` | $$\begin{bmatrix}a_{ji} \\ b_{ji}\end{bmatrix}$$ | 光度变化系数 |
| `r2new_aff` | $$\begin{bmatrix}e^{a_{ji}} \\ b_{ji}\end{bmatrix}$$ |光度变化系数　|
| `dp0` | $$\frac{\partial r_{ji}}{\partial t_{ji}^x}$$ | 对`translation`的偏导　|
| `dp1` | $$\frac{\partial r_{ji}}{\partial t_{ji}^y}$$ | 对`translation`的偏导　|
| `dp2` | $$\frac{\partial r_{ji}}{\partial t_{ji}^z}$$ | 对`translation`的偏导　|
| `dp3` | $$\frac{\partial r_{ji}}{\partial \phi_{ji}^x}$$ | 对`rotation`的偏导 |
| `dp4` | $$\frac{\partial r_{ji}}{\partial \phi_{ji}^y}$$ | 对`rotation`的偏导 |
| `dp5` | $$\frac{\partial r_{ji}}{\partial \phi_{ji}^z}$$ | 对`rotation`的偏导 |
| `dp6` | $$\frac{\partial r_{ji}}{\partial a_{ji}}$$ | 对光度变化系数的偏导|
| `dp7` | $$\frac{\partial r_{ji}}{\partial b_{ji}}$$ | 对光度变化系数的偏导|
| `dd` | $$\frac{\partial r_{ji}}{\partial \rho_{i}}$$ |对逆深度的偏导|
| `r` | $$r_{ji}$$ | 使用Huber kernel后单个点的误差|


#### Complete Jacobian

- For each optimization, we have total `8 + N` variables to optmize, i.e., `6` for the relative pose $$\mathbf{T}_{ji}$$, `2` for the relative photometric parameters $$a_{ji}, b_{ji}$$, `N` for inverse depths of `N` points of interest.

- Considering a non-linear least squares formualtion, we have $$E(\mathbf{x}) = \mathbf{e}^T\mathbf{e}$$

| Variable| Meaning|
| ---| --- |
| $$E$$ | Total energy to minimize |
| $$\mathbf{e} = \begin{bmatrix}r_{1} \\ r_{2} \\ . \\.\\. \\ r_{N} \end{bmatrix}$$  |  residual vector comprised of `N` points' residuals  |
|$$\mathbf{x} = \begin{bmatrix} t^x \\t^y \\t^z \\ \phi^x \\\phi^y\\\phi^z \\ a \\ b \\ \rho_1 \\ \rho_2 \\ . \\ . \\. \\ \rho_N \end{bmatrix}$$| variables to optimize (`8 + N`)|

- __Complete Jacobian__ ($$N \times (8 + N)$$)

$$
\mathbf{J}=\begin{bmatrix}
\frac{\partial r_{1}}{\partial t^x} & \frac{\partial r_{1}}{\partial t^y} & \frac{\partial r_{1}}{\partial t^z} & \frac{\partial r_{1}}{\partial \phi^x} & \frac{\partial r_{1}}{\partial \phi^y} & \frac{\partial r_{1}}{\partial \phi^z} & \frac{\partial r_{1}}{\partial a} & \frac{\partial r_{1}}{\partial b} & \frac{\partial r_{1}}{\partial \rho_{1}} & 0 & . & . & . & 0 \\ 
\frac{\partial r_{2}}{\partial t^x} & \frac{\partial r_{2}}{\partial t^y} & \frac{\partial r_{2}}{\partial t^z} & \frac{\partial r_{2}}{\partial \phi^x} & \frac{\partial r_{2}}{\partial \phi^y} & \frac{\partial r_{2}}{\partial \phi^z} & \frac{\partial r_{2}}{\partial a} & \frac{\partial r_{2}}{\partial b} & 0 & \frac{\partial r_{2}}{\partial \rho_{2}} & . & . & . & 0 \\
. & . & . & . & . & . & . & . & . & . & . & . & . & . \\
. & . & . & . & . & . & . & . & . & . & . & . & . & . \\
. & . & . & . & . & . & . & . & . & . & . & . & . & . \\
\frac{\partial r_{N}}{\partial t^x} & \frac{\partial r_{N}}{\partial t^y} & \frac{\partial r_{N}}{\partial t^z} & \frac{\partial r_{N}}{\partial \phi^x} & \frac{\partial r_{N}}{\partial \phi^y} & \frac{\partial r_{N}}{\partial \phi^z} & \frac{\partial r_{N}}{\partial a} & \frac{\partial r_{N}}{\partial b} & 0 & 0 & . & . & . & \frac{\partial r_{N}}{\partial \rho_{N}} 
\end{bmatrix}
$$


### Photometric Error (Plane)

#### Plane [a, b, c, d]

### Pose Graph Error

#### Similarity Sim(3)

### Literature

[^Hartley04]: R. Hartley and A. Zisserman, Multiple View Geometry in Computer Vision, 2nd ed. Cambridge University Press, 2004.

[^Engel18]: Engel, Jakob, Vladlen Koltun, and Daniel Cremers. "Direct Sparse Odometry." IEEE Transactions on Pattern Analysis and Machine Intelligence 40.3 (2018): 611-625.

[^Barfoot17]: Timothy D Barfoot. State Estimation for Robotics. Cambridge University Press, 2017.