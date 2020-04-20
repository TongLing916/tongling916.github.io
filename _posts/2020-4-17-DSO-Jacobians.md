---
layout:     post
title:      "DSO - Jacobians"
date:       2020-4-17
author:     Tong
catalog: true
tags:
    - DSO
---

> [Direct sparse odometry](https://vision.in.tum.de/research/vslam/dso)

> Foreknowledge about Lie groups and Lie algebra can be found [here](http://www.lingtong.de/2020/04/02/Jacobian-Matrices/)

### Photometric Error

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

| Variable                                              | Meaning                                                       |
| ----------------------------------------------------- | ------------------------------------------------------------- |
| $$e^{a_{ji}} = \frac{t_{j}}{t_{i}}e^{a_{j} - a_{i}}$$ | $$a_{ji}$$ is an affine tranfer parameter from $$i$$ to $$j$$ |
| $$a_{ji} = \ln(\frac{t_{j}}{t_{i}}) + a_{j} - a_{i}$$ | $$a_{ji}$$ is an affine tranfer parameter from $$i$$ to $$j$$ |
| $$b_{ji} = b_{j} - e^{a_{ji}}b_{i}$$                  | $$b_{ji}$$ is an affine tranfer parameter from $$i$$ to $$j$$ |
| $$\omega_{h}$$                                        | Huber weight, considered a constant                           |

### Preparation

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


### Relative Camera Pose

$$
\frac{\partial r_{ji}}{\partial \boldsymbol{\xi}_{ji}} = \frac{\partial r_{ji}}{\partial \mathbf{p}_{j}} \frac{\partial \mathbf{p}_{j}}{\partial \boldsymbol{\xi}_{ji}}
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
\frac{\partial \mathbf{p}_{j}}{\partial \boldsymbol{\xi}_{ji}} = \frac{\partial \mathbf{p}_{j}}{\partial \mathbf{p}^n_j} \frac{\partial \mathbf{p}^n_j}{\partial \boldsymbol{\xi}_{ji}}
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
\frac{\partial \mathbf{p}^n_j}{\partial \boldsymbol{\xi}_{ji}} &= \frac{\partial (\rho_{j}\mathbf{p}^c_j)}{\partial \boldsymbol{\xi}_{ji}} \\
&= \frac{\partial \mathbf{p}^c_j}{\partial \boldsymbol{\xi}_{ji}} \rho_{j} +\mathbf{p}^c_j \frac{\partial \rho_{j}}{\partial \boldsymbol{\xi}_{ji}} 
\end{aligned}
$$

where 

$$
\begin{aligned}
\frac{\partial \mathbf{p}^c_j}{\partial \boldsymbol{\xi}_{ji}} &= \frac{\partial (\mathbf{T}_{ji}\mathbf{p}^c_i)}{\partial \boldsymbol{\xi}_{ji}} \\
&= \lim_{\delta \boldsymbol{\xi} \rightarrow 0} \frac{\exp{(\delta\boldsymbol{\xi}^{\wedge})}\exp{(\boldsymbol{\xi}_{ji}^{\wedge})}\mathbf{p}^c_i -\exp{(\boldsymbol{\xi}_{ji}^{\wedge})}\mathbf{p}^c_i }{\delta \boldsymbol{\xi}} \\
&= \lim_{\delta \boldsymbol{\xi} \rightarrow 0} \frac{(\mathbf{I} + \delta \boldsymbol{\xi}^{\wedge})\exp{(\boldsymbol{\xi}_{ji}^{\wedge})}\mathbf{p}^c_i -\exp{(\boldsymbol{\xi}_{ji}^{\wedge})}\mathbf{p}^c_i }{\delta \boldsymbol{\xi}}\\
&= \lim_{\delta \boldsymbol{\xi} \rightarrow 0} \frac{\delta \boldsymbol{\xi}^{\wedge}\exp{(\boldsymbol{\xi}_{ji}^{\wedge})}\mathbf{p}^c_i }{\delta \boldsymbol{\xi}}\\
&= \lim_{\delta \boldsymbol{\xi} \rightarrow 0} \frac{(\exp{(\boldsymbol{\xi}_{ji}^{\wedge})}\mathbf{p}^c_i)^{\odot}\delta \boldsymbol{\xi} }{\delta \boldsymbol{\xi}}\\
&= (\exp{(\boldsymbol{\xi}_{ji}^{\wedge})}\mathbf{p}^c_i)^{\odot} \\
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
\frac{\partial \rho_{j}}{\partial \boldsymbol{\xi}_{ji}} &= \frac{\partial \frac{1}{z^c_j}}{\partial \boldsymbol{\xi}_{ji}} \\
&= \frac{\partial \frac{1}{z^c_j}}{\partial z^c_j}\frac{\partial z^c_j}{\partial \boldsymbol{\xi}_{ji}} \\
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
\frac{\partial \mathbf{p}^n_j}{\partial \boldsymbol{\xi}_{ji}} &= \frac{\partial (\rho_{j}\mathbf{p}^c_j)}{\partial \boldsymbol{\xi}_{ji}} \\
&= \frac{\partial \mathbf{p}^c_j}{\partial \boldsymbol{\xi}_{ji}} \rho_{j} +\frac{\partial \rho_{j}}{\partial \boldsymbol{\xi}_{ji}} \mathbf{p}^c_j \\
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
\frac{\partial r_{ji}}{\partial \boldsymbol{\xi}_{ji}} &= \frac{\partial r_{ji}}{\partial \mathbf{p}_{j}} \frac{\partial \mathbf{p}_{j}}{\partial \boldsymbol{\xi}_{ji}} \\
&= \frac{\partial r_{ji}}{\partial \mathbf{p}_{j}} \frac{\partial \mathbf{p}_{j}}{\partial \mathbf{p}^n_{j}} \frac{\partial \mathbf{p}^n_{j}}{\partial \boldsymbol{\xi}_{ji}} \\
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

### Absolute Camera Pose

$$\frac{\partial r_{ji}}{\partial \boldsymbol{\xi}_{iw}} = \frac{\partial r_{ji}}{\partial \boldsymbol{\xi}_{ji}}\frac{\partial \boldsymbol{\xi}_{ji}}{\partial \boldsymbol{\xi}_{iw}} $$

$$\frac{\partial r_{ji}}{\partial \boldsymbol{\xi}_{jw}} = \frac{\partial r_{ji}}{\partial \boldsymbol{\xi}_{ji}}\frac{\partial \boldsymbol{\xi}_{ji}}{\partial \boldsymbol{\xi}_{jw}} $$

As we have computed $$\frac{\partial r_{ji}}{\partial \boldsymbol{\xi}_{ji}}$$ before, we just need to compute $$\frac{\partial \boldsymbol{\xi}_{ji}}{\partial \boldsymbol{\xi}_{iw}}$$ and $$\frac{\partial \boldsymbol{\xi}_{ji}}{\partial \boldsymbol{\xi}_{jw}}$$ here.

$$
\begin{aligned}
\frac{\partial \boldsymbol{\xi}_{ji}}{\partial \boldsymbol{\xi}_{iw}} &= \lim_{\delta \boldsymbol{\xi}_{iw} \rightarrow 0} \frac{ \delta \boldsymbol{\xi}_{ji}}{ \delta \boldsymbol{\xi}_{iw}} \\
&= \lim_{\delta \boldsymbol{\xi}_{iw} \rightarrow 0} \frac{ \ln{(\exp{(\delta \boldsymbol{\xi}_{ji})}^{\wedge})^{\vee}}}{ \delta \boldsymbol{\xi}_{iw}} \\
&= \lim_{\delta \boldsymbol{\xi}_{iw} \rightarrow 0} \frac{ \ln{(\mathbf{T}_{jw} (\exp{(\delta \boldsymbol{\xi}_{iw})}^{\wedge}\mathbf{T}_{iw})^{-1} (\mathbf{T}_{jw}\mathbf{T}_{iw}^{-1})^{-1})^{\vee}}}{ \delta \boldsymbol{\xi}_{iw}} \\
&= \lim_{\delta \boldsymbol{\xi}_{iw} \rightarrow 0} \frac{ \ln{(\mathbf{T}_{jw} \mathbf{T}_{iw}^{-1} \exp{(-\delta \boldsymbol{\xi}_{iw})}^{\wedge} (\mathbf{T}_{jw}\mathbf{T}_{iw}^{-1})^{-1})^{\vee}}}{ \delta \boldsymbol{\xi}_{iw}} \\
&= \lim_{\delta \boldsymbol{\xi}_{iw} \rightarrow 0} \frac{ \ln{(\mathbf{T}_{ji} \exp{(-\delta \boldsymbol{\xi}_{iw})}^{\wedge} \mathbf{T}_{ji}^{-1})^{\vee}}}{ \delta \boldsymbol{\xi}_{iw}} \\
&= \lim_{\delta \boldsymbol{\xi}_{iw} \rightarrow 0} \frac{ \ln{(\exp{(-\mathcal{T}_{ji}\delta \boldsymbol{\xi}_{iw})}^{\wedge})^{\vee}}}{ \delta \boldsymbol{\xi}_{iw}} \\
&= \lim_{\delta \boldsymbol{\xi}_{iw} \rightarrow 0} \frac{-\mathcal{T}_{ji}\delta \boldsymbol{\xi}_{iw}}{ \delta \boldsymbol{\xi}_{iw}} \\
&= -\mathcal{T}_{ji}
\end{aligned}
$$

$$
\begin{aligned}
\frac{\partial \boldsymbol{\xi}_{ji}}{\partial \boldsymbol{\xi}_{jw}} &= \lim_{\delta \boldsymbol{\xi}_{jw} \rightarrow 0} \frac{ \delta \boldsymbol{\xi}_{ji}}{ \delta \boldsymbol{\xi}_{jw}} \\
&= \lim_{\delta \boldsymbol{\xi}_{jw} \rightarrow 0} \frac{ \ln{(\exp{(\delta \boldsymbol{\xi}_{ji})}^{\wedge})^{\vee}}}{ \delta \boldsymbol{\xi}_{jw}} \\
&= \lim_{\delta \boldsymbol{\xi}_{jw} \rightarrow 0} \frac{ \ln{(\exp{(\delta \boldsymbol{\xi}_{jw})}^{\wedge}\mathbf{T}_{jw}\mathbf{T}_{iw}^{-1} (\mathbf{T}_{jw}\mathbf{T}_{iw}^{-1})^{-1})^{\vee}}}{ \delta \boldsymbol{\xi}_{jw}} \\
&= \lim_{\delta \boldsymbol{\xi}_{jw} \rightarrow 0} \frac{ \ln{(\exp{(\delta \boldsymbol{\xi}_{jw})}^{\wedge})^{\vee}}}{ \delta \boldsymbol{\xi}_{jw}} \\
&= \lim_{\delta \boldsymbol{\xi}_{jw} \rightarrow 0} \frac{\delta \boldsymbol{\xi}_{jw}}{ \delta \boldsymbol{\xi}_{jw}} \\
&= \mathbf{I}
\end{aligned}
$$

### Relative Photometric Parameters

$$
\begin{aligned}
\frac{\partial r_{ji}}{\partial \begin{bmatrix} a_{ji} \\ b_{ji}\end{bmatrix}} &= \frac{\partial (\omega_{h} (I_{j}\left[\mathbf{p}_{j}\right]-e^{a_{ji}}I_{i}[\mathbf{p}_{i}] - b_{ji}))}{\partial \begin{bmatrix} a_{ji} \\ b_{ji}\end{bmatrix}} \\
&= \begin{bmatrix} -\omega_{h}I_{i}[\mathbf{p}_{i}]e^{a_{ji}} \\ -\omega_{h}\end{bmatrix}^T
\end{aligned}
$$

### Absolute Photometric Parameters

In the real implementation of DSO, the derivative wrt. the absolute photometric values are computed in a __weird__ way as follows

$$
\begin{aligned}
\frac{\partial r_{ji}}{\partial \begin{bmatrix} a_{i} \\ b_{i}\end{bmatrix}} 
&= \frac{\partial r_{ji}}{\partial \mathbf{a}_{ji}}\frac{\partial \mathbf{a}_{ji}}{\partial \begin{bmatrix} a_{i} \\ b_{i}\end{bmatrix}} \\ 
&= \frac{\partial r_{ji}}{\partial \begin{bmatrix} -e^{a_{ji}} \\ -b_{ji}\end{bmatrix}}\frac{\partial \begin{bmatrix} -e^{a_{ji}} \\ -b_{ji}\end{bmatrix}}{\partial \begin{bmatrix} a_{i} \\ b_{i}\end{bmatrix}}
\end{aligned}
$$

Since 

$$e^{a_{ji}} = \frac{t_{j}}{t_{i}}e^{a_{j} - a_{i}}$$

$$b_{ji} = b_{j} - e^{a_{ji}}b_{i}$$

$$
r_{ji} = \omega_{h} (I_{j}\left[\mathbf{p}_{j}\right]-e^{a_{ji}}I_{i}[\mathbf{p}_{i}] - b_{ji})
$$

we have 

$$
\frac{\partial r_{ji}}{\partial \begin{bmatrix} -e^{a_{ji}} \\ -b_{ji}\end{bmatrix}} = \begin{bmatrix}\omega_{h}(I_{i}[\mathbf{p}_{i}] - b_{i}) & \omega_{h} \end{bmatrix}
$$

$$
\frac{\partial \begin{bmatrix} -e^{a_{ji}} \\ -b_{ji}\end{bmatrix}}{\partial \begin{bmatrix} a_{i} \\ b_{i}\end{bmatrix}} = \begin{bmatrix}e^{a_{ji}} & 0 \\ -e^{a_{ji}}b_{i} & e^{a_{ji}} \end{bmatrix}
$$

$$
\frac{\partial \begin{bmatrix} -e^{a_{ji}} \\ -b_{ji}\end{bmatrix}}{\partial \begin{bmatrix} a_{j} \\ b_{j}\end{bmatrix}} = \begin{bmatrix}-e^{a_{ji}} & 0 \\ e^{a_{ji}}b_{i} & -1\end{bmatrix}
$$


### Inverse Depth

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

### Intrinsic parameters

$$
\frac{\partial \mathbf{p}_{j}}{\partial \mathbf{C}} = \begin{bmatrix}
\frac{\partial u_{j}}{\partial f_{x}} & \frac{\partial u_{j}}{\partial f_{y}} & \frac{\partial u_{j}}{\partial c_{x}} & \frac{\partial u_{j}}{\partial c_{y}} \\
\frac{\partial v_{j}}{\partial f_{x}} & \frac{\partial v_{j}}{\partial f_{y}} & \frac{\partial v_{j}}{\partial c_{x}} & \frac{\partial v_{j}}{\partial c_{y}}
\end{bmatrix}
$$

Due to the fact that

$$
\mathbf{p}_j = \mathbf{K} \mathbf{p}^n_j = \begin{bmatrix}
f_x x^n_j + c_x\\ 
f_y y^n_j + c_y\\ 
1
\end{bmatrix}
$$

we have
$$
\begin{aligned}
\frac{\partial u_{j}}{\partial f_{x}} &= x^n_j + f_x \frac{\partial x^n_j}{\partial f_{x}} \\
\frac{\partial u_{j}}{\partial f_{y}} &= f_x \frac{\partial x^n_j}{\partial f_{y}} \\
\frac{\partial u_{j}}{\partial c_{x}} &= f_x \frac{\partial x^n_j}{\partial c_{x}} + 1 \\
\frac{\partial u_{j}}{\partial c_{y}} &= f_x \frac{\partial x^n_j}{\partial c_{y}} \\
\frac{\partial v_{j}}{\partial f_{x}} &= f_y \frac{\partial y^n_j}{\partial f_{y}} \\
\frac{\partial v_{j}}{\partial f_{y}} &= y^n_j + f_y \frac{\partial y^n_j}{\partial f_{y}} \\
\frac{\partial v_{j}}{\partial c_{x}} &= f_y \frac{\partial y^n_j}{\partial c_{x}} \\
\frac{\partial v_{j}}{\partial c_{y}} &= f_y \frac{\partial y^n_j}{\partial c_{y}} + 1
\end{aligned}
$$

Now, the important thing is to compute $$\frac{\partial \mathbf{p}^{n}_{j}}{\partial \mathbf{C}}$$.

Due to the fact 
$$
\begin{aligned}
\mathbf{p}^{n}_{j} &= \rho_{j} \left ( \frac{1}{\rho_{i}} \mathbf{R}_{ji}\mathbf{K}^{-1}\mathbf{p}_i + \mathbf{t}_{ji}\right ) \\
&= \frac{\rho_{j}}{\rho_{i}}\mathbf{R}_{ji}\mathbf{K}^{-1}\mathbf{p}_i + \rho_{j}\mathbf{t}_{ji} \\ 
&= \frac{\rho_{j}}{\rho_{i}}\begin{bmatrix}r_{00} & r_{00} & r_{01} \\ r_{10} & r_{11} & r_{12} \\ r_{20} & r_{21} & r_{22}\end{bmatrix}\begin{bmatrix}
f_{x}^{-1} & 0 & -f_{x}^{-1} c_{x} \\
0 & f_{y}^{-1} & -f_{y}^{-1} c_{y} \\
0 & 0 & 1
\end{bmatrix}\begin{bmatrix}
u_{i} \\
v_{i} \\
1
\end{bmatrix}+\rho_{j} \begin{bmatrix} t^x \\ t^y \\ t^z \end{bmatrix}
\end{aligned}
$$

then we have 

$$\begin{aligned}
&u_{j}^{n}=\frac{\frac{\rho_{j}}{\rho_{i}}\left(r_{00} f_{x}^{-1}\left(u_{i}-c_{x}\right)+r_{01} f_{y}^{-1}\left(v_{i}-c_{y}\right)+r_{02}\right)+\rho_{j} t_{21}^{x}}{\frac{\rho_{j}}{\rho_{i}}\left(r_{20} f_{x}^{-1}\left(u_{i}-c_{x}\right)+r_{21} f_{y}^{-1}\left(v_{i}-c_{y}\right)+r_{22}\right)+\rho_{j} t_{21}^{z}}=\frac{A}{C}\\
&v_{j}^{n}=\frac{\frac{\rho_{j}}{\rho_{i}}\left(r_{10} f_{x}^{-1}\left(u_{i}-c_{x}\right)+r_{11} f_{y}^{-1}\left(v_{i}-c_{y}\right)+r_{12}\right)+\rho_{j} t_{21}^{y}}{\frac{\rho_{j}}{\rho_{i}}\left(r_{20} f_{x}^{-1}\left(u_{i}-c_{x}\right)+r_{21} f_{y}^{-1}\left(v_{i}-c_{y}\right)+r_{22}\right)+\rho_{j} t_{21}^{z}}=\frac{B}{C}
\end{aligned}$$

Actually, $$C = 1$$, but we still need to consider this part while computing Jacobian.

$$\begin{aligned}
\frac{\partial u_{j}^{n}}{\partial f_{x}} &=\frac{\partial A}{\partial f_{x}} \frac{1}{C}+A \frac{1}{C^{2}}(-1) \frac{\partial C}{\partial f_{x}} \\
&=\frac{\rho_{j}}{\rho_{i}} r_{00}\left(u_{i}-c_{x}\right) f_{x}^{-2}(-1) \frac{1}{C}-\frac{A}{C} \frac{1}{C} \frac{\rho_{j}}{\rho_{i}} r_{20}\left(u_{i}-c_{x}\right) f_{x}^{-2}(-1) \\
&=\frac{1}{C}\left(\frac{\rho_{j}}{\rho_{i}} r_{00}\left(u_{i}-c_{x}\right) f_{x}^{-2}(-1)+\frac{\rho_{j}}{\rho_{i}} r_{20}\left(u_{i}-c_{x}\right) f_{x}^{-2} u_{j}^{n}\right) \\
&=\frac{\rho_{j}}{\rho_{i}}\left(r_{20} u_{j}^{n}-r_{00}\right) f_{x}^{-2}\left(u_{i}-c_{x}\right)
\end{aligned}$$

Similarly, we have

$$\begin{aligned}
\frac{\partial u_{j}^{n}}{\partial f_{x}}&=\frac{\rho_{j}}{\rho_{i}}\left(r_{20} u_{j}^{n}-r_{00}\right) f_{x}^{-2}\left(u_{i}-c_{x}\right) \\ \frac{\partial u_{j}^{n}}{\partial f_{y}}&=\frac{\rho_{j}}{\rho_{i}}\left(r_{21} u_{j}^{n}-r_{01}\right) f_{y}^{-2}\left(v_{i}-c_{y}\right)\\
\frac{\partial u_{j}^{n}}{\partial c_{x}}&=\frac{\rho_{j}}{\rho_{i}}\left(r_{20} u_{j}^{n}-r_{00}\right) f_{x}^{-1} \\ \frac{\partial u_{j}^{n}}{\partial c_{y}}&=\frac{\rho_{j}}{\rho_{i}}\left(r_{21} u_{j}^{n}-r_{01}\right) f_{y}^{-1}\\
\frac{\partial v_{j}^{n}}{\partial f_{x}}&=\frac{\rho_{j}}{\rho_{i}}\left(r_{20} v_{j}^{n}-r_{10}\right) f_{x}^{-2}\left(u_{i}-c_{x}\right) \\ \frac{\partial v_{j}^{n}}{\partial f_{y}}&=\frac{\rho_{j}}{\rho_{i}}\left(r_{21} v_{j}^{n}-r_{11}\right) f_{y}^{-2}\left(v_{i}-c_{y}\right)\\
\frac{\partial v_{j}^{n}}{\partial c_{x}}&=\frac{\rho_{j}}{\rho_{i}}\left(r_{20} v_{j}^{n}-r_{10}\right) f_{x}^{-1} \\ \frac{\partial v_{j}^{n}}{\partial c_{y}}&=\frac{\rho_{j}}{\rho_{i}}\left(r_{21} v_{j}^{n}-r_{11}\right) f_{y}^{-1}
\end{aligned}$$

Finally, we have

$$\begin{aligned}
\frac{\partial u_{j}}{\partial f_{x}} &=u_{j}^{n}+f_{x} \frac{\partial u_{j}^{n}}{\partial f_{x}} \\
&=u_{j}^{n}+\frac{\rho_{j}}{\rho_{i}}\left(r_{20} u_{j}^{n}-r_{00}\right) f_{x}^{-1}\left(u_{i}-c_{x}\right) \\
\frac{\partial u_{j}}{\partial f_{y}} &=f_{x} \frac{\partial u_{j}^{n}}{\partial f_{y}} \\
&=\frac{f_{x}}{f_{y}} \frac{\rho_{j}}{\rho_{i}}\left(r_{21} u_{j}^{n}-r_{01}\right) f_{y}^{-1}\left(v_{i}-c_{y}\right) \\
\frac{\partial u_{j}}{\partial c_{x}} &=f_{x} \frac{\partial u_{j}^{n}}{\partial c_{x}}+1 \\
&=\frac{\rho_{j}}{\rho_{i}}\left(r_{20} u_{j}^{n}-r_{00}\right)+1 \\
\frac{\partial u_{j}}{\partial c_{y}} &=f_{x} \frac{\partial u_{j}^{n}}{\partial c_{y}} \\
&=\frac{f_{x}}{f_{y}} \frac{\rho_{j}}{\rho_{i}}\left(r_{21} u_{j}^{n}-r_{01}\right) \\
\frac{\partial v_{j}}{\partial f_{x}} &=f_{y} \frac{\partial v_{j}^{n}}{\partial f_{x}} \\
&=\frac{f_{y}}{f_{x}} \frac{\rho_{j}}{\rho_{i}}\left(r_{20} v_{j}^{n}-r_{10}\right) f_{x}^{-1}\left(u_{i}-c_{x}\right) \\
\frac{\partial v_{j}}{\partial f_{y}} &=v_{j}^{n}+f_{y} \frac{\partial v_{j}^{n}}{\partial f_{y}} \\
&=v_{j}^{n}+\frac{\rho_{j}}{\rho_{i}}\left(r_{21} v_{j}^{n}-r_{11}\right) f_{y}^{-1}\left(v_{i}-c_{y}\right) \\
\frac{\partial v_{j}}{\partial c_{x}} &=f_{y} \frac{\partial v_{j}^{n}}{\partial c_{x}} \\
&=\frac{f_{y}}{f_{x}} \frac{\rho_{j}}{\rho_{i}}\left(r_{20} v_{j}^{n}-r_{10}\right) \\
\frac{\partial v_{j}}{\partial c_{y}} &=f_{y} \frac{\partial v_{j}^{n}}{\partial c_{y}}+1 \\
&=\frac{\rho_{j}}{\rho_{i}}\left(r_{21} v_{j}^{n}-r_{11}\right)+1
\end{aligned}$$


### Complete Jacobian

#### Initialization

- For each optimization, we have total `8 + N` variables to optmize, i.e., `6` for the relative pose $$\mathbf{T}_{ji}$$, `2` for the relative photometric parameters $$a_{ji}, b_{ji}$$, `N` for inverse depths of `N` points of interest.

- Considering a non-linear least squares formualtion, we have $$E(\mathbf{x}) = \mathbf{e}^T\mathbf{e}$$

| Variable                                                                                                                                        | Meaning                                            |
| ----------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------- |
| $$E$$                                                                                                                                           | Total energy to minimize                           |
| $$\mathbf{e} = \begin{bmatrix}r_{1} \\ r_{2} \\ . \\.\\. \\ r_{N} \end{bmatrix}$$                                                               | residual vector comprised of `N` points' residuals |
| $$\mathbf{x} = \begin{bmatrix} t^x \\t^y \\t^z \\ \phi^x \\\phi^y\\\phi^z \\ a \\ b \\ \rho_1 \\ \rho_2 \\ . \\ . \\. \\ \rho_N \end{bmatrix}$$ | variables to optimize (`8 + N`)                    |



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


#### Sliding Window Optimization

Suppose that we have totally `M` frames, `N` points, `L` residuals, then our Jacobian is a $$L \times (4 + 8 * M + N)$$ matrix, because a camera has `4` intrinsic paramters ($$f_x, f_y, c_x, c_y$$), every frame has a pose (`6` DoF) and `2` photometric parameters ($$a, b$$), and every point has `1` inverse depth wrt. its host frame (in which it was first time observed).