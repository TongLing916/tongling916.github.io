---
layout:     post
title:      "DSO - Schur Complement"
date:       2020-4-17
author:     Tong
catalog: true
tags:
    - DSO
---

> [Direct sparse odometry](https://vision.in.tum.de/research/vslam/dso)

> Foreknowledge about Schur complement can be found [here](http://tongling916.github.io/2020/04/02/Schur-Complement/)

### Initialization

#### Preparation

As introduced [here](http://tongling916.github.io/2020/04/02/Jacobian-Matrices/), we have 

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

$$\mathbf{e} = \begin{bmatrix}r_{1} \\ r_{2} \\ . \\.\\. \\ r_{N} \end{bmatrix}$$

$$\mathbf{x} = \begin{bmatrix} t^x \\t^y \\t^z \\ \phi^x \\\phi^y\\\phi^z \\ a \\ b \\ \rho_1 \\ \rho_2 \\ . \\ . \\. \\ \rho_N \end{bmatrix} = \begin{bmatrix}\mathbf{x}_1 \\ \mathbf{x}_{2} \end{bmatrix}$$, where $$\mathbf{x}_1 = \begin{bmatrix}t^x \\t^y \\t^z \\ \phi^x \\\phi^y\\\phi^z \\ a \\ b \end{bmatrix}$$, $$\mathbf{x}_2 = \begin{bmatrix}\rho_1 \\ \rho_2 \\ . \\ . \\. \\ \rho_N \end{bmatrix}$$

In Gauss-Newton, our Hessian matrix can be formulated as 

$$
\begin{aligned}
\mathbf{H} &= \mathbf{J}^T\mathbf{J} \\
&= 
\begin{bmatrix}
\frac{\partial r_{1}}{\partial t^x} & \frac{\partial r_{2}}{\partial t^x} & . & . & . & \frac{\partial r_{N}}{\partial t^x}\\ 
\frac{\partial r_{1}}{\partial t^y} & \frac{\partial r_{2}}{\partial t^y} & . & . & . & \frac{\partial r_{N}}{\partial t^y} 
\\ \frac{\partial r_{1}}{\partial t^z} & \frac{\partial r_{2}}{\partial t^z} & . & . & . & \frac{\partial r_{N}}{\partial t^z} 
\\ \frac{\partial r_{1}}{\partial \phi^x} & \frac{\partial r_{2}}{\partial \phi^x} & . & . & . & \frac{\partial r_{N}}{\partial \phi^x} 
\\ \frac{\partial r_{1}}{\partial \phi^y} & \frac{\partial r_{2}}{\partial \phi^y} & . & . & . & \frac{\partial r_{N}}{\partial \phi^y} 
\\ \frac{\partial r_{1}}{\partial \phi^z} & \frac{\partial r_{2}}{\partial \phi^z} & . & . & . & \frac{\partial r_{N}}{\partial \phi^z} 
\\ \frac{\partial r_{1}}{\partial a} & \frac{\partial r_{2}}{\partial a} & . & . & . & \frac{\partial r_{N}}{\partial a} 
\\ \frac{\partial r_{1}}{\partial b} & \frac{\partial r_{2}}{\partial b} & . & . & . & \frac{\partial r_{N}}{\partial b}
\\ \frac{\partial r_{1}}{\partial \rho_{1}}  & 0 & . &. & . & 0
\\ 0 & \frac{\partial r_{2}}{\partial \rho_{2}}  & . & . & . & 0\\ 0 & . & . & .& . & 0 \\ 0 & . &. & .& . & 0\\ 0 & . &. & . & .& 0\\ 0 & . &. & . & 0 & \frac{\partial r_{N}}{\partial \rho_{N}} 
\end{bmatrix}
\begin{bmatrix}
\frac{\partial r_{1}}{\partial t^x} & \frac{\partial r_{1}}{\partial t^y} & \frac{\partial r_{1}}{\partial t^z} & \frac{\partial r_{1}}{\partial \phi^x} & \frac{\partial r_{1}}{\partial \phi^y} & \frac{\partial r_{1}}{\partial \phi^z} & \frac{\partial r_{1}}{\partial a} & \frac{\partial r_{1}}{\partial b} & \frac{\partial r_{1}}{\partial \rho_{1}} & 0 & . & . & . & 0 \\ 
\frac{\partial r_{2}}{\partial t^x} & \frac{\partial r_{2}}{\partial t^y} & \frac{\partial r_{2}}{\partial t^z} & \frac{\partial r_{2}}{\partial \phi^x} & \frac{\partial r_{2}}{\partial \phi^y} & \frac{\partial r_{2}}{\partial \phi^z} & \frac{\partial r_{2}}{\partial a} & \frac{\partial r_{2}}{\partial b} & 0 & \frac{\partial r_{2}}{\partial \rho_{2}} & . & . & . & 0 \\
. & . & . & . & . & . & . & . & . & . & . & . & . & . \\
. & . & . & . & . & . & . & . & . & . & . & . & . & . \\
. & . & . & . & . & . & . & . & . & . & . & . & . & . \\
\frac{\partial r_{N}}{\partial t^x} & \frac{\partial r_{N}}{\partial t^y} & \frac{\partial r_{N}}{\partial t^z} & \frac{\partial r_{N}}{\partial \phi^x} & \frac{\partial r_{N}}{\partial \phi^y} & \frac{\partial r_{N}}{\partial \phi^z} & \frac{\partial r_{N}}{\partial a} & \frac{\partial r_{N}}{\partial b} & 0 & 0 & . & . & . & \frac{\partial r_{N}}{\partial \rho_{N}} 
\end{bmatrix} \\
&= \begin{bmatrix}
\mathbf{H}_{11} & \mathbf{H}_{12}\\
\mathbf{H}_{12}^{T} & \mathbf{H}_{22}
\end{bmatrix}
\end{aligned}
$$

$$
\mathbf{H}_{11} = \sum^N_{i = 1}\begin{bmatrix}
(\frac{\partial r_{i}}{\partial t^x}\frac{\partial r_{i}}{\partial t^x}) &
(\frac{\partial r_{i}}{\partial t^x}\frac{\partial r_{i}}{\partial t^y}) &
(\frac{\partial r_{i}}{\partial t^x}\frac{\partial r_{i}}{\partial t^z}) &
(\frac{\partial r_{i}}{\partial t^x}\frac{\partial r_{i}}{\partial \phi^x}) &
(\frac{\partial r_{i}}{\partial t^x}\frac{\partial r_{i}}{\partial \phi^y})&
(\frac{\partial r_{i}}{\partial t^x}\frac{\partial r_{i}}{\partial \phi^z})&
(\frac{\partial r_{i}}{\partial t^x}\frac{\partial r_{i}}{\partial a})&
(\frac{\partial r_{i}}{\partial t^x}\frac{\partial r_{i}}{\partial b}) \\
(\frac{\partial r_{i}}{\partial t^y}\frac{\partial r_{i}}{\partial t^x}) &
(\frac{\partial r_{i}}{\partial t^y}\frac{\partial r_{i}}{\partial t^y}) &
(\frac{\partial r_{i}}{\partial t^y}\frac{\partial r_{i}}{\partial t^z}) &
(\frac{\partial r_{i}}{\partial t^y}\frac{\partial r_{i}}{\partial \phi^x}) &
(\frac{\partial r_{i}}{\partial t^y}\frac{\partial r_{i}}{\partial \phi^y})&
(\frac{\partial r_{i}}{\partial t^y}\frac{\partial r_{i}}{\partial \phi^z})&
(\frac{\partial r_{i}}{\partial t^y}\frac{\partial r_{i}}{\partial a})&
(\frac{\partial r_{i}}{\partial t^y}\frac{\partial r_{i}}{\partial b}) \\
(\frac{\partial r_{i}}{\partial t^z}\frac{\partial r_{i}}{\partial t^x}) &
(\frac{\partial r_{i}}{\partial t^z}\frac{\partial r_{i}}{\partial t^y}) &
(\frac{\partial r_{i}}{\partial t^z}\frac{\partial r_{i}}{\partial t^z}) &
(\frac{\partial r_{i}}{\partial t^z}\frac{\partial r_{i}}{\partial \phi^x}) &
(\frac{\partial r_{i}}{\partial t^z}\frac{\partial r_{i}}{\partial \phi^y})&
(\frac{\partial r_{i}}{\partial t^z}\frac{\partial r_{i}}{\partial \phi^z})&
(\frac{\partial r_{i}}{\partial t^z}\frac{\partial r_{i}}{\partial a})&
(\frac{\partial r_{i}}{\partial t^z}\frac{\partial r_{i}}{\partial b}) \\
(\frac{\partial r_{i}}{\partial \phi^x}\frac{\partial r_{i}}{\partial t^x}) &
(\frac{\partial r_{i}}{\partial \phi^x}\frac{\partial r_{i}}{\partial t^y}) &
(\frac{\partial r_{i}}{\partial \phi^x}\frac{\partial r_{i}}{\partial t^z}) &
(\frac{\partial r_{i}}{\partial \phi^x}\frac{\partial r_{i}}{\partial \phi^x}) &
(\frac{\partial r_{i}}{\partial \phi^x}\frac{\partial r_{i}}{\partial \phi^y})&
(\frac{\partial r_{i}}{\partial \phi^x}\frac{\partial r_{i}}{\partial \phi^z})&
(\frac{\partial r_{i}}{\partial \phi^x}\frac{\partial r_{i}}{\partial a}) &
(\frac{\partial r_{i}}{\partial \phi^x}\frac{\partial r_{i}}{\partial b})\\
(\frac{\partial r_{i}}{\partial \phi^y}\frac{\partial r_{i}}{\partial t^x}) &
(\frac{\partial r_{i}}{\partial \phi^y}\frac{\partial r_{i}}{\partial t^y}) &
(\frac{\partial r_{i}}{\partial \phi^y}\frac{\partial r_{i}}{\partial t^z}) &
(\frac{\partial r_{i}}{\partial \phi^y}\frac{\partial r_{i}}{\partial \phi^x}) &
(\frac{\partial r_{i}}{\partial \phi^y}\frac{\partial r_{i}}{\partial \phi^y})&
(\frac{\partial r_{i}}{\partial \phi^y}\frac{\partial r_{i}}{\partial \phi^z})&
(\frac{\partial r_{i}}{\partial \phi^y}\frac{\partial r_{i}}{\partial a}) &
(\frac{\partial r_{i}}{\partial \phi^y}\frac{\partial r_{i}}{\partial b})\\
(\frac{\partial r_{i}}{\partial \phi^z}\frac{\partial r_{i}}{\partial t^x}) &
(\frac{\partial r_{i}}{\partial \phi^z}\frac{\partial r_{i}}{\partial t^y}) &
(\frac{\partial r_{i}}{\partial \phi^z}\frac{\partial r_{i}}{\partial t^z}) &
(\frac{\partial r_{i}}{\partial \phi^z}\frac{\partial r_{i}}{\partial \phi^x}) &
(\frac{\partial r_{i}}{\partial \phi^z}\frac{\partial r_{i}}{\partial \phi^y})&
(\frac{\partial r_{i}}{\partial \phi^z}\frac{\partial r_{i}}{\partial \phi^z})&
(\frac{\partial r_{i}}{\partial \phi^z}\frac{\partial r_{i}}{\partial a}) &
(\frac{\partial r_{i}}{\partial \phi^z}\frac{\partial r_{i}}{\partial b})\\
(\frac{\partial r_{i}}{\partial a}\frac{\partial r_{i}}{\partial t^x}) &
(\frac{\partial r_{i}}{\partial a}\frac{\partial r_{i}}{\partial t^y}) &
(\frac{\partial r_{i}}{\partial a}\frac{\partial r_{i}}{\partial t^z}) &
(\frac{\partial r_{i}}{\partial a}\frac{\partial r_{i}}{\partial \phi^x}) &
(\frac{\partial r_{i}}{\partial a}\frac{\partial r_{i}}{\partial \phi^y})&
(\frac{\partial r_{i}}{\partial a}\frac{\partial r_{i}}{\partial \phi^z})&
(\frac{\partial r_{i}}{\partial a}\frac{\partial r_{i}}{\partial a}) &
(\frac{\partial r_{i}}{\partial a}\frac{\partial r_{i}}{\partial b})\\
(\frac{\partial r_{i}}{\partial b}\frac{\partial r_{i}}{\partial t^x}) &
(\frac{\partial r_{i}}{\partial b}\frac{\partial r_{i}}{\partial t^y}) &
(\frac{\partial r_{i}}{\partial b}\frac{\partial r_{i}}{\partial t^z}) &
(\frac{\partial r_{i}}{\partial b}\frac{\partial r_{i}}{\partial \phi^x}) &
(\frac{\partial r_{i}}{\partial b}\frac{\partial r_{i}}{\partial \phi^y})&
(\frac{\partial r_{i}}{\partial b}\frac{\partial r_{i}}{\partial \phi^z})&
(\frac{\partial r_{i}}{\partial b}\frac{\partial r_{i}}{\partial a}) &
(\frac{\partial r_{i}}{\partial b}\frac{\partial r_{i}}{\partial b})
\end{bmatrix}
$$

$$
\mathbf{H}_{12} = \begin{bmatrix} 
(\frac{\partial r_{1}}{\partial t^x}\frac{\partial r_{1}}{\partial \rho_{1}}) &
(\frac{\partial r_{2}}{\partial t^x}\frac{\partial r_{2}}{\partial \rho_{2}}) &
. &
. &
. &
(\frac{\partial r_{N}}{\partial t^x}\frac{\partial r_{N}}{\partial \rho_{N}}) 
\\
(\frac{\partial r_{1}}{\partial t^y}\frac{\partial r_{1}}{\partial \rho_{1}}) &
(\frac{\partial r_{2}}{\partial t^y}\frac{\partial r_{2}}{\partial \rho_{2}}) &
. &
. &
. &
(\frac{\partial r_{N}}{\partial t^y}\frac{\partial r_{N}}{\partial \rho_{N}}) 
\\
(\frac{\partial r_{1}}{\partial t^z}\frac{\partial r_{1}}{\partial \rho_{1}}) &
(\frac{\partial r_{2}}{\partial t^z}\frac{\partial r_{2}}{\partial \rho_{2}}) &
. &
. &
. &
(\frac{\partial r_{N}}{\partial t^z}\frac{\partial r_{N}}{\partial \rho_{N}}) 
\\
(\frac{\partial r_{1}}{\partial \phi^x}\frac{\partial r_{1}}{\partial \rho_{1}}) &
(\frac{\partial r_{2}}{\partial \phi^x}\frac{\partial r_{2}}{\partial \rho_{2}}) &
. &
. &
. &
(\frac{\partial r_{N}}{\partial \phi^x}\frac{\partial r_{N}}{\partial \rho_{N}}) 
\\
(\frac{\partial r_{1}}{\partial \phi^y}\frac{\partial r_{1}}{\partial \rho_{1}}) &
(\frac{\partial r_{2}}{\partial \phi^y}\frac{\partial r_{2}}{\partial \rho_{2}}) &
. &
. &
. &
(\frac{\partial r_{N}}{\partial \phi^y}\frac{\partial r_{N}}{\partial \rho_{N}}) 
\\
(\frac{\partial r_{1}}{\partial \phi^z}\frac{\partial r_{1}}{\partial \rho_{1}}) &
(\frac{\partial r_{2}}{\partial \phi^z}\frac{\partial r_{2}}{\partial \rho_{2}}) &
. &
. &
. &
(\frac{\partial r_{N}}{\partial \phi^z}\frac{\partial r_{N}}{\partial \rho_{N}}) 
\\
(\frac{\partial r_{1}}{\partial a}\frac{\partial r_{1}}{\partial \rho_{1}}) &
(\frac{\partial r_{2}}{\partial a}\frac{\partial r_{2}}{\partial \rho_{2}}) &
. &
. &
. &
(\frac{\partial r_{N}}{\partial a}\frac{\partial r_{N}}{\partial \rho_{N}}) 
\\
(\frac{\partial r_{1}}{\partial b}\frac{\partial r_{1}}{\partial \rho_{1}}) &
(\frac{\partial r_{2}}{\partial b}\frac{\partial r_{2}}{\partial \rho_{2}}) &
. &
. &
. &
(\frac{\partial r_{N}}{\partial b}\frac{\partial r_{N}}{\partial \rho_{N}}) 
\end{bmatrix}
$$

$$
\mathbf{H}_{22} = \begin{bmatrix} 
(\frac{\partial r_{1}}{\partial \rho_{1}})^2 & 0 & . & . & . & 0 \\
0 & (\frac{\partial r_{2}}{\partial \rho_{2}})^2 & . & . & . & 0 \\
0 & 0 & . & . & . & 0 \\
0 & 0 & . & . & . & 0 \\
0 & 0 & . & . & 0 & (\frac{\partial r_{N}}{\partial \rho_{N}})^2
\end{bmatrix}
$$

$$\begin{aligned}
\mathbf{b} &= \mathbf{J}^T\mathbf{e} \\
&= \begin{bmatrix}
\frac{\partial r_{1}}{\partial t^x} & \frac{\partial r_{2}}{\partial t^x} & . & . & . & \frac{\partial r_{N}}{\partial t^x}\\ 
\frac{\partial r_{1}}{\partial t^y} & \frac{\partial r_{2}}{\partial t^y} & . & . & . & \frac{\partial r_{N}}{\partial t^y} 
\\ \frac{\partial r_{1}}{\partial t^z} & \frac{\partial r_{2}}{\partial t^z} & . & . & . & \frac{\partial r_{N}}{\partial t^z} 
\\ \frac{\partial r_{1}}{\partial \phi^x} & \frac{\partial r_{2}}{\partial \phi^x} & . & . & . & \frac{\partial r_{N}}{\partial \phi^x} 
\\ \frac{\partial r_{1}}{\partial \phi^y} & \frac{\partial r_{2}}{\partial \phi^y} & . & . & . & \frac{\partial r_{N}}{\partial \phi^y} 
\\ \frac{\partial r_{1}}{\partial \phi^z} & \frac{\partial r_{2}}{\partial \phi^z} & . & . & . & \frac{\partial r_{N}}{\partial \phi^z} 
\\ \frac{\partial r_{1}}{\partial a} & \frac{\partial r_{2}}{\partial a} & . & . & . & \frac{\partial r_{N}}{\partial a} 
\\ \frac{\partial r_{1}}{\partial b} & \frac{\partial r_{2}}{\partial b} & . & . & . & \frac{\partial r_{N}}{\partial b}
\\ \frac{\partial r_{1}}{\partial \rho_{1}}  & 0 & . &. & . & 0
\\ 0 & \frac{\partial r_{2}}{\partial \rho_{2}}  & . & . & . & 0\\ 0 & . & . & .& . & 0 \\ 0 & . &. & .& . & 0\\ 0 & . &. & . & .& 0\\ 0 & . &. & . & 0 & \frac{\partial r_{N}}{\partial \rho_{N}} 
\end{bmatrix} \begin{bmatrix}r_{1} \\ r_{2} \\ . \\.\\. \\ r_{N} \end{bmatrix}
\\
&= \begin{bmatrix}
\sum^N_{i=1}(\frac{\partial r_{i}}{\partial t^x} r_{i}) \\ 
\sum^N_{i=1}(\frac{\partial r_{i}}{\partial t^y} r_{i}) \\ 
\sum^N_{i=1}(\frac{\partial r_{i}}{\partial t^z} r_{i}) \\
\sum^N_{i=1}(\frac{\partial r_{i}}{\partial \phi^x} r_{i})\\
\sum^N_{i=1}(\frac{\partial r_{i}}{\partial \phi^y} r_{i}) \\ 
\sum^N_{i=1}(\frac{\partial r_{i}}{\partial \phi^z} r_{i}) \\ 
\sum^N_{i=1}(\frac{\partial r_{i}}{\partial a} r_{i}) \\ 
\sum^N_{i=1}(\frac{\partial r_{i}}{\partial b} r_{i}) \\
\frac{\partial r_{1}}{\partial \rho_{1}} r_{1} \\
\frac{\partial r_{2}}{\partial \rho_{2}} r_{2} \\
. \\
. \\
. \\
\frac{\partial r_{N}}{\partial \rho_{N}} r_{N} \\
\end{bmatrix}
\end{aligned}
$$

#### Schur complement

Obviously, we can exploit the spartsiy in $$\mathbf{H}_{22}$$ to solve $$\delta \mathbf{x}_{1}$$ with

$$(\mathbf{H}_{11}-\mathbf{H}_{12} \mathbf{H}_{22}^{-1} \mathbf{H}_{12}^{T})\delta \mathbf{x}_{1} = -(\mathbf{b}_{1}-\mathbf{H}_{12} \mathbf{H}_{22}^{-1} \mathbf{b}_{2})$$

and solve $$\delta \mathbf{x}_{2}$$ with

$$\delta \mathbf{x}_{2} = -\mathbf{H}_{22}^{-1}(\mathbf{b}_{2} + \mathbf{H}^T_{12}\delta\mathbf{x}_{1})$$

Remember, 

$$
\mathbf{H}_{22}^{-1} = \begin{bmatrix} 
(\frac{\partial r_{1}}{\partial \rho_{1}})^{-2} & 0 & . & . & . & 0 \\
0 & (\frac{\partial r_{2}}{\partial \rho_{2}})^{-2} & . & . & . & 0 \\
0 & 0 & . & . & . & 0 \\
0 & 0 & . & . & . & 0 \\
0 & 0 & . & . & 0 & (\frac{\partial r_{N}}{\partial \rho_{N}})^{-2}
\end{bmatrix}
$$

#### Codes

| Variable used in `CoarseInitializer` | Variable used here     | Meaning               |
| ------------------------------------ | ---------- | -------------- |
| `JbBuffer_new[i][0]`                 | $$\frac{\partial r_{i}}{\partial t^x}\frac{\partial r_{i}}{\partial \rho_{i}}$$    | 第`i`个点提供的计算$$\mathbf{H}_{12}$$所需的中间量                      |
| `JbBuffer_new[i][1]`                 | $$\frac{\partial r_{i}}{\partial t^y}\frac{\partial r_{i}}{\partial \rho_{i}}$$    | 第`i`个点提供的计算$$\mathbf{H}_{12}$$所需的中间量                      |
| `JbBuffer_new[i][2]`                 | $$\frac{\partial r_{i}}{\partial t^z}\frac{\partial r_{i}}{\partial \rho_{i}}$$    | 第`i`个点提供的计算$$\mathbf{H}_{12}$$所需的中间量                      |
| `JbBuffer_new[i][3]`                 | $$\frac{\partial r_{i}}{\partial \phi^x}\frac{\partial r_{i}}{\partial \rho_{i}}$$ | 第`i`个点提供的计算$$\mathbf{H}_{12}$$所需的中间量                      |
| `JbBuffer_new[i][4]`                 | $$\frac{\partial r_{i}}{\partial \phi^y}\frac{\partial r_{i}}{\partial \rho_{i}}$$ | 第`i`个点提供的计算$$\mathbf{H}_{12}$$所需的中间量                      |
| `JbBuffer_new[i][5]`                 | $$\frac{\partial r_{i}}{\partial \phi^z}\frac{\partial r_{i}}{\partial \rho_{i}}$$ | 第`i`个点提供的计算$$\mathbf{H}_{12}$$所需的中间量                      |
| `JbBuffer_new[i][6]`                 | $$\frac{\partial r_{i}}{\partial a}\frac{\partial r_{i}}{\partial \rho_{i}}$$      | 第`i`个点提供的计算$$\mathbf{H}_{12}$$所需的中间量                      |
| `JbBuffer_new[i][7]`                 | $$\frac{\partial r_{i}}{\partial b}\frac{\partial r_{i}}{\partial \rho_{i}}$$      | 第`i`个点提供的计算$$\mathbf{H}_{12}$$所需的中间量                      |
| `JbBuffer_new[i][8]`                 | $$r_{i} \frac{\partial r_{i}}{\partial \rho_{i}}$$         | 第`i`个点提供的计算$$\mathbf{b}$$所需的中间量   |
| `JbBuffer_new[i][9]`                 | $$(\frac{\partial r_{i}}{\partial \rho_{i}})^2$$           | 第`i`个点提供的计算$$\mathbf{H}_{22}$$以及$$\mathbf{H}_{22}^{-1}$$所需的中间量                    |
| `Accumulator9 acc9`                  |          | 计算$$\mathbf{H}_{11}$$和$$\mathbf{b}_{1}$$     |
| `Accumulator9 acc9SC`                |          | 计算$$\mathbf{H}_{12} \mathbf{H}_{22}^{-1} \mathbf{H}_{12}^{T}$$和$$\mathbf{H}_{12} \mathbf{H}_{22}^{-1} \mathbf{b}_{2}$$ |
| `Accumulator11 E`                    |          | 计算总的光度误差      |

### Sliding Window Optimization

#### Preparation

For convenience, we consider only __one__ residual (which is DEFINITELY __IMPOSSIBLE__) wrt. two frames in the following sections. 

As introduced [here](http://tongling916.github.io/2020/04/02/Jacobian-Matrices/), we have 

$$
\mathbf{J}=\begin{bmatrix}
\frac{\partial r}{\partial \mathbf{C}} & \frac{\partial r}{\partial \boldsymbol{\xi}_{iw}} & \frac{\partial r}{\partial \mathbf{a}_{i}} & \frac{\partial r}{\partial \boldsymbol{\xi}_{jw}} & \frac{\partial r}{\partial \mathbf{a}_{j}} & \frac{\partial r}{\partial \rho}
\end{bmatrix}
$$

$$\mathbf{e} = \begin{bmatrix}r\end{bmatrix}$$

$$\mathbf{x} = \begin{bmatrix} \mathbf{C} \\ \boldsymbol{\xi}_{iw} \\ \mathbf{a}_{i} \\ \boldsymbol{\xi}_{jw} \\ \mathbf{a}_{j} \\  \rho \end{bmatrix} = \begin{bmatrix}\mathbf{x}_1 \\ \mathbf{x}_{2} \end{bmatrix}$$, where 
$$\mathbf{x}_1 = \begin{bmatrix}\mathbf{C} \\ \boldsymbol{\xi}_{iw} \\ \mathbf{a}_{i} \\ \boldsymbol{\xi}_{jw} \\ \mathbf{a}_{j}\end{bmatrix}$$, $$\mathbf{x}_2 = \begin{bmatrix}\rho \end{bmatrix}$$

| Variable | Meaning | 
| - | - |
| $$r$$ | single residual|
| $$\mathbf{C}=\begin{bmatrix}f_{x} \\ f_{y} \\ c_{x} \\ c_{y} \end{bmatrix}$$ | intrinsic paramters (`4 x 1`)|
| $$\boldsymbol{\xi}_{iw}$$ | pose $$\mathbf{T}_{iw}$$ of frame i (`6 x 1`)|
| $$\mathbf{a}_{i}=\begin{bmatrix}a_{i} \\ b_{i}\end{bmatrix}$$ | photometric paramters of frame i (`2 x 1`)|
| $$\boldsymbol{\xi}_{jw}$$ | pose $$\mathbf{T}_{jw}$$ of frame j (`6 x 1`)|
| $$\mathbf{a}_{j}=\begin{bmatrix}a_{j} \\ b_{j}\end{bmatrix}$$ | photometric paramters of frame j (`2 x 1`)|
| $$\rho$$ | inverse depth (`1 x 1`)|

#### Hessian

$$
\begin{aligned}
\mathbf{H} &= \mathbf{J}^T\mathbf{J} \\
&= \begin{bmatrix}
(\frac{\partial r}{\partial \mathbf{C}})^{T} 
\\ (\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}})^{T} 
\\ (\frac{\partial r}{\partial \mathbf{a}_{i}})^{T} 
\\ (\frac{\partial r}{\partial \boldsymbol{\xi}_{jw}})^{T} 
\\ (\frac{\partial r}{\partial \mathbf{a}_{j}})^{T} 
\\ (\frac{\partial r}{\partial \rho})^{T}
\end{bmatrix}
\begin{bmatrix}
\frac{\partial r}{\partial \mathbf{C}} 
& \frac{\partial r}{\partial \boldsymbol{\xi}_{iw}} 
& \frac{\partial r}{\partial \mathbf{a}_{i}}
& \frac{\partial r}{\partial \boldsymbol{\xi}_{jw}} 
& \frac{\partial r}{\partial \mathbf{a}_{j}} 
& \frac{\partial r}{\partial \rho}
\end{bmatrix} \\
&= \begin{bmatrix}
(\frac{\partial r}{\partial \mathbf{C}})^{T}\frac{\partial r}{\partial \mathbf{C}} 
&(\frac{\partial r}{\partial \mathbf{C}})^{T}\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}} 
&(\frac{\partial r}{\partial \mathbf{C}})^{T}\frac{\partial r}{\partial \mathbf{a}_{i}} 
&(\frac{\partial r}{\partial \mathbf{C}})^{T} \frac{\partial r}{\partial \boldsymbol{\xi}_{jw}} 
&(\frac{\partial r}{\partial \mathbf{C}})^{T} \frac{\partial r}{\partial \mathbf{a}_{j}} 
&(\frac{\partial r}{\partial \mathbf{C}})^{T} \frac{\partial r}{\partial \rho}\\ 
(\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}})^{T} \frac{\partial r}{\partial \mathbf{C}}
& (\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}})^{T}\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}} 
& (\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}})^{T}\frac{\partial r}{\partial \mathbf{a}_{i}} 
& (\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}})^{T} \frac{\partial r}{\partial \boldsymbol{\xi}_{jw}} 
& (\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}})^{T} \frac{\partial r}{\partial \mathbf{a}_{j}} 
& (\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}})^{T} \frac{\partial r}{\partial \rho}\\
(\frac{\partial r}{\partial \mathbf{a}_{i}})^{T}\frac{\partial r}{\partial \mathbf{C}} 
& (\frac{\partial r}{\partial \mathbf{a}_{i}})^{T}\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}} 
& (\frac{\partial r}{\partial \mathbf{a}_{i}})^{T} \frac{\partial r}{\partial \mathbf{a}_{i}} 
& (\frac{\partial r}{\partial \mathbf{a}_{i}})^{T} \frac{\partial r}{\partial \boldsymbol{\xi}_{jw}} 
& (\frac{\partial r}{\partial \mathbf{a}_{i}})^{T} \frac{\partial r}{\partial \mathbf{a}_{j}} 
&(\frac{\partial r}{\partial \mathbf{a}_{i}})^{T} \frac{\partial r}{\partial \rho}\\ 
(\frac{\partial r}{\partial \boldsymbol{\xi}_{jw}})^{T}\frac{\partial r}{\partial \mathbf{C}} 
& (\frac{\partial r}{\partial \boldsymbol{\xi}_{jw}})^{T}\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}} 
& (\frac{\partial r}{\partial \boldsymbol{\xi}_{jw}})^{T} \frac{\partial r}{\partial \mathbf{a}_{i}} 
& (\frac{\partial r}{\partial \boldsymbol{\xi}_{jw}})^{T} \frac{\partial r}{\partial \boldsymbol{\xi}_{jw}} 
& (\frac{\partial r}{\partial \boldsymbol{\xi}_{jw}})^{T} \frac{\partial r}{\partial \mathbf{a}_{j}} 
& (\frac{\partial r}{\partial \boldsymbol{\xi}_{jw}})^{T} \frac{\partial r}{\partial \rho}\\ 
(\frac{\partial r}{\partial \mathbf{a}_{j}})^{T} \frac{\partial r}{\partial \mathbf{C}}
& (\frac{\partial r}{\partial \mathbf{a}_{j}})^{T}\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}} 
&  (\frac{\partial r}{\partial \mathbf{a}_{j}})^{T} \frac{\partial r}{\partial \mathbf{a}_{i}} 
&  (\frac{\partial r}{\partial \mathbf{a}_{j}})^{T} \frac{\partial r}{\partial \boldsymbol{\xi}_{jw}} 
&  (\frac{\partial r}{\partial \mathbf{a}_{j}})^{T} \frac{\partial r}{\partial \mathbf{a}_{j}} 
&  (\frac{\partial r}{\partial \mathbf{a}_{j}})^{T} \frac{\partial r}{\partial \rho}\\ 
(\frac{\partial r}{\partial \rho})^{T} \frac{\partial r}{\partial \mathbf{C}}
& (\frac{\partial r}{\partial \rho})^{T}\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}} 
& (\frac{\partial r}{\partial \rho})^{T} \frac{\partial r}{\partial \mathbf{a}_{i}} 
& (\frac{\partial r}{\partial \rho})^{T} \frac{\partial r}{\partial \boldsymbol{\xi}_{jw}} 
& (\frac{\partial r}{\partial \rho})^{T} \frac{\partial r}{\partial \mathbf{a}_{j}} 
& (\frac{\partial r}{\partial \rho})^{T} \frac{\partial r}{\partial \rho}
\end{bmatrix} \\
&= \begin{bmatrix} 
\mathbf{H}_{11} & \mathbf{H}_{12} \\ 
\mathbf{H}_{12}^T & \mathbf{H}_{22}
\end{bmatrix}
\end{aligned}
$$

$$
\mathbf{H}_{11} = \begin{bmatrix}
(\frac{\partial r}{\partial \mathbf{C}})^{T}\frac{\partial r}{\partial \mathbf{C}} 
&(\frac{\partial r}{\partial \mathbf{C}})^{T}\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}} 
&(\frac{\partial r}{\partial \mathbf{C}})^{T}\frac{\partial r}{\partial \mathbf{a}_{i}} 
&(\frac{\partial r}{\partial \mathbf{C}})^{T} \frac{\partial r}{\partial \boldsymbol{\xi}_{jw}} 
&(\frac{\partial r}{\partial \mathbf{C}})^{T} \frac{\partial r}{\partial \mathbf{a}_{j}} \\ 
(\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}})^{T} \frac{\partial r}{\partial \mathbf{C}}
& (\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}})^{T}\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}} 
& (\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}})^{T}\frac{\partial r}{\partial \mathbf{a}_{i}} 
& (\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}})^{T} \frac{\partial r}{\partial \boldsymbol{\xi}_{jw}} 
& (\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}})^{T} \frac{\partial r}{\partial \mathbf{a}_{j}} \\
(\frac{\partial r}{\partial \mathbf{a}_{i}})^{T}\frac{\partial r}{\partial \mathbf{C}} 
& (\frac{\partial r}{\partial \mathbf{a}_{i}})^{T}\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}} 
& (\frac{\partial r}{\partial \mathbf{a}_{i}})^{T} \frac{\partial r}{\partial \mathbf{a}_{i}} 
& (\frac{\partial r}{\partial \mathbf{a}_{i}})^{T} \frac{\partial r}{\partial \boldsymbol{\xi}_{jw}} 
& (\frac{\partial r}{\partial \mathbf{a}_{i}})^{T} \frac{\partial r}{\partial \mathbf{a}_{j}} \\ 
(\frac{\partial r}{\partial \boldsymbol{\xi}_{jw}})^{T}\frac{\partial r}{\partial \mathbf{C}} 
& (\frac{\partial r}{\partial \boldsymbol{\xi}_{jw}})^{T}\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}} 
& (\frac{\partial r}{\partial \boldsymbol{\xi}_{jw}})^{T} \frac{\partial r}{\partial \mathbf{a}_{i}} 
& (\frac{\partial r}{\partial \boldsymbol{\xi}_{jw}})^{T} \frac{\partial r}{\partial \boldsymbol{\xi}_{jw}} 
& (\frac{\partial r}{\partial \boldsymbol{\xi}_{jw}})^{T} \frac{\partial r}{\partial \mathbf{a}_{j}} \\ 
(\frac{\partial r}{\partial \mathbf{a}_{j}})^{T} \frac{\partial r}{\partial \mathbf{C}}
& (\frac{\partial r}{\partial \mathbf{a}_{j}})^{T}\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}} 
&  (\frac{\partial r}{\partial \mathbf{a}_{j}})^{T} \frac{\partial r}{\partial \mathbf{a}_{i}} 
&  (\frac{\partial r}{\partial \mathbf{a}_{j}})^{T} \frac{\partial r}{\partial \boldsymbol{\xi}_{jw}} 
&  (\frac{\partial r}{\partial \mathbf{a}_{j}})^{T} \frac{\partial r}{\partial \mathbf{a}_{j}} 
\end{bmatrix} 
$$

$$
\mathbf{H}_{12} = \begin{bmatrix}
(\frac{\partial r}{\partial \mathbf{C}})^{T} \frac{\partial r}{\partial \rho}
\\ (\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}})^{T} \frac{\partial r}{\partial \rho}
\\ (\frac{\partial r}{\partial \mathbf{a}_{i}})^{T} \frac{\partial r}{\partial \rho}
\\ (\frac{\partial r}{\partial \boldsymbol{\xi}_{jw}})^{T} \frac{\partial r}{\partial \rho}
\\ (\frac{\partial r}{\partial \mathbf{a}_{j}} )^{T} \frac{\partial r}{\partial \rho}
\end{bmatrix}
$$

$$
\mathbf{H}_{22} = 
\begin{bmatrix}
    (\frac{\partial r}{\partial \rho})^{T} \frac{\partial r}{\partial \rho}
\end{bmatrix} =
\begin{bmatrix}
    (\frac{\partial r}{\partial \rho})^{2}
\end{bmatrix}
$$

#### b

$$
\begin{aligned}
\mathbf{b} &= \mathbf{J}^T\mathbf{e} \\
&= \begin{bmatrix}
(\frac{\partial r}{\partial \mathbf{C}})^{T} 
\\ (\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}})^{T} 
\\ (\frac{\partial r}{\partial \mathbf{a}_{i}})^{T} 
\\ (\frac{\partial r}{\partial \boldsymbol{\xi}_{jw}})^{T} 
\\ (\frac{\partial r}{\partial \mathbf{a}_{j}})^{T} 
\\ (\frac{\partial r}{\partial \rho})^{T}
\end{bmatrix} r
\end{aligned}
$$

$$
\mathbf{b}_{1} = \begin{bmatrix}
r(\frac{\partial r}{\partial \mathbf{C}})^{T} 
\\ r(\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}})^{T} 
\\ r(\frac{\partial r}{\partial \mathbf{a}_{i}})^{T} 
\\ r(\frac{\partial r}{\partial \boldsymbol{\xi}_{jw}})^{T} 
\\ r(\frac{\partial r}{\partial \mathbf{a}_{j}})^{T}
\end{bmatrix} 
$$

$$
\mathbf{b}_{2} = \begin{bmatrix}
r (\frac{\partial r}{\partial \rho})^{T} 
\end{bmatrix}
$$

#### Implementation

In the real implementation, the author didn't directly compute $$\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}}$$, $$\frac{\partial r}{\partial \boldsymbol{\xi}_{jw}}$$, $$\frac{\partial r}{\partial \mathbf{a}_{i}}$$ and $$\frac{\partial r}{\partial \mathbf{a}_{j}}$$. Instead, he used the chain rules 

$$
\begin{aligned}
\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}} &= \frac{\partial r}{\partial \boldsymbol{\xi}_{ji}} \frac{\partial \boldsymbol{\xi}_{ji}}{\partial \boldsymbol{\xi}_{iw}}
\end{aligned}
$$

$$
\begin{aligned}
\frac{\partial r}{\partial \boldsymbol{\xi}_{jw}} &= \frac{\partial r}{\partial \boldsymbol{\xi}_{ji}} \frac{\partial \boldsymbol{\xi}_{ji}}{\partial \boldsymbol{\xi}_{jw}}
\end{aligned}
$$

$$
\begin{aligned}
\frac{\partial r}{\partial \mathbf{a}_{i}} &= \frac{\partial r}{\partial \begin{bmatrix} -e^{a_{ji}} \\ -b_{ji}\end{bmatrix}} \frac{\partial \begin{bmatrix} -e^{a_{ji}} \\ -b_{ji}\end{bmatrix}}{\partial \mathbf{a}_{i}}
\end{aligned}
$$

$$
\begin{aligned}
\frac{\partial r}{\partial \mathbf{a}_{j}} &= \frac{\partial r}{\partial \begin{bmatrix} -e^{a_{ji}} \\ -b_{ji}\end{bmatrix}} \frac{\partial \begin{bmatrix} -e^{a_{ji}} \\ -b_{ji}\end{bmatrix}}{\partial \mathbf{a}_{j}}
\end{aligned}
$$

And their tranposes are 

$$
\begin{aligned}
(\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}})^{T} &= (\frac{\partial \boldsymbol{\xi}_{ji}}{\partial \boldsymbol{\xi}_{iw}})^{T}(\frac{\partial r}{\partial \boldsymbol{\xi}_{ji}})^{T}
\end{aligned}
$$

$$
\begin{aligned}
(\frac{\partial r}{\partial \boldsymbol{\xi}_{jw}})^{T} &= (\frac{\partial \boldsymbol{\xi}_{ji}}{\partial \boldsymbol{\xi}_{jw}})^{T}(\frac{\partial r}{\partial \boldsymbol{\xi}_{ji}})^{T} 
\end{aligned}
$$

$$
\begin{aligned}
(\frac{\partial r}{\partial \mathbf{a}_{i}})^{T} &= (\frac{\partial \begin{bmatrix} -e^{a_{ji}} \\ -b_{ji}\end{bmatrix}}{\partial \mathbf{a}_{i}})^{T}(\frac{\partial r}{\partial \begin{bmatrix} -e^{a_{ji}} \\ -b_{ji}\end{bmatrix}})^{T} 
\end{aligned}
$$

$$
\begin{aligned}
(\frac{\partial r}{\partial \mathbf{a}_{j}})^{T} &= (\frac{\partial \begin{bmatrix} -e^{a_{ji}} \\ -b_{ji}\end{bmatrix}}{\partial \mathbf{a}_{j}})^{T}(\frac{\partial r}{\partial \begin{bmatrix} -e^{a_{ji}} \\ -b_{ji}\end{bmatrix}})^{T}
\end{aligned}
$$


Details about these Jacobians can be found in this [post](http://tongling916.github.io/2020/04/17/DSO-Jacobians/).

#### Schur Complement

$$(\mathbf{H}_{11}-\mathbf{H}_{12} \mathbf{H}_{22}^{-1} \mathbf{H}_{12}^{T})\delta \mathbf{x}_{1} = -(\mathbf{b}_{1}-\mathbf{H}_{12} \mathbf{H}_{22}^{-1} \mathbf{b}_{2})$$

$$
\begin{aligned}
    \mathbf{H}_{\text{sc}} &= \mathbf{H}_{12} \mathbf{H}_{22}^{-1} \mathbf{H}_{12}^{T} \\
    &= \begin{bmatrix}
        (\frac{\partial r}{\partial \mathbf{C}})^{T} \frac{\partial r}      {\partial \rho}
        \\ (\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}})^{T} \frac     {\partial r}{\partial \rho}
        \\ (\frac{\partial r}{\partial \mathbf{a}_{i}})^{T} \frac{\partial r}        {\partial \rho}
        \\ (\frac{\partial r}{\partial \boldsymbol{\xi}_{jw}})^{T} \frac     {\partial r}{\partial \rho}
        \\ (\frac{\partial r}{\partial \mathbf{a}_{j}} )^{T} \frac{\partial r}       {\partial \rho}
        \end{bmatrix}
        \begin{bmatrix}
            (\frac{\partial r}{\partial \rho})^{-2}
        \end{bmatrix}
        \begin{bmatrix}
            (\frac{\partial r}{\partial \rho})^{T} \frac{\partial r}{\partial \mathbf{C}}
            & (\frac{\partial r}{\partial \rho})^{T}\frac{\partial r}   {\partial      \boldsymbol{\xi}_{iw}} 
            & (\frac{\partial r}{\partial \rho})^{T} \frac{\partial r}  {\partial         \mathbf{a}_{i}} 
            & (\frac{\partial r}{\partial \rho})^{T} \frac{\partial r}  {\partial         \boldsymbol{\xi}_{jw}} 
            & (\frac{\partial r}{\partial \rho})^{T} \frac{\partial r}  {\partial         \mathbf{a}_{j}}      
        \end{bmatrix} \\
    &= \begin{bmatrix}
        (\frac{\partial r}{\partial \mathbf{C}})^{T} \frac{\partial r}{\partial \mathbf{C}} &
        (\frac{\partial r}{\partial \mathbf{C}})^{T} \frac{\partial r}{\partial  \boldsymbol{\xi}_{iw}} & 
        (\frac{\partial r}{\partial \mathbf{C}})^{T} \frac{\partial r}{\partial \mathbf{a}_{i}} &
        (\frac{\partial r}{\partial \mathbf{C}})^{T} \frac{\partial r}{\partial \boldsymbol{\xi}_{jw}} &
        (\frac{\partial r}{\partial \mathbf{C}})^{T} \frac{\partial r}{\partial  \mathbf{a}_{j}}
        \\ 
        (\frac{\partial r}{\partial  \boldsymbol{\xi}_{iw}})^{T} \frac{\partial r}{\partial \mathbf{C}} &
        (\frac{\partial r}{\partial  \boldsymbol{\xi}_{iw}})^{T} \frac{\partial r}{\partial  \boldsymbol{\xi}_{iw}} & 
        (\frac{\partial r}{\partial  \boldsymbol{\xi}_{iw}})^{T} \frac{\partial r}{\partial \mathbf{a}_{i}} &
        (\frac{\partial r}{\partial  \boldsymbol{\xi}_{iw}})^{T} \frac{\partial r}{\partial \boldsymbol{\xi}_{jw}} &
        (\frac{\partial r}{\partial  \boldsymbol{\xi}_{iw}})^{T} \frac{\partial r}{\partial  \mathbf{a}_{j}}
        \\
        (\frac{\partial r}{\partial \mathbf{a}_{i}})^{T} \frac{\partial r}{\partial \mathbf{C}} &
        (\frac{\partial r}{\partial \mathbf{a}_{i}})^{T} \frac{\partial r}{\partial  \boldsymbol{\xi}_{iw}} & 
        (\frac{\partial r}{\partial \mathbf{a}_{i}})^{T} \frac{\partial r}{\partial \mathbf{a}_{i}} &
        (\frac{\partial r}{\partial \mathbf{a}_{i}})^{T} \frac{\partial r}{\partial \boldsymbol{\xi}_{jw}} &
        (\frac{\partial r}{\partial \mathbf{a}_{i}})^{T} \frac{\partial r}{\partial  \mathbf{a}_{j}}
        \\
        (\frac{\partial r}{\partial \boldsymbol{\xi}_{jw}})^{T} \frac{\partial r}{\partial \mathbf{C}} &
        (\frac{\partial r}{\partial \boldsymbol{\xi}_{jw}})^{T} \frac{\partial r}{\partial  \boldsymbol{\xi}_{iw}} & 
        (\frac{\partial r}{\partial \boldsymbol{\xi}_{jw}})^{T} \frac{\partial r}{\partial \mathbf{a}_{i}} &
        (\frac{\partial r}{\partial \boldsymbol{\xi}_{jw}})^{T} \frac{\partial r}{\partial \boldsymbol{\xi}_{jw}} &
        (\frac{\partial r}{\partial \boldsymbol{\xi}_{jw}})^{T} \frac{\partial r}{\partial  \mathbf{a}_{j}}
        \\
        (\frac{\partial r}{\partial \mathbf{a}_{j}})^{T} \frac{\partial r}{\partial \mathbf{C}} &
        (\frac{\partial r}{\partial \mathbf{a}_{j}})^{T} \frac{\partial r}{\partial  \boldsymbol{\xi}_{iw}} & 
        (\frac{\partial r}{\partial \mathbf{a}_{j}})^{T} \frac{\partial r}{\partial \mathbf{a}_{i}} &
        (\frac{\partial r}{\partial \mathbf{a}_{j}})^{T} \frac{\partial r}{\partial \boldsymbol{\xi}_{jw}} &
        (\frac{\partial r}{\partial \mathbf{a}_{j}})^{T} \frac{\partial r}{\partial  \mathbf{a}_{j}}
        \end{bmatrix}
\end{aligned}
$$

$$
\begin{aligned}
\mathbf{b}_{\text{sc}} &= \mathbf{H}_{12} \mathbf{H}_{22}^{-1} \mathbf{b}_{2} \\ 
&= \begin{bmatrix}
        (\frac{\partial r}{\partial \mathbf{C}})^{T} \frac{\partial r}      {\partial \rho}
        \\ (\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}})^{T} \frac     {\partial r}{\partial \rho}
        \\ (\frac{\partial r}{\partial \mathbf{a}_{i}})^{T} \frac{\partial r}        {\partial \rho}
        \\ (\frac{\partial r}{\partial \boldsymbol{\xi}_{jw}})^{T} \frac     {\partial r}{\partial \rho}
        \\ (\frac{\partial r}{\partial \mathbf{a}_{j}} )^{T} \frac{\partial r}       {\partial \rho}
    \end{bmatrix}
    \begin{bmatrix}
        (\frac{\partial r}{\partial \rho})^{-2}
    \end{bmatrix}
    \begin{bmatrix}
        r (\frac{\partial r}{\partial \rho})^{T} 
    \end{bmatrix} \\ 
    &= \begin{bmatrix}
        (\frac{\partial r}{\partial \mathbf{C}})^{T} r
        \\ (\frac{\partial r}{\partial \boldsymbol{\xi}_{iw}})^{T} r
        \\ (\frac{\partial r}{\partial \mathbf{a}_{i}})^{T} r
        \\ (\frac{\partial r}{\partial \boldsymbol{\xi}_{jw}})^{T} r
        \\ (\frac{\partial r}{\partial \mathbf{a}_{j}} )^{T} r
    \end{bmatrix}
\end{aligned}
$$

