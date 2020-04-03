---
layout:     post
title:      "Schur Complement"
date:       2020-4-2
author:     Tong
catalog: true
tags:
    - SLAM
---

### Foreknowledge

- For a diagonal matrix

$$A=\left(\begin{array}{cccc}
a_{11} & 0 & \cdots & 0 \\
0 & a_{22} & \cdots & 0 \\
\vdots & \vdots & & \vdots \\
0 & 0 & \cdots & a_{n n}
\end{array}\right)$$,

its inverse is 

$$A^{-1}=\left(\begin{array}{cccc}
1 / a_{11} & 0 & \cdots & 0 \\
0 & 1 / a_{22} & \cdots & 0 \\
\vdots & \vdots & \vdots \\
0 & 0 & \cdots & 1 / a_{n n}
\end{array}\right)$$

### Schur Complement [^Barfoot17]

#### Problem Formulation

In Newton's or Gauss-Newton method, we often need to solve the following equation

$$\underbrace{\left[\begin{array}{ll}
\mathbf{H}_{11} & \mathbf{H}_{12} \\
\mathbf{H}_{12}^{T} & \mathbf{H}_{22}
\end{array}\right]}_{\mathbf{H}} \underbrace{\left[\begin{array}{l}
\delta \mathbf{x}_{1} \\
\delta \mathbf{x}_{2}
\end{array}\right]}_{\delta \mathbf{x}}=\underbrace{-\left[\begin{array}{l}
\mathbf{b}_{1} \\
\mathbf{b}_{2}
\end{array}\right]}_{-\mathbf{b}}$$

|Variable|Meaning|
|---|---|
|$$\mathbf{H}$$|Hessian matrix|
|$$\delta \mathbf{x}$$|Increment of variables to optimize|
|$$\mathbf{x}_{1}$$|`K` variables to optimize|
|$$\mathbf{x}_{2}$$|`M` variables to optimize|

#### Technique

If $$\mathbf{H}_{22}$$ has some sparse structure (diagonal matrix) like below, 

![](https://cdn.mathpix.com/snip/images/PsleFG3qjwvIi4xNH6qIO7jG1vqZIb3Flt6z1gPaADY.original.fullsize.png)

we can exploit this sparsity by premultiplying both sides by
$$\left[\begin{array}{cc}
\mathbf{I} & -\mathbf{H}_{12} \mathbf{H}_{22}^{-1} \\
\mathbf{0} & \mathbf{I}
\end{array}\right]$$, then we have 

$$\left[\begin{array}{cc}
\mathbf{H}_{11}-\mathbf{H}_{12} \mathbf{H}_{22}^{-1} \mathbf{H}_{12}^{T} & \mathbf{0} \\
\mathbf{H}_{12}^{T} & \mathbf{H}_{22}
\end{array}\right]\left[\begin{array}{c}
\delta \mathbf{x}_{1} \\
\delta \mathbf{x}_{2}
\end{array}\right]=-\left[\begin{array}{c}
\mathbf{b}_{1}-\mathbf{H}_{12} \mathbf{H}_{22}^{-1} \mathbf{b}_{2} \\
\mathbf{b}_{2}
\end{array}\right]$$

As a result, we can solve $$\delta \mathbf{x}_{1}$$ with 
$$(\mathbf{H}_{11}-\mathbf{H}_{12} \mathbf{H}_{22}^{-1} \mathbf{H}_{12}^{T})\delta \mathbf{x}_{1} = -(\mathbf{b}_{1}-\mathbf{H}_{12} \mathbf{H}_{22}^{-1} \mathbf{b}_{2})
$$ first, then substitute it into 
the second equation 
$$\mathbf{H}_{12}^{T} \delta \mathbf{x}_{1} + \mathbf{H}_{22} \delta \mathbf{x}_{2} = -\mathbf{b}_{2}
$$ and solve $$\delta \mathbf{x}_{2} = -\mathbf{H}_{22}^{-1}(\mathbf{b}_{2} + \mathbf{H}^T_{12}\delta\mathbf{x}_{1})$$.

$$\delta \mathbf{x}_{2} = -\mathbf{H}_{22}^{-1}(\mathbf{b}_{2} + \mathbf{H}^T_{12}\delta\mathbf{x}_{1})$$

This procedure brings the complexity of each solve down from $$O((K + M)^3)$$ without sparsity to $$O((K^3 + K^2M))$$ with sparsity, which is most beneficial when
$$K \ll  M$$.

To speed up solving $$\delta \mathbf{x}_{1}$$, we can use some decomposition techniques introduced soon.

#### Gauss-Newton

As explained in this [Themenrunde](https://github.com/TongLing916/tongling916.github.io/blob/master/documents/Themenrunde_LDSO.pdf), the variables used in Gauss-Newton are computed as follows (Suppose weight matrix is an identity matrix)

$$\mathbf{H} = \mathbf{J}^T\mathbf{J}$$

$$\mathbf{b} = \mathbf{J}^T\mathbf{e}$$

|Variable | Meaning|
|---|---|
|$$\mathbf{J}$$|Jacobian matrix|
|$$\mathbf{e}$$|residual vector|

### Decomposition 

- Cholesky Decomposition [^Barfoot17]

- $$LDL^T$$ Decomposition [^Hartley04]

### Example - DSO [^Engel18]

#### Preparation

As introduced [here](www.lingtong.de/2020/04/02/Jacobian-Matrices/), we have 

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

| Variable used in `CoarseInitializer` | Variable used here      | Meaning      |
| -------- | ----------------------------- |-------- |
|`JbBuffer_new[i][0]` | $$\frac{\partial r_{i}}{\partial t^x}\frac{\partial r_{i}}{\partial \rho_{i}}$$| 第`i`个点提供的计算$$\mathbf{H}_{12}$$所需的中间量　|
|`JbBuffer_new[i][1]` | $$\frac{\partial r_{i}}{\partial t^y}\frac{\partial r_{i}}{\partial \rho_{i}}$$| 第`i`个点提供的计算$$\mathbf{H}_{12}$$所需的中间量　|
|`JbBuffer_new[i][2]` | $$\frac{\partial r_{i}}{\partial t^z}\frac{\partial r_{i}}{\partial \rho_{i}}$$| 第`i`个点提供的计算$$\mathbf{H}_{12}$$所需的中间量　|
|`JbBuffer_new[i][3]` | $$\frac{\partial r_{i}}{\partial \phi^x}\frac{\partial r_{i}}{\partial \rho_{i}}$$| 第`i`个点提供的计算$$\mathbf{H}_{12}$$所需的中间量　|
|`JbBuffer_new[i][4]` | $$\frac{\partial r_{i}}{\partial \phi^y}\frac{\partial r_{i}}{\partial \rho_{i}}$$| 第`i`个点提供的计算$$\mathbf{H}_{12}$$所需的中间量　|
|`JbBuffer_new[i][5]` | $$\frac{\partial r_{i}}{\partial \phi^z}\frac{\partial r_{i}}{\partial \rho_{i}}$$| 第`i`个点提供的计算$$\mathbf{H}_{12}$$所需的中间量　|
|`JbBuffer_new[i][6]` | $$\frac{\partial r_{i}}{\partial a}\frac{\partial r_{i}}{\partial \rho_{i}}$$| 第`i`个点提供的计算$$\mathbf{H}_{12}$$所需的中间量　|
|`JbBuffer_new[i][7]` | $$\frac{\partial r_{i}}{\partial b}\frac{\partial r_{i}}{\partial \rho_{i}}$$| 第`i`个点提供的计算$$\mathbf{H}_{12}$$所需的中间量　|
|`JbBuffer_new[i][8]` | $$r_{i} \frac{\partial r_{i}}{\partial \rho_{i}}$$| 第`i`个点提供的计算$$\mathbf{b}$$所需的中间量　|
|`JbBuffer_new[i][9]` | $$(\frac{\partial r_{i}}{\partial \rho_{i}})^2$$| 第`i`个点提供的计算$$\mathbf{H}_{22}$$以及$$\mathbf{H}_{22}^{-1}$$所需的中间量　|
|`Accumulator9 acc9` | | 计算$$\mathbf{H}_{11}$$和$$\mathbf{b}_{1}$$ |
|`Accumulator9 acc9SC` | | 计算$$\mathbf{H}_{12} \mathbf{H}_{22}^{-1} \mathbf{H}_{12}^{T}$$和$$\mathbf{H}_{12} \mathbf{H}_{22}^{-1} \mathbf{b}_{2}$$ |
|`Accumulator11 E`||计算总的光度误差|



### Literature

[^Engel18]: Engel, Jakob, Vladlen Koltun, and Daniel Cremers. "Direct Sparse Odometry." IEEE Transactions on Pattern Analysis and Machine Intelligence 40.3 (2018): 611-625.

[^Barfoot17]: Timothy D Barfoot. State Estimation for Robotics. Cambridge University Press, 2017.

[^Hartley04]: R. Hartley and A. Zisserman, Multiple View Geometry in Computer Vision, 2nd ed. Cambridge University Press, 2004.