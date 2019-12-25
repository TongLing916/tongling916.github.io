---
layout:     post
title:      "Probabilistic Semi-Dense Mapping from Highly Accurate Feature-Based Monocular SLAM"
date:       2019-12-22
author:     Tong
catalog: true
tags:
    - SLAM
---

> <<Probabilistic Semi-Dense Mapping from Highly Accurate Feature-Based Monocular SLAM>>

### A. Stereo Search Constraints

### B. Epipolar Search

#### Equation 1

$$
\mathbf{x}_{j}^{\top} \mathbf{F}_{j i} \mathbf{x}_{i}=\mathbf{x}_{j}^{\top} \mathbf{l}_{j}=0 \quad \rightarrow \quad v_{j}=m \cdot u_{j}+n，
$$
where $$x_i$$ and $$x_j$$ are pixels on frame $$i$$ and $$j$$. $$F_{ji}$$ is the fundamental matrix. $$l_j = \mathbf{F}_{j i} \mathbf{x}_{i}$$ represents the epipolar line on the frame $j$, with which we can obtain the exact coordinates of every pixel on the line if we know $$u_j$$ or $$v_j$$.


#### Equation 2

$$
e\left(u_{j}\right)=\frac{r_{I}^{2}}{\sigma_{I}^{2}}+\frac{r_{G}^{2}}{\sigma_{G}^{2}}, \quad r_{I}=I_{p}-I\left(u_{j}\right), \quad r_{G}=G_{p}-G\left(u_{j}\right)
$$


#### Equation 3

Due to the relationship
$$
\sigma_{G}^{2}=\theta \sigma_{I}^{2}
$$

$$
e\left(u_{j}\right)=\left(r_{I}^{2}+\frac{1}{\theta} r_{G}^{2}\right) \frac{1}{\sigma_{I}^{2}}
$$

#### Equation 4

$$
\frac{\partial e}{\partial u_{j}}=\frac{-2\left(r_{I} g+\frac{1}{\theta} r_{G} q\right)}{\sigma_{I}^{2}}
$$

#### Equation 5

$$
g \approx \frac{I\left(u_{j}+1\right)-I\left(u_{j}-1\right)}{2}, \quad q \approx \frac{G\left(u_{j}+1\right)-G\left(u_{j}-1\right)}{2}
$$

#### Equation 6

$$
u_{0}^{*}=u_{0}+\frac{g\left(u_{0}\right) r_{I}\left(u_{0}\right)+\frac{1}{\theta} q\left(u_{0}\right) r_{G}\left(u_{0}\right)}{g^{2}\left(u_{0}\right)+\frac{1}{\theta} q^{2}\left(u_{0}\right)}
$$

#### Equation 7

$$
\sigma_{u_{0}^{*}}^{2}=\frac{2 \sigma_{I}^{2}}{g^{2}\left(u_{0}\right)+\frac{1}{\theta} q^{2}\left(u_{0}\right)}
$$

#### Equation 8

$$
\rho_{p}\left(u_{j}\right)=\frac{\mathbf{r}_{z}^{j i} \overline{\mathbf{X}}_{p}\left(u_{j}-c_{x}\right)-f_{x} \mathbf{r}_{x}^{j i} \overline{\mathbf{X}}_{p}}{-\mathbf{t}_{z}^{j i}\left(u_{j}-c_{x}\right)+f_{x} \mathbf{t}_{x}^{j i}}
$$

#### Equation 9

$$
\begin{array}{c}{\rho_{j}=\rho_{p}\left(u_{0}^{*}\right)} \\ {\sigma_{\rho_{j}}=\max \left(\left|\rho_{p}\left(u_{0}^{*}+\sigma_{u_{0}^{*}}\right)-\rho_{j}\right|,\left|\rho_{p}\left(u_{0}^{*}-\sigma_{u_{0}^{*}}\right)-\rho_{j}\right|\right)}\end{array}
$$

### C. Inverse Depth Hypothesis Fusion

#### Equation 10

$$
\frac{\left(\rho_{a}-\rho_{b}\right)^{2}}{\sigma_{a}^{2}}+\frac{\left(\rho_{a}-\rho_{b}\right)^{2}}{\sigma_{b}^{2}}<5.99
$$

#### Equation 11

$$
\rho_{p}=\frac{\sum_{n} \frac{1}{\sigma_{\rho_{j}}^{2}} \rho_{j}}{\sum_{n} \frac{1}{\sigma_{\rho_{j}}^{2}}}, \quad \sigma_{\rho_{p}}^{2}=\frac{1}{\sum_{n} \frac{1}{\sigma_{\rho_{j}}^{2}}}
$$

### D. Intra-Keyframe Depth Checking, Smoothing and Growing


### E. Inter-Keyframe Depth Checking and Smoothing

#### Equation 12

$$
\begin{aligned} \mathbf{x}_{j}=& \mathbf{K} \mathbf{R}_{j i} \frac{1}{\rho_{p}} \overline{\mathbf{X}}_{p}+\mathbf{K} \mathbf{t}_{j i} \\ \rho_{j}=& \frac{\rho_{p}}{\mathbf{r}_{z}^{j i} \overline{\mathbf{X}}_{p}+\rho_{p} \mathbf{t}_{z}^{j i}} \end{aligned}
$$

#### Equation 13

$$
\frac{\left(\rho_{j}-\rho_{j, n}\right)^{2}}{\sigma_{\rho_{j, n}}^{2}}<3.84
$$

#### Equation 14

$$
d_{p}^{*}=\min _{d_{p}} \sum_{j, n}\left(d_{j, n}-d_{p} \mathbf{r}_{z}^{j i} \overline{\mathbf{X}}_{p}-\mathbf{t}_{z}^{j i}\right)^{2} \frac{1}{d_{j, n}^{4} \sigma_{\rho_{j, n}^{2}}}
$$
