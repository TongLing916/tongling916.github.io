---
layout:     post
title:      "Kalman Filter"
date:       2019-8-24
author:     Tong
catalog: true
tags:
    - SLAM
---

> <<Probablistic Robotics>>

### Bayes Filter

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/post-bayes-filter.PNG)

### Kalman Filter

The Kalman filter implements belief computation for continuous states. It is not applicable to discrete or hybrid state spaces.

At time _t_, the belief is represented by the mean $$\mu_{t}$$ and the covariance $$\Sigma_{t}$$.

#### Four assumptions

* Markov assumptions: the state is a complete summary of the past. (past and future data are independent if one knows the current state $$x_{t}$$)

* The next state probability
$$
p\left(x_{t} | u_{t}, x_{t-1}\right)
$$ must be a __linear__ function in its arguments with added Gaussian noise. $$x_{t}$$ is the state vector, $$u_{t}$$ is the control vector. The random variable $$\varepsilon_{t}$$ is a Gaussian random vector that models the randomness in the state transition. Its mean is zero and its covariance will be denoted $$R_{t}$$.

$$x_{t}=A_{t} x_{t-1}+B_{t} u_{t}+\varepsilon_{t}$$

$$
\begin{array}{l}p\left(x_{t} | u_{t}, x_{t-1}\right) =\operatorname{det}\left(2 \pi R_{t}\right)^{-\frac{1}{2}} \exp \left\{-\frac{1}{2}\left(x_{t}-A_{t} x_{t-1}-B_{t} u_{t}\right)^{T} R_{t}^{-1}\left(x_{t}-A_{t} x_{t-1}-B_{t} u_{t}\right)\right\}\end{array}
$$

* The measurement probability must also be __linear__ in its arguments, with added Gaussian noise. $$z_{t}$$ is the measurement vector and the vector $$\delta_{t}$$ describes the measurement noise. The distribution of $$\delta_{t}$$ is a multivariate Gaussian with zero mean and covariance $$Q_{t}$$.

$$
z_{t}=C_{t} x_{t}+\delta_{t}
$$

$$
p\left(z_{t} | x_{t}\right)=\operatorname{det}\left(2 \pi Q_{t}\right)^{-\frac{1}{2}} \exp \left\{-\frac{1}{2}\left(z_{t}-C_{t} x_{t}\right)^{T} Q_{t}^{-1}\left(z_{t}-C_{t} x_{t}\right)\right\}
$$


* The initial belief $$\operatorname{bel}\left(x_{0}\right)$$ must be normal distributed.

$$
\operatorname{bel}\left(x_{0}\right)=p\left(x_{0}\right)=\operatorname{det}\left(2 \pi \Sigma_{0}\right)^{-\frac{1}{2}} \exp \left\{-\frac{1}{2}\left(x_{0}-\mu_{0}\right)^{T} \Sigma_{0}^{-1}\left(x_{0}-\mu_{0}\right)\right\}
$$

#### Algorithm

$$
\begin{array}{lll}{1 :} & {\text { Algorithm Kalman filter }\left(\mu_{t-1}, u_{t}, z_{t}\right) :} \\ {2 :} & {\overline{\mu}_{t}=A_{t} \mu_{t-1}+B_{t} u_{t}} \\ {\text {3: }} & {\overline{\Sigma}_{t}=A_{t} \Sigma_{t-1} A_{t}^{T}+R_{t}} \\ {4 :} & {K_{t}=\overline{\Sigma}_{t} C_{t}^{T}\left(C_{t} \overline{\Sigma}_{t} C_{t}^{T}+Q_{t}\right)^{-1}} \\ {5 :} & {\mu_{t}=\overline{\mu}_{t}+K_{t}\left(z_{t}-C_{t} \overline{\mu}_{t}\right)} \\ {6 :} & {\Sigma_{t}=\left(I-K_{t} C_{t}\right) \overline{\Sigma}_{t}} \\ {7} & {\text { return } \mu_{t}, \Sigma_{t}}\end{array}
$$

where $$K_{t}$$ is called __Kalman gain__. It specifies the degree to which the measurement is incorporated into the new state estimate.

#### Shortcomings

* Gaussians are unimodal, that is, they posses a single maximum. Gaussian posteriors are a poor match for many global estimation problems in which many distinct hypotheses exist.

#### Implementation (Python)
```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from numpy.linalg import inv


class KalmanFilter():
    # ===========================================================
    # KalmanFilter.init(x0, P0)
    # -------------------
    #
    # Initialization of the kalman filter class
    #
    # x_k = x0
    # P_k = P0
    #
    # Inputs:
    # - x   :  State vector (n x 1)
    # - P   :  State covariance matrix (n x n)
    # ===========================================================
    def __init__(self, x0, P0):
        self.x = x0
        self.P = P0
        self.I = np.identity(P0.shape[0])  # for simplicity of computation

    # ===========================================================
    # prediction(Ak, Rk, [uk], [Bk])
    # --------------------------------
    #
    # Prediction step of the continous Kalman filter
    #
    # \bar{x}_k = A_k * x_{k-1} + B_k * u_k
    # \bar{P}_k = A_k * P_{k-1} * A_k^T + R_k
    #
    # Inputs:
    # - Ak    :  State transition matrix (n x n)
    # - Rk    :  System noise matrix Qk = E{w w^T}
    # - uk    :  Control variable of known inputs (optional) (n x 1)
    # - Bk    :  Control matrix (optional) (n x n)
    # ===========================================================
    def prediction(self, Ak, Rk, uk=0, Bk=0):
        if Bk == 0:
            Bk = np.zeros(1)
        self.x = np.dot(Ak, self.x) + np.dot(Bk, uk)
        self.P = np.dot(np.dot(Ak, self.P), Ak.T) + Rk

    # ===========================================================
    # update(zk, Ck, Qk)
    # ---------------
    #
    # Update step of the Kalman filter
    #
    # K_k        = \bar{P}_k * C_k^T * (C_k * \bar{P}_k  * C_k^T + Q_k)^{-1}
    # innovation = z_k - C_k * \bar{x}_k
    # x_k        = \bar{x}_k + K_k * innovation
    # P_k        = (I - K_k * C_k) * \bar{P}_k
    #
    # with
    #
    # Inputs:
    # - zk   :  Measurement vector
    # - Ck   :  Measurement matrix
    # - Qk   :  Measurement covariance noise matrix Qk = E{v v^T}
    # ===========================================================
    def update(self, zk, Ck, Qk):               
        temp1 = np.dot(self.P, Ck.T)
        temp2 = np.dot(np.dot(Ck, self.P), Ck.T) + Qk
        temp3 = inv(temp2)
        K_k = np.dot(temp1, temp3)

        innovation = zk - np.dot(Ck, self.x)
        self.x = self.x + np.dot(K_k, innovation)
        self.P = np.dot(self.I - np.dot(K_k, Ck), self.P)
```


#### Implementation (C++)

### Extended Kalman Filter

#### Introduction

The extended Kalman filter (EKF) calculates an approximation to the true belief. It represents this approximation by a Gaussian.

#### Linearization

THe key idea underlying the EKF is called __linearization__. EKFs utilize a method called (first order) Taylor expansion.

$$
\begin{aligned} x_{t} &=g\left(u_{t}, x_{t-1}\right)+\varepsilon_{t} \\ z_{t} &=h\left(x_{t}\right)+\delta_{t} \end{aligned}
$$

$$
g^{\prime}\left(u_{t}, x_{t-1}\right) \quad :=\quad \frac{\partial g\left(u_{t}, x_{t-1}\right)}{\partial x_{t-1}}
$$

$$
\begin{aligned} g\left(u_{t}, x_{t-1}\right) & \approx g\left(u_{t}, \mu_{t-1}\right)+\underbrace{g^{\prime}\left(u_{t}, \mu_{t-1}\right)}_{=G_{t}}\left(x_{t-1}-\mu_{t-1}\right) \\ &=g\left(u_{t}, \mu_{t-1}\right)+G_{t}\left(x_{t-1}-\mu_{t-1}\right) \end{aligned}
$$

$$
\begin{array}{l}{p\left(x_{t} | u_{t}, x_{t-1}\right)} \\ {\approx \operatorname{det}\left(2 \pi R_{t}\right)^{-\frac{1}{2}} \exp \left\{-\frac{1}{2}\left[x_{t}-g\left(u_{t}, \mu_{t-1}\right)-G_{t}\left(x_{t-1}-\mu_{t-1}\right)\right]^{T}\right.} \\ {\left.\quad R_{t}^{-1}\left[x_{t}-g\left(u_{t}, \mu_{t-1}\right)-G_{t}\left(x_{t-1}-\mu_{t-1}\right)\right]\right\}}\end{array}
$$

$$
\begin{aligned} h\left(x_{t}\right) & \approx h\left(\overline{\mu}_{t}\right)+\underbrace{h^{\prime}\left(\overline{\mu}_{t}\right)}_{=H_{t}}\left(x_{t}-\overline{\mu}_{t}\right) \\ &=h\left(\overline{\mu}_{t}\right)+H_{t}\left(x_{t}-\overline{\mu}_{t}\right) \end{aligned}
$$

$$
\begin{aligned} p\left(z_{t} | x_{t}\right)=& \operatorname{det}\left(2 \pi Q_{t}\right)^{-\frac{1}{2}} \exp \left\{-\frac{1}{2}\left[z_{t}-h\left(\overline{\mu}_{t}\right)-H_{t}\left(x_{t}-\overline{\mu}_{t}\right)\right]^{T}\right.\\ &\left.Q_{t}^{-1}\left[z_{t}-h\left(\overline{\mu}_{t}\right)-H_{t}\left(x_{t}-\overline{\mu}_{t}\right)\right]\right\} \end{aligned}
$$

#### Algorithm

$$
\begin{array}{l}{\text { Algorithm Extended Kalman filter }\left(\mu_{t-1}, \Sigma_{t-1}, u_{t}, z_{t}\right) :} \\ {\quad \overline{\mu}_{t}=g\left(u_{t}, \mu_{t-1}\right)} \\ {\quad \begin{array}{l}{\overline{\Sigma}_{t}=G_{t} \sum_{t-1} G_{t}^{T}+R_{t}} \\ {K_{t}=\overline{\Sigma}_{t} H_{t}^{T}\left(H_{t} \overline{\Sigma}_{t} H_{t}^{T}+Q_{t}\right)^{-1}} \\ {\mu_{t}=\overline{\mu}_{t}+K_{t}\left(z_{t}-h\left(\overline{\mu}_{t}\right)\right)} \\ {\Sigma_{t}=\left(I-K_{t} H_{t}\right) \overline{\Sigma}_{t}} \\ {\text { return } \mu_{t}, \Sigma_{t}}\end{array}}\end{array}
$$

#### Shortcomings

* EKFs are incapable of representing multimodal beliefs. (Remedy: multi-hypothesis (extended) Kalman filter (MHEKF))

* An important limitation of the EKF arises from the fact that it approximates state transitions and measurements using linear Taylor expansions.

#### Implementation (Python)

#### Implementation (C++)
