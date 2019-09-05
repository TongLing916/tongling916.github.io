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
\begin{array}{lll}{1 :} & {\text { Algorithm Kalman filter }\left(\mu_{t-1}, u_{t}, z_{t}\right) :} \\ {2 :} & {\overline{\mu}_{t}=A_{t} \mu_{t-1}+B_{t} u_{t}} \\ {\text { 3: }} & {\overline{\Sigma}_{t}=A_{t} \Sigma_{t-1} A_{t}^{T}+R_{t}} \\ {4 :} & {K_{t}=\overline{\Sigma}_{t} C_{t}^{T}\left(C_{t} \overline{\Sigma}_{t} C_{t}^{T}+Q_{t}\right)^{-1}} \\ {5 :} & {\mu_{t}=\overline{\mu}_{t}+K_{t}\left(z_{t}-C_{t} \overline{\mu}_{t}\right)} \\ {6 :} & {\Sigma_{t}=\left(I-K_{t} C_{t}\right) \overline{\Sigma}_{t}} \\ {7} & {\text { return } \mu_{t}, \Sigma_{t}}\end{array}
$$

where $$K_{t}$$ is called __Kalman gain__. It specifies the degree to which the measurement is incorporated into the new state estimate.

#### Shortcomings

* Gaussians are unimodal, that is, they posses a single maximum. Gaussian posteriors are a poor match for many global estimation problems in which many distinct hypotheses exist.

#### Implementation (Python)

#### Implementation (C++)


### Extended Kalman Filter

#### Introduction

#### Implementation (Python)

#### Implementation (C++)
