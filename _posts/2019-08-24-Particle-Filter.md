---
layout:     post
title:      "Particle Filter"
date:       2019-8-24
author:     Tong
catalog: true
tags:
    - SLAM
---

> <<Probablistic Robotics>>

### Introduction

The key idea of particle filter is to represent the posterior $$b e l\left(x_{t}\right)$$ by a set of random state samples drawn from this posterior.

In particle filters, the samples of a posterior distribution are called __particles__ and are denoted as follows. Each particle $$x_{t}^{[m]}$$ (with $$1 \leq m \leq M$$) is a hypothesis to what the true world state may be at time $$t$$. Here, $$M$$ denotes the number of particles in the particle set $$\mathcal{X}_{t}$$.

$$
\mathcal{X}_{t} :=x_{t}^{[1]}, x_{t}^{[2]}, \ldots, x_{t}^{[M]}
$$

### Algorithm

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/post-particle-filter.PNG?token=AEVZO3LOVFHVDISFO66GSUK6NRROW)

$$w_{t}^{[m]}$$ is called _importance factor_.

The real trick of the particle filter is  _resampling_ or _importance resampling_. The algorithm draws with replacement $$M$$ particles from the temporary set $$\overline{\mathcal{X}}_{t}$$. The probability of drawing each particles is given by its importance weight.

### Properties

1. The first approximation error relates to the fact that only finitely many particles are used.

2. A second source error of error relates to the randomness introduced in the resampling phase. The resampling process induces a loss of diversity in the particle population. Such error is called _variance_ of the estimator. (Strategies: 1) One may reduce the frequency at which resampling takes place. 2) low variance sampling: instead of selecting samples independently of each other in the resampling process, the selection involves a sequential stochastic process (The algorithm computes a single random number and selects samples according to this number but with a probability propertional to the sample weight).

3. A third source of error pertains to the divergence of the proposal and target distribution. The efficiency of the particle filter relies crucially on the 'match' between the proposal and the target distribution.

4. A fourth and final disadvantage is known as the _particle deprivation problem_. Particle deprivation occurs as the result of random resampling: an unlucky series of random numbers can wipe out all particles near the true state. A popular solution to this problem is to add a small number of randomly generated particles into the set after each resampling process, regardless of the actual sequence of motion and measurement commands.

### Implementation (Python)

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

class Particle:
    def __init__(self, state, weight):
        self.state = state
        self.weight = weight
```

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import pi, sqrt, exp

import numpy as np
from numpy.linalg import inv, det

from particle import Particle

class ParticleFilter():
    # ===========================================================
    # ParticleFilter.init(x0, P0)
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
    def __init__(self, particles, map):
        self.particles = particles
        self.map = map

    # ===========================================================
    # prediction_measurement(state)
    # -------------------
    #
    # Mesurement function:
    #
    # zk = h(xk)
    #
    # Inputs:
    # - state  :  State vector (n x 1)
    #
    # Return:
    # - prediction : Measurement predicted according to the state
    # ===========================================================
    def prediction_measurement(self, state):
        pos = state[0]
        prediction = min(abs(self.map - pos))
        return prediction

    # ===========================================================
    # prediction_and_update(Ak, Rk, zk, Qk)
    # --------------------------------
    #
    # Prediction and update of the Particle filter
    #
    #
    # Inputs:
    # - Ak    :  State transition matrix (n x n)
    # - Rk    :  System noise matrix Rk = E{w w^T}
    # - zk    :  Measurement vector
    # - Qk    :  Measurement covariance noise matrix Qk = E{v v^T}
    # ===========================================================
    def prediction_and_update(self, Ak, Rk, zk, Qk):
        # prediction
        new_weights = []
        for particle in self.particles:
            particle.state = np.dot(Ak, particle.state) + np.dot(Rk, np.random.randn(np.shape(Rk)[0], 1))
            prediction = self.prediction_measurement(particle.state)
            particle.weight = float(particle.weight * (1/sqrt(2*pi*Qk) * exp(-0.5*(zk-prediction)**2 / Qk)))
            new_weights.append(particle.weight)


        # normalization of weights
        new_weights = np.array(new_weights)
        total_weights = np.sum(new_weights)
        if total_weights != 0:
            new_weights = new_weights / total_weights
            for particle in self.particles:
                particle.weight = particle.weight / total_weights

        N_eff = 1./ np.sum(new_weights**2) # rule of thumb

        # resampling
        if N_eff < len(self.particles)*0.75:
            cumulative_weights = np.cumsum(new_weights)
            new_particles = self.particles
            if total_weights != 0:
                for i in range(len(self.particles)):
                    r = np.random.rand()
                    index = next(index for index, val in enumerate(cumulative_weights) if val > r)
                    new_particles[i] = self.particles[index]
                    new_particles[i].weight = 1./len(new_particles)
            self.particles = new_particles    
```

### Implementation (C++)
