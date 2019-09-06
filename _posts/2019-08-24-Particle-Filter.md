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

![]()

$$w_{t}^{[m]}$$ is called _importance factor_.

The real trick of the particle filter is  _resampling_ or _importance resampling_. The algorithm draws with replacement $$M$$ particles from the temporary set $$\overline{\mathcal{X}}_{t}$$. The probability of drawing each particles is given by its importance weight.

### Properties

1. The first approximation error relates to the fact that only finitely many particles are used.

2. A second source error of error relates to the randomness introduced in the resampling phase. The resampling process induces a loss of diversity in the particle population. Such error is called _variance_ of the estimator. (Strategies: 1) One may reduce the frequency at which resampling takes place. 2) low variance sampling: instead of selecting samples independently of each other in the resampling process, the selection involves a sequential stochastic process (The algorithm computes a single random number and selects samples according to this number but with a probability propertional to the sample weight).

3. A third source of error pertains to the divergence of the proposal and target distribution. The efficiency of the particle filter relies crucially on the 'match' between the proposal and the target distribution.

4. A fourth and final disadvantage is known as the _particle deprivation problem_. Particle deprivation occurs as the result of random resampling: an unlucky series of random numbers can wipe out all particles near the true state. A popular solution to this problem is to add a small number of randomly generated particles into the set after each resampling process, regardless of the actual sequence of motion and measurement commands. 

### Implementation (Python)

### Implementation (C++)
