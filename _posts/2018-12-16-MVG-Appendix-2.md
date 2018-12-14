---
layout: post
title: "Appendix 2: Gaussian (Normal) and Chi Distributions"
date:       2018-12-16
author:     Tong
catalog: true
tags:
    - MVG
---

> All contents come from <<Multiple View Geometry in Computer Vision>>.

### A2.1 Gaussian probability distribution

1. The variables $$x_i$$ are said to conform to a joint Gaussian distribution, if the probability distribution of $$X$$ is of the form $$P(\overline{X}+\Delta X) = (2\pi)^{-N/2}det(\Sigma ^{-1})^{1/2}exp(-(\Delta X)^T\Sigma ^{-1}(\Delta X)/2) \quad (A2.1)$$ for some positive-semidefinite matrix $$\Sigma ^{-1}$$. It may be verified that $$\overline{X}$$ and $$\Sigma$$ are the mean and covariance of the distribution.WW

2. In the special case where $$\Sigma$$ is a scalar matrix $$\Sigma = \sigma ^2I$$ the Gaussian PDF takes a simple form $$P(X) = (\sqrt{2\pi}\sigma)^{-N}exp(-\sum_{i=1}^{N}(x_i - \overline{x}_i)^2/2\sigma^2)$$ where $$X=(x_1,x_2,...,x_N)^T$$. This distribution is called an _isotropic Gaussian distribution__.

### A2.2 Chi distribution

1. The $$\chi ^2_n$$ distribution is the distribution of the sum of squares of $$n$$ independent Gaussian random variables.

2. If $$v$$ is Gaussian random vector with mean $$\overline{v}$$ and covariance matrix $$\Sigma$$, then the value of $$(v-\overline{v})^T\Sigma ^+(v-\overline{v})$$ satisfies a $$\chi ^2_r$$ distribution, where $$r = rank\Sigma$$.
