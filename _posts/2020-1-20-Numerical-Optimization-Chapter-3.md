---
layout:     post
title:      "Chapter 3. Line Search Methods"
date:       2020-1-20
author:     Tong
catalog: true
tags:
    - SLAM
---

- The iteration is given by $$x_{k+1}=x_{k}+\alpha_{k} p_{k} \quad (3.1)$$.

- The positive scalar $$\alpha_{k}$$ is called the _step length_.

- $$p_{k}$$ is a _descent direction_ - one for which $$p_{k}^{T} \nabla f_{k}<0$$.

- Usually, $$p_{k}=-B_{k}^{-1} \nabla f_{k}$$, $$B_{k}$$ is a symmetric and nonsingular matrix.

### 3.1. Step Length

- The ideal choice would be the global minimizer of the univariate function $
\phi(\cdot)$ defined by$$
\phi(\alpha)=f\left(x_{k}+\alpha p_{k}\right), \quad \alpha>0
$$, but in general, it is too expensive. more pratical strategies perform an _inexact_ line search to identify a step length that achieves adequate reductions in $$f$$ at minimal cost.

- The line search is done in two stages: A __bracketing phase__ finds an interval containning desirable step lengths, and a __bisection__ or __interpolation phase__ computes a good step length within this interval.

- The _Wolfe conditions_ (3.6)
    - _Armijo condition_: $$\alpha_{k}$$ should give _sufficient decrease_ in the objective function $$f$$.
        - $$f\left(x_{k}+\alpha p_{k}\right) \leq f\left(x_{k}\right)+c_{1} \alpha \nabla f_{k}^{T} p_{k}$$.
    - _Curvature condition_: 1) rules out unacceptably short steps. 2) ensures that the slope of $$\phi$$ at $$\alpha_{k}$$ is greater than $$c_2$$ times the initial slope $$\phi^{\prime}(0)$$. This makes sense because 1) if the slope $$\phi^{\prime}(\alpha)$$ is strongly negative, we have an indication that we can reduce $$f$$ significantly by moving further along the chosen direction. 2) if $$\phi^{\prime}(\alpha)$$ is only slightly negative or even positive, it is a sign that we cannot expect much more decrease in $$f$$ in this direction.
        - $$\nabla f\left(x_{k}+\alpha_{k} p_{k}\right)^{T} p_{k} \geq c_{2}\nabla f_{k}^{T} p_{k}$$

- The _strong Wolfe conditions_ ($$0<c_{1}<c_{2}<1$$)
    - $$f\left(x_{k}+\alpha_{k} p_{k}\right) \leq f\left(x_{k}\right)+c_{1} \alpha_{k} \nabla f_{k}^{T} p_{k}$$
    - $$\left|\nabla f\left(x_{k}+\alpha_{k} p_{k}\right)^{T} p_{k}\right| \leq c_{2}\left|\nabla f_{k}^{T} p_{k}\right|$$
    - The only __difference__ with the Wolfe conditions is that we no longer allow the derivative $$\phi^{\prime}\left(\alpha_{k}\right)$$ to be too positive. Hence, we excluse points that far from stationary points of $$\phi$$.

- __Lemma 3.1__
    - Suppose that $$
f: \mathbb{R}^{n} \rightarrow \mathbf{R}
$$ is continuously differntiable. Let $$p_k$$ be a descent direction at $$x_k$$, and assume that $$f$$ is bounded below along the ray $$
\left\{x_{k}+\alpha p_{k} | \alpha>0\right\}
$$. Then if $$0<c_{1}<c_{2}<1$$, there exist intervals of step lengths satisfying the Wolfe conditions and the strong Wolfe conditions.

- __Property__ of the Wolfe conditions
    - Scale-invariant

- __Usage__ of the Wolfe conditions
    - Most line search methods.
    - Quasi-Newton methods.

- The _Goldstein conditions_
    - Step length $$\alpha$$ achieves sufficient decrease while preventing $$\alpha$$ being too small.
    - $$
f\left(x_{k}\right)+(1-c) \alpha_{k} \nabla f_{k}^{T} p_{k} \leq f\left(x_{k}+\alpha_{k} p_{k}\right) \leq f\left(x_{k}\right)+c \alpha_{k} \nabla f_{k}^{T} p_{k}, $$
0<c<1 / 2
$$
$$

- A __disadvantage__ of the Goldstein conditions vis-a-vis the Wolfe conditions is that the first equality may exclude all minimizers of $$\phi$$.

- __Usage__ of the Goldstein conditions
    - Newton-type methods.
    - __NOT__ well suited for quasi-Newton methods.

- _Backtracking_ approach
    - Core: By choosing step lengths appropriately, we can use __only__ _Armijo condition_ to terminate the line search procedure.
    - In this procedure, the initial step length $$\bar{\alpha}$$ is chosen to be 1 in Newton and quasi-Newton methods, but can have different values in other algorithms such as steepest descent or conjugate gradient.
    - This simple and popular strategy for terminating a line search is well suited for Newton methods but is less appropriate for quasi-Newton and conjugate gradient methods.

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/backtracking_line_search.PNG?token=AEVZO3IRCDC4CTR3G7HODA26LBKY4)

### 3.2. Convergence of Line Search Methods

- We discuss requirements on search directions $$p_{k}$$ in this section, focusing on one key property: the angle $$\theta_{k}$$ between $$p_{k}$$ and the steepest descent direction $$-\nabla f_{k}$$, defined by
$$
\cos \theta_{k}=\frac{-\nabla f_{k}^{T} p_{k}}{\left\|\nabla f_{k}\right\|\left\|p_{k}\right\|}.
$$

- The steepest descent method is globally convergent.

- __Theorem 3.2.__: Consider any iteration of the form (3.1), where $$p_{k}$$ is a descent direction and $$\alpha_{k}$$ satisfies the Wolfe conditions (3.6). Suppose that $$f$$ is bounded below in $$\mathbb{R}^{n}$$ and that $$f$$ is continuously differentiable in an open set $$\mathcal{N}$$ containing the level set $$\mathcal{L} \stackrel{\text { def}}{=}\left\{x: f(x) \leq f\left(x_{0}\right)\right\}$$, where $$x_{0}$$ is the starting point of the iteration. Assume also that the gradient $$\nabla f(x)$$ is Lipschitz continuous on $$\mathcal{N}$$, that is, there exists a constant $$L > 0$$ such that
$$
\|\nabla f(x)-\nabla f(\tilde{x})\| \leq L\|x-\tilde{x}\|, \quad \text { for all } x, \tilde{x} \in \mathcal{N} \quad (3.13)
$$
Then,
$$
\sum_{k \geq 0} \cos ^{2} \theta_{k}\left\|\nabla f_{k}\right\|^{2}<\infty \quad (3.14)
$$

- Inequality (3.14), which we call the _Zoutendijk condition_, implies that
$$
\cos ^{2} \theta_{k}\left\|\nabla f_{k}\right\|^{2} \rightarrow 0   \quad (3.16)
$$
This limit can be used in turn to derive global convergence results for line search algorithms.

- If our method for choosing the search direction $$p_{k}$$ in the iteration (3.1) ensures that the angle $$\theta_{k}$$ defined by (3.12) is bounded away from $$90^{\circ}$$, there is a positive constant $$\delta$$ such that
$$
\cos \theta_{k} \geq \delta>0, \quad \text { for all } k.    \quad(3.17)
$$
It follows immediately from (3.16) that
$$
\lim _{k \rightarrow \infty}\left\|\nabla f_{k}\right\|=0. \quad(3.18)
$$

- In other words, we can be sure that the gradient norms $$\left\|\nabla f_{k}\right\|$$ converge to zero, provided that the search directions are never too close to orthogonality with the gradient.

- We use the term _globally convergent_ to refer to algorithms for which the property (3.18) is satisfied.

- We cannot guarantee that the method converges to a minimizer, but only that it is attracted by stationary points. Only by making additional requirements on the search direction $$p_{k}$$ - by introducing negative curvature information from the Hessian $$\nabla^{2} f\left(x_{k}\right)$$, for example - can we strength these results to include convergence to a local minimum.

- A weaker result
$$
\liminf _{k \rightarrow \infty}\left\|\nabla f_{k}\right\|=0 \quad (3.21)
$$.
In other words, just a subsequence of the gradient norms $$\left\|\nabla f_{k_{j}}\right\|$$ converges to zero, rather than the whole sequence.

- Consider _any_ algorithm for which
    - every iteration produces a decrease in the objective function.
    - every $$m$$th iteration is a steepest descent step, with step length chosen to satisfy the Wolfe or Goldstein conditions.
Then, since $$\cos \theta_{k}=1$$ for the steepest descent steps, the result (3.21) holds.

### 3.3. Rate of Convergence

- The challenge is to design algorithms that incorporate both properties: good global convergence guarantees and a rapid rate of convergence.

#### Convergence rate of steepest descent

__Theorem 3.4__
Suppose that $$f: \mathbb{R}^{n} \rightarrow \mathbb{R}$$ is twice continuously differentiable, and that the iterates generated by the steepest-descent method with exact line searches converge to a point $$x^{*}$$ at which the Hessian matrix $$\nabla^{2} f\left(x^{*}\right)$$ is positive definite. Let $$r$$ be any scalar satisfying
$$
r \in\left(\frac{\lambda_{n}-\lambda_{1}}{\lambda_{n}+\lambda_{1}}, 1\right),
$$
where $$\lambda_{1} \leq \lambda_{2} \leq \cdots \leq \lambda_{n}$$ are the eigenvalues of $$\nabla^{2} f\left(x^{*}\right)$$. Then for all $$k$$ sufficiently large, we
have
$$
f\left(x_{k+1}\right)-f\left(x^{*}\right) \leq r^{2}\left[f\left(x_{k}\right)-f\left(x^{*}\right)\right]
$$

#### Newton's method

- Newton iteration, for which the search is given by
$$
\left\{\left\|\nabla f_{k}\right\|\right\}. \quad (3.30)
$$

__Theorem 3.5__
Suppose that $$f$$ is twice differentiable and that the Hessian $$\nabla^{2} f\left(x^{*}\right)$$ is Lipschitz continuous (see (A.42)) in a neighborhood of a solution $$x^{*}$$ at which the sufficient conditions (Theorem 2.4) are satisfied. Consider the iteration $$x_{k+1}=x_{k}+p_{k}$$, where $$p_{k}$$ is given by (3.30). Then
- (i) if the starting point $$x^{0}$$  is sufficiently close to $$x^{*}$$ , the sequence of iterates converges to $$x^{*}$$ ;
- (ii) the rate of convergence of $$\left\{x_{k}\right\}$$ is quadratic; and
- (iii) the sequence of gradient norms $$\left\{\left\|\nabla f_{k}\right\|\right\}$$ converges quadratically to zero.

- As the iterates generated by Newton’s method approach the solution, the Wolfe (or Goldstein) conditions will accept the step length $$\alpha_{k}$$ for all large $$k$$.

#### Quasi-Newton methods

### 3.4 Newton's Method with Hessian Modification

### 3.5. Step-Length Selection Algorithms

#### Interpolation

#### The initial step length

#### A line search algorithm for the Wolfe conditions

####
