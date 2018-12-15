---
layout: post
title: "Appendix 5: Least-squares Minimization"
date:       2018-12-18
author:     Tong
catalog: true
tags:
    - MVG
---

> All contents come from <<Multiple View Geometry in Computer Vision>>.

### A5.1 Solution of linear equations

1. __Algorithm A5.1.__ Linear least-squares solution to an over-determined full-rank set of linear equations.
![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/post-algorithm-A5.1.JPG)

1. __Algorithm A5.2.__ General solution to deficient-rank system
![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/post-algorithm-A5.2.JPG)

### A5.2 The pseudo-inverse

1. Given a square diagonal matrix $$D$$, we define its _pseudo-inverse_ to be the diagonal matrix $$D^+$$ such that $$D^+_ {ii} = \left\{\begin{matrix}
0 \quad if \; D_{ii} = 0\\
D^{-1}_{ii} \quad otherwise.
\end{matrix}\right.$$

2. Consider an $$m \times n$$ matrix $$A$$ with $$m\geq n$$. Let the SVD of $$A$$ be $$A=UDV^T$$. We define the _pseudo-inverse_ of $$A$$ to be the matrix $$A^+=VD^+U^T \quad \quad (A5.1)$$.

3. __Result A5.1.__ The least-squares solution to an $$m \times n$$ system of equations $$Ax=b$$ of rank $$n$$ is given by $$x=A^+ b$$. In the case of a deficient-rank system, $$x=A^+ b$$ is the solution that minimizes $$\left \| x \right \|$$.

4. Define the null space $$N_L X=\left \{ x^T \| x^TX=0 \right \}$$.

5. __Result 5.2.__ Let $$A$$ be a symmetric matrix, then $$A^{+X} \stackrel{def}{=} X(X^TAX)^{-1}X^T=A^+$$ if and only if N_L(X)=N_L(A).
^{-1}A^T$$.

#### A5.2.1 Linear least-squares using normal equations

1. __Algorithm A5.3.__ Linear least-squares using the normal equations
![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/post-algorithm-A5.3.JPG)

2. __Result 5.3.__ If $$A$$ is an $$m \times n$$ matrix of rank $$n$$, then $$A^+ = (A^TA)

3. __Weighted linear least-squares problems.__ One desires to solve a weighted least-squares problem of the form $$Ax-b=0$$ by minimizing the $$C$$-norm $$\left \| Ax-b \right \|_ C$$ of the error. One can obtain the weighted normal equations: $$(A^TCA)x=A^TCb \quad \quad (A5.3)$$.

### A5.3 Least-squares solution of homogeneous equations

1. __Algorithm A5.4.__ Least-squares solution of a homogeneous system of linear equations
![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/post-algorithm-A5.4.JPG)

### A5.4 Least-squares solution to constrained systems

1. __Algorithm A5.5.__ Algorithm for constrained minimization
![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/post-algorithm-A5.5.JPG)

#### A5.4.1 More constrained minimization

1. __Algorithm A5.6.__ Algorithm for constrained minimization, subject to a span-space constraint
![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/post-algorithm-A5.6.JPG)

#### A5.4.2 Yet another minimization problem

1. __Algorithm A5.7.__ Least-squares solution of homogeneous equations subject to the constraint $$\left \| Cx \right \|=1$$.
![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/post-algorithm-A5.7.JPG)
