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

### A5.2 The pseudo-inverse

1. __Algorithm A5.2.__ General solution to deficient-rank system
![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/post-algorithm-A5.2.JPG)

#### A5.2.1 Linear least-squares using normal equations

1. __Algorithm A5.3.__ Linear least-squares using the normal equations
![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/post-algorithm-A5.3.JPG)

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
