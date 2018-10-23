---
layout: post
title: "MVG Chapter 2-Projective Geometry and Transformations of 2D"
date:       2018-10-20
author:     Tong
catalog: true
tags:
    - MVG
---

## 2.1 Planar geometry
1. What is _Tensor_? 

2. A significant advantage of the algebraic approach to geometry is that results derived in this way may more easily be used to derive algorithms and practical computational methods.

## 2.2 The 2D projective plane
### 2.2.1 Points and lines

3. __Homogeneous representation of lines.__ $$ ax + by + c = 0 $$ and $$ (ka)x + (kb)y + (kc) = 0 $$ are the same, for any non-zero constant $$k$$. Thus, the vectors $$ (a, b, c)^\intercal $$ and $$ k(a, b, c)^\intercal $$ represent the same line, for any non-zero $$k$$.

4. __Homogeneous representation of points.__ $$ (x,y,1)^\intercal $$.

5. __Result 2.1.__ _The points $$ x $$ lies on the line $$ l $$ if and only if $$ x^\intercal l = 0 $$.

6. __Degrees of freedom (dof).__ A line is specified by two parameters (the two independent ratios $$ \big\{a : b : c\big\} $$) and so has two degrees of freedom.

7. __Result 2.2.__ _The intersection of two lines $$ l $$ and $$ l^\prime $$ is the point $$ x =  l \times {l^\prime} $$._

8. __Result 2.4.__ _The line through two points $$ x $$ and $$ x^\prime $$ is the point $$ l =  x \times x^\prime $$._

### 2.2.2 Ideal points and the line at infinity

9. __Intersection of parallel lines.__ E.g. $$ ax+by+c = 0 $$ and $$ ax+by+c^\prime = 0 $$, represented by vectors $$ l = (a,b,c)^\intercal $$ and $$ l^\prime = (a,b,c^\prime)^\intercal $$. The intersection is $$ l \times l^\prime = (c^\prime - c)(b,-a,0)^\intercal $$, and igoring the scale factor $$ (c^\prime - c) $$, this is the point $$ (b,-a,0)^\intercal $$.

10. __Ideal points and the line at infinity.__ Homogeneous vectors $$ x = (x_1,x_2,x_3)^\intercal $$ such that $$ x_3 \neq 0 $$ correspond to finite points in $$ \mathbb{R^2} $$. One may augment $$ \mathbb{R^2} $$ by adding points with last coordinate $$x_3 = 0$$. The resulting space is the set of all homogeneous 3-vectors, namely the projective space $$ \mathbb{P^2} $$. The points with last coordinate $$ x_3 = 0 $$ are known as _ideal_ points, or points at infinity. The set of all ideal points may be written $$ (x_1,x_2,0)^\intercal $$, with a particular point specified by the ratio $$x_1:x_2$$. Note this set lies on a single line, the _line of infinity_, denoted by the vector $$l_\infty = (0,0,1)^\intercal$$. The line at inifinity can be also thought of as the set of directions of lines in the plane.

11. __A model for the projective plane.__ A fruitful way of thinking of $$ \mathbb{P^2} $$ is as a set of rays in $$ \mathbb{R^3} $$. <br>
![](http://www.robots.ox.ac.uk/~vgg/hzbook/hzbook2/WebPage/pngfiles/projgeomfigs-ray_model.png)
*Fig. A model of the projective plane*
	
12. __Reuslt 2.6. Duality principle.__ _To any theorem of 2-dimensional projective geometry there corresponds a dual theorem, which may be derived by interchanging the roles of points and lines in the original theorem._ Example: Result 2.2 and 2.4.

### 2.2.3 Conics and dual conics

13. The euqation of a conic in inhomogeneous coordinate is
$$
ax^2 + bxy + cy^2 + dx + ey + f = 0
$$ 
<br> i.e. a polynomial of degree 2. "Homogenizing" this by the replacements: $$ x \mapsto x_1/x_3, y \mapsto x_2/x_3 $$ gives
$$
a{x_1}^2 + b{x_1}{x_2} + c{x_2}^2 + d{x_1}{x_3} + e{x_2}{x_3} + f{x_3}^2 = 0 \quad (2.1) 
$$
<br> or in matrix form <br>
$$
x^\intercal C x = 0 \quad (2.2) 
$$
<br>where the conic coefficient matrix $$C$$ is given by <br>
$$
C = \begin{bmatrix} a & b/2 & d/2 \\ b/2 & c & e/2 \\ d/2 & e/2 & f \end{bmatrix} \quad (2.3) 
$$
<br> Note that the conic coefficient matrix is symmetric. C is a homogeneous representation of a conic. The conic has five degrees of freedom which can be thought of as the ratios $$ \big\{a : b : c : d ： e : f\big\} $$ or equivalently the six elemnts of a symmetric matrix less one for scale.

14. __Five points define a conic.__ Stacking the constraints from five points we obtain, where $$c=(a,b,c,d,e,f)^\intercal$$<br>
$$
\begin{bmatrix} {x_1}^2 & x_1 y_1 & {y_1}^2 & x_1 & y_1 & 1 \\ {x_2}^2 & x_2 y_2 & {y_2}^2 & x_2 & y_2 & 1 \\ {x_3}^2 & x_3 y_3 & {y_3}^2 & x_3 & y_3 & 1 \\ {x_4}^2 & x_4 y_4 & {y_4}^2 & x_4 & y_4 & 1 \\ {x_5}^2 & x_5 y_5 & {y_5}^2 & x_5 & y_5 & 1\end{bmatrix} c = 0 \quad (2.4) 
$$

15. __Result 2.7__ _The line $$ l $$ tangent to $$ C $$ at a point $$ x $$ on $$ C $$ is given by $$ l = Cx $$._

16. __Dual conics.__ The conic $$C$$ defined above is more properly termed a _point_ conic, as it defines an equation on points. Given the duality result 2.6 of $$\mathbb{P^2} $$ it is not surprising that there is also a conic which defines an equation on lines. This _dual_ (or line) conic is also represented by a $$3 \times 3$$ matrix, which we denote as $$C^*$$. A line $$l$$ _tangent_ to the conic $$C$$ satisfies $$l^\intercal C^* l = 0$$. The notation $$C^*$$ indicates that $$C^*$$ is the adjoint matrix of $$C$$. For a non-singular symmetric matrix $$C^*=C^{-1}$$.
<br>Note: The equation for a dual conic is straightforward to derive in the case that $$C$$ has full rank.
![](http://www.robots.ox.ac.uk/~vgg/hzbook/hzbook2/WebPage/pngfiles/projgeomfigs-line_conic.png)
*Fig. Dual conics are also known as conic envelopes.*

17. __Degenerate conics.__ If the matrix $$C$$ is not of full rank, then the conic is termed degenerate. Degenerate point conics include two lines (rank 2), and a repeated line (rank 1). E.g. $$C = l m^\intercal + m l^\intercal$$.
	
## 2.3 Projective transformations

18. __Defition 2.9.__ A _projectivity_ is an invertible mapping _h_ from $$ \mathbb{ P^2 } $$ to itself such that three points $$x_1$$, $$x_2$$ and $$x_3$$ lie on the same line if and only if $$h(x_1), h(x_2), h(x_3)$$do.
<br> Projectivities form a group since the inverse of a projectivity is also a projectivity, and so is the composition of two projectivities. A projectivity is also called a _collineation_, a _projective transformation_ or a _homography_.

19. __Theorem 2.10.__ _A mapping h : $$ \mathbb{ P^2 } \rightarrow \mathbb{ P^2 } $$ is a projectivity if and only if there exists a non-singular 3 $$ \times $$ 3 matrix $$ \mathbf{H} $$ such that for any point in $$ \mathbb{ P^2 } $$ represented by a vector $$x$$ it is true that $$ h(x) = Hx $$_.
<br> To interpret this theorem, any point in $$\mathbb{P^2}$$ is represented as a homogeneous 3-vector. $$x$$, and $$Hx$$ is a linear mapping of homogeneous coordinates.

20. __Definition 2.11. Projective transformation.__ A planar projective transformation is a linear transformation on homogeneous 3-vectors represented by a non-singular $$3 \times 3$$ matrix:<br>
$$
\begin{pmatrix} x^\prime_1 \\ x^\prime_2 \\ x^\prime_3 \end{pmatrix} = \begin{bmatrix} h_{11} & h_{12} & h_{13} \\ h_{21} & h_{22} & h_{23} \\ h_{31} & h_{32} & h_{33} \end{bmatrix} \begin{pmatrix} x_1 \\ x_2 \\ x_3 \end{pmatrix} \quad (2.5) 
$$
<br>or more briefly, $$x^\prime = Hx$$

16. What is a _homogeneous_ matrix?

17. What is _perspectivity_?

18. What does _contravariant or covariant_ mean?

19. What is _projective linear group_?

20. What is _general linear group_?

21. What is _affine group_?

22. What is _Euclidean group_?

23. What is _oriented Euclidean group_?

24. What is _invariant_?

25. What are _isometries_?

26. What is _similarity_?

27. What is _metric structure_?

28. What is _affinity_?

29. What is _SVD_?

30. What is _cross ratio_?

31. What is the key diffrence between a projective and affine transformation?

32. What is the decomposition of a projective transformation?

33. How can we calculate the number of invariants?

34. What are concurrent lines?

35. __Transformation of lines.__

36. __Result 2.13.__ _Under a point transformation $$ x^\prime = Hx $$, a conic $$ C $$ transforms to $$ C^\prime = H^{-\intercal}CH^{-1} $$._

37. __Result 2.14.__ _Under a point transformation x^\prime = Hx, a dual conic $$ C^* $$ transforms to $$ C^{*\prime} = HC^*H^\intercal $$._

38. __Result 2.16.__ _The number of functionally independent invariants is equal to, or greater than, the number of degrees of freedom of the configuration less the number of degrees of freedom of the transformation._

39. 






