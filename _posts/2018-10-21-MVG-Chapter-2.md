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
<br> To interpret this theorem, any point in $$\mathbb{P^2}$$ is represented as a homogeneous 3-vector, $$x$$, and $$Hx$$ is a linear mapping of homogeneous coordinates.
<br> __Proof.__ Let $$x_1,x_2$$ and $$x_3$$ lie on a line $$l$$. Thus $$l^\intercal x_i = 0$$ for $$i = 1,...,3$$. Let $$H$$ be a non-singular $$3 \times 3$$ matrix. One verifies that $$l^\intercal H^{-1} H x_i = 0$$. Thus, the points $$H x_i$$ all lie on the line $$ H^{-\intercal} l $$, and collinearity is preserved by the transformation.

20. __Definition 2.11. Projective transformation.__ A planar projective transformation is a linear transformation on homogeneous 3-vectors represented by a non-singular $$3 \times 3$$ matrix:<br>
$$
\begin{pmatrix} x^\prime_1 \\ x^\prime_2 \\ x^\prime_3 \end{pmatrix} = \begin{bmatrix} h_{11} & h_{12} & h_{13} \\ h_{21} & h_{22} & h_{23} \\ h_{31} & h_{32} & h_{33} \end{bmatrix} \begin{pmatrix} x_1 \\ x_2 \\ x_3 \end{pmatrix} \quad (2.5) 
$$
<br>or more briefly, $$x^\prime = Hx$$
<br>Note that the matrix $$H$$ in this equation may be changed by multiplication by an arbitrary non-zero sclae factor without altering the projective transformation. Consequently we say that $$H$$ is a _homography_ matrix, since as in the homogeneous representation of a point, only the ratio of the matrix elements is significant. There are eight independent ratios amongst the nine elements of $$H$$, and it follows that a projective transformation has eight degrees of freedom.
<br>A projective transformation projects every figure into a projectively equivalent figure, leaving all its projective properties invariant.

21. __Mappings between planes.__ Projection along rays through a common point (the centre of projection) defines a mapping from one plane to another. If a coordinate system is defined in each plane and points are represented in homogeneous coordinates, then the _central projection_ mapping may be expressed by $$x^\prime = Hx$$ where $$H$$ is a non-singular $$3 \times 3$$ matrix.
<br> Actually, if the two coordinate systems defined in the two planes are both Euclidean (rectilinear) coordinate systems then the mapping defined by central projection is more restricted than an arbitrary projective transformation. It is called _perspectivity_ rather than a full projectivity, and may be represented by a transformation with six degrees of freedom.

### 2.3.1 Transformations of lines and conics

22. __Transformation of lines.__ According to the proof of theorem 2.10, under the point transformation $$x^\prime = Hx$$, a line transforms as 
$$
l^\prime = H^{-\intercal} l  \quad (2.6)
$$
<br> One may alternatively write $$l^{\prime \intercal} = l^\intercal H^{-1}$$. Note the fundamentally different way in which lines and points transform. Points transform according to $$H$$, whereas lines (as rows) transform according to $$H^{-1}$$. This may be explained in terms of "covariant" or "contravariant" behaviour. One says that points transform _contravariantly_ and lines transform _covariantly_.

23. __Transformation of conics.__ Under a point transformation $$x^\prime = Hx$$, (2.2) becomes
$$
\begin{align}
x^\intercal C x & = x^{\prime \intercal}[H^{-1}]^\intercal C H^{-1} x^\prime  \\
                & = x^{\prime \intercal}H^{-\intercal} C H^{-1} x^\prime
\end{align}
$$
<br> which is a quadratic form $$x^{\prime T} C^\prime x\prime$$ with $$C^\prime = H^{-\intercal} C H^{-1}$$

24. __Result 2.13.__ _Under a point transformation $$ x^\prime = Hx $$, a conic $$ C $$ transforms to $$ C^\prime = H^{-\intercal}CH^{-1} $$._
<br> THe presence of $$H^{-1}$$ in this transformation may be expressed by saying that a conic transforms _covariantly_.

25. __Result 2.14.__ _Under a point transformation x^\prime = Hx, a dual conic $$ C^* $$ transforms to $$ C^{*\prime} = HC^*H^\intercal $$._

## 2.4 A hierarchy of transformations

26. The group of invertible  $$n \times n$$ matrices with real elements is the (real) _general linear group_ on $$n$$ dimensions, or $$GL(n)$$. To obtain the _projective linear group_ the matrices related by a scalar multiplier are identified, giving $$PL(n)$$ (this is a quotient group of $$GL(n)$$). In the case of projective transformations of the plane $$n = 3$$.
<br> The important subgroups of $$PL(3)$$ include _affine group_, which is the subgroup of $$PL(3)$$ consisting of matrices for which the last row is (0, 0, 1), 
<br> and the _Euclidean group_, which is a subgroup of the affine group for which in addition the upper left hand $$2 \times 2$$ matrix is orthogonal. 
<br> One may also identify the _oriented Euclidean group_ in which the upper left hand $$2 \times 2$$ matrix has determinant 1.

27. __Invariants.__ An alternative to describing the transformation _algebraically_, i.e. as a matrix acting on coordiantes of a point or curve, is to describe the transformation in terms of those elements or quantities that are preserved or _invariant_.
<br>Note: a Euclidean transformation (translation and rotation). A similarity (e.g. translation, rotation and isotropic scaling).

### 2.4.1 Class I: Isometries

28. Isometries are transformations of the plane $$\mathbb{R^2}$$ that preserve Euclidean distance (from _iso_ = same, _metric_ = measure). An isometry is represented as
$$
\begin{pmatrix} x^\prime \\ y^\prime \\ 1 \end{pmatrix} = \begin{bmatrix} \epsilon \cos \theta & - \sin \theta & t_x \\ \epsilon \sin \theta & \cos \theta & t_y \\ 0 & 0 & 1 \end{bmatrix} \begin{pmatrix} x \\ y \\ 1 \end{pmatrix} 
$$
<br> where $$\epsilon = \pm 1$$. If $$\epsilon = 1$$ then the isometry is _orientation-preserving_ and is a _Euclidean_ transformation (a composition of a translation and rotation). If $$\epsilon = -1$$ then the isometry reverses orientation. An example is the composition of a reflection.
<br> Euclidean transformations model the motion of a rigid object. However, the orientation reversing isometries often arise as ambiguities in structure recovery.
<br> A planar Euclidean transformation can be written more concisely in block form as 
$$
x^\prime = H_E x = \begin{bmatrix} R & t \\ 0^\intercal & 1 \end{bmatrix} x  \quad (2.7)
$$
<br> where $$R$$ is a $$2 \times 2$$ rotation matrix (an orthogonal matrix such that $$R^\intercal R = R R^\intercal = I$$), $$t$$ a translation 2-vector, and $$0$$ a null 2-vector. 
<br> A Euclidean transformation is also known as a _displacement_.
<br> A planar Euclidean transformation has three degrees of freedom, one for the rotation and two for the translation. Thus three parameters must be specified in order to define the transformation. The transformation can be computed from two point correspondences (because one point can provide two equations).

29. __Invariants.__ Length (the distance between two points), angle (the angle between two lines), and area.

30. __Groups and orientation.__ An isometry is orientation-preserving if the upper left hand $$2 \times 2$$ matrix has determinant 1. Orientation-_preserving_ isometries form a group, orientation-_reversing_ ones do not.

### 2.4.2 Class II: Similarity transformations

31. A similarity transformation (or more simply a _similarity_) is an isometry composed with an isotropic scaling. In the case of a Euclidean transformation composed with a scaling (i.e. no reflection) the similarity has matrix representation
$$
\begin{pmatrix} x^\prime \\ y^\prime \\ 1 \end{pmatrix} = \begin{bmatrix} s \cos \theta & - s \sin \theta & t_x \\ s \sin \theta & s \cos \theta & t_y \\ 0 & 0 & 1 \end{bmatrix} \begin{pmatrix} x \\ y \\ 1 \end{pmatrix} \quad (2.8)
$$
<br> This can be written more concisely in block form as 
$$
x^\prime = H_S x = \begin{bmatrix} sR & t \\ 0^\intercal & 1 \end{bmatrix} x  \quad (2.9)
$$
<br> where the scalar _s_ represents the isotropic scaling. A similarity transformation is also known as an _equi-form_ transformation, because it preserves "shape"(form). A planar similarity transformation has four degrees of freedom, the scaling accounting for one more degree of freedom tham a Euclidean transformation. A similarity can be computed from two point correspondences.

32. __Invariants.__ Angles between lines are not affected by rotation, translation or isotropic scaling, and so are similarity invariants. In particular parallel lines are mapped to parallel lines. The length between two points is not a similarity invariant, but the _ratio_ of two lengths/areas is an invariant.

33. __Metric structure.__ The description _metric structure_ implies that the structure is defined up to a similarity.

### 2.4.3 Class III: Affine transformation

34.An affine transformation (or more simply an _affinity_) is a non-singular linear transformation followed by a translation. It has the matrix representation
$$
\begin{pmatrix} x^\prime \\ y^\prime \\ 1 \end{pmatrix} = \begin{bmatrix} a_{11} & a_{12} & t_x \\ a_{21} & a_{22} & t_y \\ 0 & 0 & 1 \end{bmatrix} \begin{pmatrix} x \\ y \\ 1 \end{pmatrix} \quad (2.10)
$$
<br> or in block form
$$
x^\prime = H_A x = \begin{bmatrix} A & t \\ 0^\intercal & 1 \end{bmatrix} x  \quad (2.11)
$$
<br> with $$A$$ a $$2 \times 2$$ non-singular. A planar affine transformation has six degrees of freedom corresponding to the six matrix elements. The transformation can be computed from three point correspondences.
<br> A helpful way to understand the geometric effects of the linear component $$A$$ of an affine transformation is as the composition of two fundamental transformations, namely rotations and non-isotropic scalings. The affine matrix $$A$$ can always be decomposed as 
$$
A = R(\theta) R(-\phi) D R(\phi)
$$
<br> where $$R(\theta)$$ and $$R(\phi)$$ are rotations by $$\theta$$ and $$\phi$$ respectively, and $$D$$ is a diagonal matrix:
$$
D = \begin{bmatrix} \lambda_1 & 0 \\ 0 & \lambda_2 \end{bmatrix}
$$
<br> This decomposition follows directly from the SVD.
<br> The affine matrix $$A$$ is hence seen to be the concatenation of a rotation (by $$\phi$$); a scaling by $$\lambda_1$$ and $$\lambda_2$$ respectively in the (rotated) $$x$$ and $$y$$ directions; a rotation back (by $$-\phi$$); and a finally another rotation (by $$\theta$$). The only "new" geometry, compared to a similarity, is the non-isotropic scaling. This accounts for the two extra degrees of freedom possessed by an affinity over a similarity. They are the angle $$\phi$$ specifying the scaling direction, and the ratio of the scaling parameters $$\lambda_1 : \lambda_2$$. 
<br> The essence of an affinity is this scaling in orthogonal directions, oriented at a particular angle.
![](http://www.robots.ox.ac.uk/~vgg/hzbook/hzbook2/WebPage/pngfiles/projgeomfigs-affine_def_all.png)
*Fig. Distortions arising from a planar affine transformation*

35. __Invariants.__ Because an affine transformation includes non-isotropic scaling, the similarity invariants of length ratios and angles between lines are not preserved under an affinity. Three important invariants are:
	- __Parallel lines.__ 
	- __Ratio of lengths of parallel line segments.__
	- __Ratio of areas.__
	
36. An affinity is orientation-preserving or -reversing according to whether $$\det A$$ is positive or negative respectively. Since $$\det A = \lambda_1 \lambda_2$$ the property depends only on the sign of the scalings.

### 2.4.4 Class IV: Projective transformations


30. What is _cross ratio_?

31. What is the key diffrence between a projective and affine transformation?

32. What is the decomposition of a projective transformation?

33. How can we calculate the number of invariants?

34. What are concurrent lines?


38. __Result 2.16.__ _The number of functionally independent invariants is equal to, or greater than, the number of degrees of freedom of the configuration less the number of degrees of freedom of the transformation._

39. 






