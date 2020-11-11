---
layout:     post
title:      "Eigen - Nitty Gritty"
date:       2020-3-26
author:     Tong
catalog: true
tags:
    - SLAM
---

### [Aliasing](http://eigen.tuxfamily.org/dox/group__TopicAliasing.html)

#### Summary

- Aliasing occurs when the same matrix or array coefficients appear both on the left- and the right-hand side of an assignment operator.
  - Aliasing is harmless with __coefficient-wise__ computations; this includes __scalar multiplication__ and matrix or array __addition__.
  - When you __multiply__ two matrices, Eigen assumes that aliasing occurs. If you know that there is no aliasing, then you can use `noalias()`.
  - In all other situations, Eigen assumes that there is no aliasing issue and thus gives the wrong result if aliasing does in fact occur. To prevent this, you have to use `eval()` or one of the `xxxInPlace()` functions.

#### Problem 1

```c++
MatrixXi mat(3,3); 
mat << 1, 2, 3,   4, 5, 6,   7, 8, 9;
cout << "Here is the matrix mat:\n" << mat << endl;
 
// This assignment shows the aliasing problem
mat.bottomRightCorner(2,2) = mat.topLeftCorner(2,2);
cout << "After the assignment, mat = \n" << mat << endl;
```

```bash
	
Here is the matrix mat:
1 2 3
4 5 6
7 8 9
After the assignment, mat = 
1 2 3
4 1 2
7 4 1
```

```c++
Matrix2i a; a << 1, 2, 3, 4;
cout << "Here is the matrix a:\n" << a << endl;
 
a = a.transpose(); // !!! do NOT do this !!!
cout << "and the result of the aliasing effect:\n" << a << endl;
```

```bash
Here is the matrix a:
1 2
3 4
and the result of the aliasing effect:
1 2
2 4
```

#### Solution 1

```c++
MatrixXi mat(3,3); 
mat << 1, 2, 3,   4, 5, 6,   7, 8, 9;
cout << "Here is the matrix mat:\n" << mat << endl;
 
// The eval() solves the aliasing problem
mat.bottomRightCorner(2,2) = mat.topLeftCorner(2,2).eval();
cout << "After the assignment, mat = \n" << mat << endl;
```

```c++
MatrixXf a(2,3); a << 1, 2, 3, 4, 5, 6;
cout << "Here is the initial matrix a:\n" << a << endl;
 
 
a.transposeInPlace();
cout << "and after being transposed:\n" << a << endl;
```

#### Problem 2

- You should not use `noalias()` when there is in fact aliasing taking place. If you do, then you may get wrong results:

```c++
MatrixXf matA(2,2); 
matA << 2, 0,  0, 2;
matA.noalias() = matA * matA;
cout << matA;
```

```bash
4 0
0 4
```

#### Solution 2

```c++
MatrixXf matA(2,2), matB(2,2); 
matA << 2, 0,  0, 2;
 
// Simple but not quite as efficient
matB = matA * matA;
cout << matB << endl << endl;
 
// More complicated but also more efficient
matB.noalias() = matA * matA;
cout << matB;
```

```bash
4 0
0 4

4 0
0 4
```

#### Problem 3

- Moreover, starting in __Eigen 3.3__, aliasing is __not assumed__ if the destination matrix is __resized__ and the product is not directly assigned to the destination. Therefore, the following example is also wrong:

```c++
MatrixXf A(2,2), B(3,2);
B << 2, 0,  0, 3, 1, 1;
A << 2, 0, 0, -2;
A = (B * A).cwiseAbs();
cout << A;
``` 

```bash
4 0
0 6
2 2
```

#### Solution 3

```c++
MatrixXf A(2,2), B(3,2);
B << 2, 0,  0, 3, 1, 1;
A << 2, 0, 0, -2;
A = (B * A).eval().cwiseAbs();
cout << A;
```

```bash
4 0
0 6
2 2
```



### [STL Containers](https://eigen.tuxfamily.org/dox/group__TopicStlContainers.html)

- An Eigen object is called "[fixed-size vectorizable](https://eigen.tuxfamily.org/dox/group__TopicFixedSizeVectorizable.html)" if it has fixed size and that size is a multiple of 16 bytes.
  - `Eigen::Vector2d`
  - `Eigen::Vector4d`
  - `Eigen::Vector4f`
  - `Eigen::Matrix2d`
  - `Eigen::Matrix2f`
  - `Eigen::Matrix4d`
  - `Eigen::Matrix4f`
  - `Eigen::Affine3d`
  - `Eigen::Affine3f`
  - `Eigen::Quaterniond`
  - `Eigen::Quaternionf`
- Using STL containers on __fixed-size vectorizable__ Eigen types, or classes having members of such types, requires taking the following two steps:
  - A 16-byte-aligned allocator must be used. Eigen does provide one ready for use: `aligned_allocator`.
  - If you want to use the `std::vector` container, you need to `#include <Eigen/StdVector>`.


```c++
std::map<int, Eigen::Vector4f, std::less<int>,
         Eigen::aligned_allocator<std::pair<const int, Eigen::Vector4f> > >
```

```c++
#include<Eigen/StdVector>

std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >
```

```c++
#include<Eigen/StdVector>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix2d)
std::vector<Eigen::Vector2d>
```



### Initialization

```c++
int dim = 15;

// OK
Eigen::MatrixXd H1(dim, dim);

// Bad (Compilation fails)
Eigen::Matrix<double, dim, dim> H2;
```


```c++
int dim = 15;

// OK
Eigen::MatrixXd H1(dim, dim);

// Bad (Compilation fails)
Eigen::Matrix<double, dim, dim> H2;
```


```c++
const int dim = 15;

// OK
Eigen::MatrixXd H1(dim, dim);

// OK
Eigen::Matrix<double, dim, dim> H2;
```


```c++
constexpr int dim = 15;

// OK
Eigen::MatrixXd H1(dim, dim);

// OK
Eigen::Matrix<double, dim, dim> H2;
```

### [Inverse Of a selfadjoint matrix](https://www.alexejgossmann.com/generalized_inverse/)

- A matrix $$H$$ is [selfadjoint](https://eigen.tuxfamily.org/dox/classEigen_1_1SelfAdjointEigenSolver.html) if it equals its adjoint. For real matrices, this means that the matrix is symmetric: it equals its transpose, e.g., Hessian matrix.

- [Relation beween matrix inversion and eigenvalues](https://www.quora.com/What-is-the-relation-of-a-matrix-inversion-and-eigenvalues)

- For a Hessian matrix (symmetric) arising from a normal equation, we have
$$
H = QDQ^{T},\quad H^{-1} = QD^{-1}Q^{T}
$$

- If $$H$$ is not invertible (determinant is zero), we will compute its pseudoinverse.

- Note that the `saes.eigenvalues()` are in __ascending__ order (from small to large).

```c++
  const double eps = 1e-8;
  Eigen::MatrixXd J(3, 3);
  J << 2, 1, 1, 0, 2, 3, 4, 5, 1;
  Eigen::MatrixXd H = J.transpose() * J;
  H.noalias() = 0.5 * (H + H.transpose());  // Ensure symmetry

  const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(H);
  const Eigen::MatrixXd H_inv =
      saes.eigenvectors() *
      Eigen::VectorXd((saes.eigenvalues().array() > eps)
                          .select(saes.eigenvalues().array().inverse(), 0))
          .asDiagonal() *
      saes.eigenvectors().transpose();

  LOG(WARNING) << "Determinant of H: " << H.determinant();
  LOG(WARNING) << "Inverse H\n" << H.inverse();
  LOG(WARNING) << "Pseudoinverse H\n"
               << H.completeOrthogonalDecomposition().pseudoInverse();
  LOG(WARNING) << "Inverse H (eigen-decomposition)\n" << H_inv;
```

### [Solve least squares problems](https://eigen.tuxfamily.org/dox/group__LeastSquares.html)

- Check [semi-definite](https://stackoverflow.com/questions/35227131/eigen-check-if-matrix-is-positive-semi-definite)