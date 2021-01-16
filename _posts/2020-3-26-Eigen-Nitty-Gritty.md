---
layout:     post
title:      "Eigen - Nitty Gritty"
date:       2020-3-26
author:     Tong
catalog: true
tags:
    - SLAM
---

### [Storage orders](https://eigen.tuxfamily.org/dox/group__TopicStorageOrders.html)

- The default in Eigen is column-major. 

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

### Inverse matrix

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/Ax=b.jpg)

```c++
#include <iostream>

#include <glog/logging.h>
#include <Eigen/Dense>

int main() {
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  Eigen::Matrix3d H;
  H << 30, 10, 20, 0, 40, 10, 0, 0, 30;
  H = H.selfadjointView<Eigen::Upper>();

  std::cout << "H: det(H) = " << H.determinant() << std::endl
            << H << std::endl
            << std::endl;

  {
    std::cout << "Inverse H" << std::endl
              << H.inverse() << std::endl
              << std::endl;
  }

  {
    // NOTE: Do not use pseudoInverse() to solve Ax=b
    // https://eigen.tuxfamily.org/dox/classEigen_1_1CompleteOrthogonalDecomposition.html#a3c89639299720ce089435d26d6822d6f
    const Eigen::MatrixXd H_pinv =
        H.completeOrthogonalDecomposition().pseudoInverse();
    std::cout << "Pseudoinverse H" << std::endl
              << H_pinv << std::endl
              << std::endl;
  }

  {
    // NOTE: H must selfadjoint
    // https://eigen.tuxfamily.org/dox/classEigen_1_1SelfAdjointEigenSolver.html
    constexpr double eps = 1e-8;
    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(H);
    const Eigen::MatrixXd H_inv =
        saes.eigenvectors() *
        Eigen::VectorXd((saes.eigenvalues().array() > eps)
                            .select(saes.eigenvalues().array().inverse(), 0))
            .asDiagonal() *
        saes.eigenvectors().transpose();
    std::cout << "Inverse H (eigen-decomposition)" << std::endl
              << H_inv << std::endl
              << std::endl;
  }

  {
    // NOTE: H must be pos. def.
    // http://eigen.tuxfamily.org/dox/classEigen_1_1LLT.html
    Eigen::MatrixXd Hinv = Eigen::MatrixXd::Identity(H.rows(), H.cols());
    H.llt().solveInPlace(Hinv);
    std::cout << "Inverse H (LLT)" << std::endl
              << Hinv << std::endl
              << std::endl;
  }

  {
    // NOTE: H must be pos. or neg. semi. def.
    // http://eigen.tuxfamily.org/dox/classEigen_1_1LDLT.html
    Eigen::MatrixXd Hinv = Eigen::MatrixXd::Identity(H.rows(), H.cols());
    H.ldlt().solveInPlace(Hinv);
    std::cout << "Inverse H (LDLT)" << std::endl
              << Hinv << std::endl
              << std::endl;
  }

  return 0;
}
```

### [Solve least squares problems](https://eigen.tuxfamily.org/dox/group__LeastSquares.html)

> [Solving linear systems with Eigen (and OpenCV)](https://www.patrikhuber.ch/blog/2015/01/solving-linear-systems-with-eigen/)

> [Linear algebra and decompositions](https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html  )

- Check [semi-definite](https://stackoverflow.com/questions/35227131/eigen-check-if-matrix-is-positive-semi-definite)
- Check [invertibility](https://eigen.tuxfamily.org/dox/classEigen_1_1FullPivLU.html) (the determinant is often __not__ a good way of checking if a matrix is [invertible](https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html).)

```c++
// Jx = -r
// (Normal equation) J^T J x = -J^T r
class NormalEquationSolver {
 public:
  static VecX Solve(const MatXX& J, const VecX& r,
                    const METHOD& method = LDLT) {
    CHECK_GT(J.cols(), 0);
    CHECK_GT(J.rows(), 0);
    CHECK_EQ(J.rows(), r.rows());

    switch (method) {
      case INVERSE: {
        // ~0.063 ms with invertibility check
        // ~0.029 ms without invertibility check
        const MatXX H = J.transpose() * J;
        const MatXX b = -J.transpose() * r;
        Eigen::FullPivLU<MatXX> lu(H);
        lu.setThreshold(1e-6);
        if (!lu.isInvertible()) {
          LOG(ERROR) << "H is not invertible! Something wrong? det(H)="
                     << H.determinant();
          break;
        }
        return H.inverse() * b;
      }

      case PSEUDO_INVERSE: {
        // ~0.048 ms
        const MatXX H = J.transpose() * J;
        const MatXX b = -J.transpose() * r;
        return H.completeOrthogonalDecomposition().pseudoInverse() * b;
      }

      case LDLT: {
        // ~0.013 ms (fastest)
        // https://eigen.tuxfamily.org/dox/classEigen_1_1LDLT.html#aa257dd7a8acf8b347d5a22a13d6ca3e1
        // NOTE: H must be pos. def. or neg. def.
        const MatXX H = J.transpose() * J;
        const MatXX b = -J.transpose() * r;
        return H.ldlt().solve(b);
      }

      case QR: {
        // ~0.022 ms
        return J.colPivHouseholderQr().solve(-r);
      }

      case SVD: {
        // ~0.077 ms (most accurate and reliable)
        return J.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(-r);
      }

      case PCG: {
        LOG(WARNING) << "NOT IMPLEMENTED YET! Use LDLT instead :)";
        break;
      }

      default:
        break;
    }
    return (J.transpose() * J).ldlt().solve(-J.transpose() * r);
  }
};
```

### [Inverse of SE3](https://github.com/rpng/open_vins/blob/v2.3/ov_core/src/utils/quat_ops.h#L446)

It is better to analytically compute the inverse of a SE3 instead of numerically.

```c++
inline Eigen::Matrix4d InvSE3(const Eigen::Matrix4d &T) {
  Eigen::Matrix4d Tinv = Eigen::Matrix4d::Identity();
  Tinv.block<3, 3>(0, 0) = T.block<3, 3>(0, 0).transpose();
  Tinv.block<3, 1>(0, 3).noalias() =
      -Tinv.block<3, 3>(0, 0) * T.block<3, 1>(0, 3);
  return Tinv;
}
```

### [View](https://eigen.tuxfamily.org/dox/group__QuickRefPage.html)

```c++
  Eigen::Matrix4d Q;
  Q << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16;
  std::cout << Q << std::endl << std::endl;
  //  1  2  3  4
  //  5  6  7  8
  //  9 10 11 12
  // 13 14 15 16

  Eigen::Matrix4d O;
  O.setOnes();

  {
    Eigen::MatrixXd U = Q.selfadjointView<Eigen::Upper>();
    std::cout << U << std::endl << std::endl;
    // 1  2  3  4
    // 2  6  7  8
    // 3  7 11 12
    // 4  8 12 16

    Eigen::MatrixXd L = Q.selfadjointView<Eigen::Lower>();
    std::cout << L << std::endl << std::endl;
    //  1  5  9 13
    //  5  6 10 14
    //  9 10 11 15
    // 13 14 15 16
  }

  {
    Eigen::MatrixXd U = Q.triangularView<Eigen::Upper>();
    std::cout << U << std::endl << std::endl;
    // 1  2  3  4
    // 0  6  7  8
    // 0  0 11 12
    // 0  0  0 16

    Eigen::MatrixXd L = Q.triangularView<Eigen::Lower>();
    std::cout << L << std::endl << std::endl;
    //  1  0  0  0
    //  5  6  0  0
    //  9 10 11  0
    // 13 14 15 16
  }

  {
    Eigen::Matrix4d Q1 = Q, Q2 = Q;
    Q1.triangularView<Eigen::Upper>() = O;
    std::cout << Q1 << std::endl << std::endl;
    //  1  1  1  1
    //  5  1  1  1
    //  9 10  1  1
    // 13 14 15  1

    Q2.triangularView<Eigen::Lower>() = O;
    std::cout << Q2 << std::endl;
    // 1  2  3  4
    // 1  1  7  8
    // 1  1  1 12
    // 1  1  1  1
  }
```


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


