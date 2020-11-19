---
layout:     post
title:      "Schur Complement"
date:       2020-4-2
author:     Tong
catalog: true
tags:
    - SLAM
---

### Foreknowledge

- For a diagonal matrix

$$A=\left(\begin{array}{cccc}
a_{11} & 0 & \cdots & 0 \\
0 & a_{22} & \cdots & 0 \\
\vdots & \vdots & & \vdots \\
0 & 0 & \cdots & a_{n n}
\end{array}\right)$$,

its inverse is 

$$A^{-1}=\left(\begin{array}{cccc}
1 / a_{11} & 0 & \cdots & 0 \\
0 & 1 / a_{22} & \cdots & 0 \\
\vdots & \vdots & \vdots \\
0 & 0 & \cdots & 1 / a_{n n}
\end{array}\right)$$

### Schur Complement [^Barfoot17]

#### Problem Formulation

In Newton's or Gauss-Newton method, we often need to solve the following equation

$$\underbrace{\left[\begin{array}{ll}
\mathbf{H}_{11} & \mathbf{H}_{12} \\
\mathbf{H}_{12}^{T} & \mathbf{H}_{22}
\end{array}\right]}_{\mathbf{H}} \underbrace{\left[\begin{array}{l}
\delta \mathbf{x}_{1} \\
\delta \mathbf{x}_{2}
\end{array}\right]}_{\delta \mathbf{x}}=\underbrace{-\left[\begin{array}{l}
\mathbf{b}_{1} \\
\mathbf{b}_{2}
\end{array}\right]}_{-\mathbf{b}}$$

|Variable|Meaning|
|---|---|
|$$\mathbf{H}$$|Hessian matrix|
|$$\delta \mathbf{x}$$|Increment of variables to optimize|
|$$\mathbf{x}_{1}$$|`K` variables to optimize|
|$$\mathbf{x}_{2}$$|`M` variables to optimize|

#### Technique

If $$\mathbf{H}_{22}$$ has some sparse structure (diagonal matrix) like below, 

![](https://cdn.mathpix.com/snip/images/PsleFG3qjwvIi4xNH6qIO7jG1vqZIb3Flt6z1gPaADY.original.fullsize.png)

we can exploit this sparsity by premultiplying both sides by
$$\left[\begin{array}{cc}
\mathbf{I} & -\mathbf{H}_{12} \mathbf{H}_{22}^{-1} \\
\mathbf{0} & \mathbf{I}
\end{array}\right]$$, then we have 

$$\left[\begin{array}{cc}
\mathbf{H}_{11}-\mathbf{H}_{12} \mathbf{H}_{22}^{-1} \mathbf{H}_{12}^{T} & \mathbf{0} \\
\mathbf{H}_{12}^{T} & \mathbf{H}_{22}
\end{array}\right]\left[\begin{array}{c}
\delta \mathbf{x}_{1} \\
\delta \mathbf{x}_{2}
\end{array}\right]=-\left[\begin{array}{c}
\mathbf{b}_{1}-\mathbf{H}_{12} \mathbf{H}_{22}^{-1} \mathbf{b}_{2} \\
\mathbf{b}_{2}
\end{array}\right]$$

As a result, we can solve $$\delta \mathbf{x}_{1}$$ with 
$$(\mathbf{H}_{11}-\mathbf{H}_{12} \mathbf{H}_{22}^{-1} \mathbf{H}_{12}^{T})\delta \mathbf{x}_{1} = -(\mathbf{b}_{1}-\mathbf{H}_{12} \mathbf{H}_{22}^{-1} \mathbf{b}_{2})
$$ first, then substitute it into 
the second equation 
$$\mathbf{H}_{12}^{T} \delta \mathbf{x}_{1} + \mathbf{H}_{22} \delta \mathbf{x}_{2} = -\mathbf{b}_{2}
$$ and solve $$\delta \mathbf{x}_{2} = -\mathbf{H}_{22}^{-1}(\mathbf{b}_{2} + \mathbf{H}^T_{12}\delta\mathbf{x}_{1})$$.

$$\delta \mathbf{x}_{2} = -\mathbf{H}_{22}^{-1}(\mathbf{b}_{2} + \mathbf{H}^T_{12}\delta\mathbf{x}_{1})$$

This procedure brings the complexity of each solve down from $$O((K + M)^3)$$ without sparsity to $$O((K^3 + K^2M))$$ with sparsity, which is most beneficial when
$$K \ll  M$$.

To speed up solving $$\delta \mathbf{x}_{1}$$, we can use some decomposition techniques introduced soon.

#### Gauss-Newton

As explained in this [Themenrunde](https://github.com/TongLing916/tongling916.github.io/blob/master/documents/Themenrunde_LDSO.pdf), the variables used in Gauss-Newton are computed as follows (Suppose weight matrix is an identity matrix)

$$\mathbf{H} = \mathbf{J}^T\mathbf{J}$$

$$\mathbf{b} = \mathbf{J}^T\mathbf{e}$$

|Variable | Meaning|
|---|---|
|$$\mathbf{J}$$|Jacobian matrix|
|$$\mathbf{e}$$|residual vector|

### Decomposition 

- Cholesky Decomposition [^Barfoot17]

- $$LDL^T$$ Decomposition [^Hartley04]

### [Example - DSO](http://tongling916.github.io/2020/04/17/DSO-Schur-Complement/) [^Engel18]

### Visualization [^Sola16]

- Schur complement

```MATLAB
% Data Generation
N = 20; % nbr of poses, and number of landmarks

% I. Problem construction: factors
J = []; % start with empty Jacobian
k = 0;  % index for factors

% 1. motion
for n = 1:N-1    % index for poses
  k = k+1;       % add one factor
  J(k,n) = rand; % we simulate a non−zero block with just one scalar
  J(k,n+1) = rand;
end

% 2. landmark observations
f = 0;                   % index for landmarks
for n=1:N                % index for poses
  f = f+1;               % add one landmark
  jj = [0 randperm(5)];  % random sort a few recent landmarks
  m = randi(4);          % nbr. of landmark measurements
  for j = jj(1:m)        % measure m of the recent landmarks
    if j < f
      k = k+1;           % add one factor
      J(k,n) = rand;     % use state n
      J(k,N+f-j) = rand; % use a recent landmark
    end
  end
end

% Visualization
Jp = J(:, pr);          % Jacobian wrt. poses
Jl = J(:, lr);          % Jacobian wrt. landmarks
H = J'*J;               % Hessian matrix
pr = 1:N;               % poses
lr = N+1:N+f;           % landmarks
Hpp = H(pr,pr);         % poses Hessian
Hpl = H(pr,lr);         % cross Hessian
Hll = H(lr,lr);         % landmarks Hessian
Hsc = Hpl / Hll * Hpl'; % Schur complement
Spp = Hpp - Hsc;        % Schur complement of Hpp

figure(1), set(1,'name','Hessians')
subplot(3,3,1), spy(J), title 'J'
subplot(3,3,2), spy(Jp), title 'J_{pose}'
subplot(3,3,3), spy(Jl), title 'J_{landmarks}'
subplot(3,3,4), spy(H), title 'H'
subplot(3,3,5), spy(Hpp), title 'H_{11}'
subplot(3,3,6), spy(Hpl), title 'H_{12}'
subplot(3,3,7), spy(Hll), title 'H_{22}'
subplot(3,3,8), spy(Hsc), title 'H_{sc}'
subplot(3,3,9), spy(Spp), title 'H_{11} - H_{sc}'
```

- QR vs. Cholesky vs. Cholesky with Schur complement

```MATLAB
% Comparing: QR vs. Cholesky vs. Cholesky with Schur complement
N = 20; % nbr of poses, and number of landmarks

% I. Problem construction: factors
J = []; % start with empty Jacobian
k = 0; % index for factors

% 1. motion
for n = 1:N-1 % index for poses
  k = k+1; % add one factor
  J(k,n) = rand; % we simulate a non−zero block with just one scalar
  J(k,n+1) = rand;
end

% 2. landmark observations
f = 0; % index for landmarks
for n=1:N % index for poses
  f = f+1; % add one landmark
  jj = [0 randperm(5)]; % random sort a few recent landmarks
  m = randi(4); % nbr. of landmark measurements
  for j = jj(1:m) % measure m of the recent landmarks
    if j < f
      k = k+1; % add one factor
      J(k,n) = rand; % use state n
      J(k,N+f-j) = rand; % use a recent landmark
    end
  end
end

% II. Factorizing and plotting
% 1. QR
p = colamd(J); % column reordering
A = J(:,p); % reordered J
[~,Rj] = qr(J,0);
[~,Ra] = qr(A,0);
figure(1), set(1,'name','QR')
subplot(2,2,1), spy(J), title 'A = \Omega^{T/2} J'
subplot(2,2,2), spy(Rj), title 'R'
subplot(2,2,3), spy(A), title 'A^{\prime}'
subplot(2,2,4), spy(Ra), title 'R^{\prime}'

% 2. Cholesky
H = J'*J; % Hessian matrix
p = colamd(H); % column reordering
figure(2), set(2,'name','Cholesky')
subplot(2,2,1), spy(H), title 'H = J^T \Omega J'
subplot(2,2,2), spy(chol(H)), title 'R'
subplot(2,2,3), spy(H(p,p)), title 'H^{\prime}'
subplot(2,2,4), spy(chol(H(p,p))), title 'R^{\prime}'

% 3. Cholesky + Schur
pr = 1:N; % poses
lr = N+1:N+f; % landmarks
Hpp = H(pr,pr); % poses Hessian
Hpl = H(pr,lr); % cross Hessian
Hll = H(lr,lr); % landmarks Hessian
Spp = Hpp - Hpl / Hll * Hpl'; % Schur complement of Hpp
p = colamd(Spp); % column reordering

figure(3), set(3,'name','Schur + Cholesky')
subplot(2,3,1), spy(Spp), title 'S_{PP}'
subplot(2,3,2), spy(chol(Spp)), title 'R_{PP}'
subplot(2,3,4), spy(Spp(p,p)), title 'S_{PP}^{\prime}'
subplot(2,3,5), spy(chol(Spp(p,p))), title 'R_{PP}^{\prime}'
subplot(2,3,3), spy(Hll), title 'H_{LL}'
subplot(2,3,6), spy(inv(Hll)), title 'H_{LL}^{−1}'
```


### Literature

[^Engel18]: Engel, Jakob, Vladlen Koltun, and Daniel Cremers. "Direct Sparse Odometry." IEEE Transactions on Pattern Analysis and Machine Intelligence 40.3 (2018): 611-625.

[^Barfoot17]: Timothy D Barfoot. State Estimation for Robotics. Cambridge University Press, 2017.

[^Hartley04]: R. Hartley and A. Zisserman, Multiple View Geometry in Computer Vision, 2nd ed. Cambridge University Press, 2004.

[^Sola16]: Sola, Joan. "Course on SLAM." Institut de Robotica i Informatica Industrial (IRI) (2016).