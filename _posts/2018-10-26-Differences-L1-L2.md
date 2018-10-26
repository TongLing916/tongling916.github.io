---
layout:     post
title:      "Differences between the L1-norm and the L2-norm"
date:       2018-10-26
author:     Tong
catalog: true
tags:
    - Algorithm
---

1. 参考这篇[博客][blog-difference]，我们可以的到以下的结论

$$
\begin{array}{|c|c|}
\hline
\textbf{L2 loss function} & \textbf{L1 loss function}                                              \\ \hline
not robust                & robust (就算error很大，与L2的平方相比还是相对小很多)                                     \\ \hline
stable                    & not stable (当Outlier和Inlier不是差很多，L1受Outlier位置的影响比较大)                   \\ \hline
Always one solution       & Possibly mutiple solutions (参考Euclidean distance 和 Manhattan distance) \\ \hline
\end{array}
$$

2. 这篇[博客][medium-difference]通俗地讲解了为什么L1 norm regularization有sparsity


[blog-difference]: http://www.chioka.in/differences-between-l1-and-l2-as-loss-function-and-regularization/
[medium-difference]: https://medium.com/mlreview/l1-norm-regularization-and-sparsity-explained-for-dummies-5b0e4be3938a