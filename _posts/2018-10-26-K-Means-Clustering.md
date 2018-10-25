---
layout:     post
title:      "K-Means Clustering"
date:       2018-10-26
author:     Tong
catalog: true
tags:
    - Algorithm
---

## Basics

1. 参考[视频][youtube-k-means-clustering]

2. [Wikipedia][wiki-k-means]

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/post-k-means.PNG)

## [k-means++: The Advantages of Careful Seeding][paper-k-means++]

1. A variant that chooses centers at random from the data points, but weighs the data points according to their squared distance squared from the closet center already chosen. This augmentation improves both the speed and the accuracy of `k-means`.

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/post-k-means%2B%2B.PNG）


[youtube-k-means-clustering]: https://www.youtube.com/watch?v=9991JlKnFmk
[wiki-k-means]: https://zh.wikipedia.org/wiki/K-%E5%B9%B3%E5%9D%87%E7%AE%97%E6%B3%95
[paper-k-means++]: http://ilpubs.stanford.edu:8090/778/1/2006-13.pdf
