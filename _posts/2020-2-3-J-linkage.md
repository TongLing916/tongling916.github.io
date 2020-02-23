---
layout:     post
title:      "J-linkage: Robust fitting of multiple models [2008]"
date:       2020-2-3
author:     Tong
catalog: true
tags:
    - SLAM
---

> http://www.diegm.uniud.it/fusiello/demo/jlk/

### J-linkage [^Toldo2008]

* Paper: _Robust Multiple Structures Estimation with J-linkage_

* Problem: fitting multiple instances of a model to data corrupted by noise and outliers.

* Theory: Based on random sampling and conceptual data representation. Each point is represented with the characteristic function of the set of random models that fit the point.

* A tailored agglomerative clustering, called __J-linkage__, is used to group points belonging to the same model.

* Advantages
    * prior specification of the number of models not required
    * parameters tuning not required

#### Definitions

* __Pseudo-outlier__: outliers to the structure of interest but inliers to a different structure.

* __Minimal sample sets (MSS)__: The minimal sets of data points necessary to estimate the model.

* __Concensus set (CS)__: The CS of a model is the set of points such that their distance from the model is less than a threshold $$\sigma$$.

* __Preference set (PS)__: Imagine to build a $$N \times M$$ matrix where entry $$(i, j)$$ is $$1$$ if point $$i$$ belongs to the CS of model $$j$$, 0 otherwise. Each __column__ of that matrix is the __characteristic function__ of the CS of a model hypothesis. Each __row__ indicates which models a point has given consensus to, i.e., which models it prefers. We call this the _preference set_ (PS) of a point.

* __Conceptual representation__: The characteristic function of the preference set of a point can be regarded as a conceptual representation of that point. Points belonging to the same structure will have similar conceptual representations, in other words, they will cluster in the _conceptual space_.

* __Jaccard distance__: Given two sets A and B, the Jaccard distance is
$$
d_{\mathrm{J}}(A, B)=\frac{|A \cup B|-|A \cap B|}{|A \cup B|}
$$.

The Jaccard distance measures the degree of overlap of the two sets and ranges from 0 (identical sets) to 1 (disjoint sets).

#### Clustering

* The general agglomerative clustering algorithm proceeds in a bottom-up manner: Starting from all singletons, each sweep of the algorithm merges the two clusters with the smallest distance. The way the distance between clusters is computed produces different flavours of the algorithm, namely the simple linkage, complete linkage and average linkage.

##### Algorithm: J-linkage

* __Input__: the set of data points, each point represented by its preference set (PS)

* __Output__: clusters of points belonging to the same model

* Process
    1. Put each point in its own cluster.
    2. Define the PS of a cluster as the _intersection_ of the PSs of its points.
    3. Among all current clusters, pick the two clusters with the smallest _Jaccard distance_ between the respective PSs.
    4. Replace these two clusters with the union of the two original ones.
    5. Repeat from step 3 while the smallest Jaccard distance is lower than 1.


### T-linkage [2014]

### Literature

[^Toldo2008]: R. Toldo and A. Fusiello. Robust multiple structures estimation with J-linkage. In European Conference on Computer Vision, pages 537–547, 2008.
