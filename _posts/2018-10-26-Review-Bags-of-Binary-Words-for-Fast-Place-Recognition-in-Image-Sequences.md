---
layout:     post
title:      "Review - Bags of Binary Words for Fast Place Recognition in Image Sequences"
date:       2018-10-26
author:     Tong
catalog: true
tags:
    - SLAM
---

<blockquote><p><<Bags of Binary Words for Fast Place Recognition in Image Sequences>> 读后感</p></blockquote>

This paper can be found in this [website][paper-Bag-of-Words]

## 1. Introduction

1. Why do we need loop closures?
<br> After an exploratory period, when ares non-observed for long are re-orbserved, standard matching algorithm fail. When they are robustly detected, loop closures provide correct data association. The same methods used for detection can be used for robot relocation after track lost.

2. What are the methods for loop detection or robot relocation?
<br> For small environments, map-to-image methods. See the results in this [paper][paper-comparison-loop-closing-tech].
<br> For large environments, image-to-image methods such as [FAB-MAP][paper-FAB-MAP].

3. What is the bottleneck of the loop closure algorithm?
<br> The extraction of features.

4. What is the basis of the new approach in the paper?
<br> Based on [bag of words][] and geometrical check, with several novelties that make it much faster than current approaches.

5. Where does the main speed improvement come from?
<br> The use of a slightly modified version of the [BRIEF descriptor][] with FAST keypoints [10].

6. 

[paper-Bag-of-Words]: http://doriangalvez.com/papers/GalvezTRO12.pdf
[paper-comparison-loop-closing-tech]: http://webdiis.unizar.es/GRPTR/pubs/2008_Williams_RSS_IDA.pdf
[paper-FAB-MAP]: http://www.robots.ox.ac.uk/~pnewman/papers/IJRRFabMap.pdf