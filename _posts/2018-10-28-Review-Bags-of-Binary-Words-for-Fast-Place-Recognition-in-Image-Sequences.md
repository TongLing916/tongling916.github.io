---
layout:     post
title:      "Review - Bags of Binary Words for Fast Place Recognition in Image Sequences"
date:       2018-10-28
author:     Tong
catalog: true
tags:
    - SLAM
---

> <<Bags of Binary Words for Fast Place Recognition in Image Sequences>> 读后感

This paper can be found in this [website][paper-Bag-of-Words].

## 1. Introduction

1. _Why do we need loop closures?_
<br> After an exploratory period, when ares non-observed for long are re-orbserved, standard matching algorithm fail. When they are robustly detected, loop closures provide correct data association. The same methods used for detection can be used for robot relocation after track lost.

2. _What are the methods for loop detection or robot relocation?_
<br> For small environments, map-to-image methods. See the results in this [paper][paper-comparison-loop-closing-tech].
<br> For large environments, image-to-image methods such as [FAB-MAP][paper-FAB-MAP].

3. _What is the bottleneck of the loop closure algorithm?_
<br> The extraction of features.

4. _What is the basis of the new approach in the paper?_
<br> Based on [bag of words][website-bag-of-words] and geometrical check, with several novelties that make it much faster than current approaches.

5. _Where does the main speed improvement come from?_
<br> The use of a slightly modified version of the [BRIEF descriptor][website-BRIEF] with [FAST keypoints][website-FAST].

6. _What is special in bag of words in this paper?_
<br> The bag of words in this paper discretizes a binary space, and augment it with a direct index, in addition to the usual inverse index. The inverse index is used for fast retrieval of images potentially similar to a given one. The direct index to efficiently obtain point correspondences between images, speeding up the geometrical check during the loop verification.

7. _What is special in the complete loop detection algorithm?_
<br> We verify the temporal consistency of the image matches obtained. One of the novelties in this paper is a technique to prevent images collected in the same place from competing among them when the database is queried. We achieve this by grouping together those images that depict the same place during the matching.

## 2. Related work

1. We rely on a temporal consistency check to consider previous matches and enhance the reliability of the detections.

2. We use a bag of binary words for the first time, as well as propose a technique to prevent images collected close in time and depicting the same place from competing between them during the matching, so that we can work at a higher frequency.

3. To verify loop closing candidates, a geometrical check is usually performed. We apply an epipolar constraint to the best matching candidate as in this [paper][paper-loop-detection], but we take advantage of a direct index to calculate correspondence points faster.

4. We reduce the excution time by using efficient features. BRIEF descriptor, along with other recent descriptors as BRISK or ORB, are binary and require very little time to be computed. As an advantage, their information is very compact, so that they occupy less memory and are faster to compare. This allows a much faster conversion into the bag-of-words space.

## 3. Binary features

1. 

## 4. Image database

1. 

## 5. Loop detection algorithm

[paper-Bag-of-Words]: http://doriangalvez.com/papers/GalvezTRO12.pdf
[paper-comparison-loop-closing-tech]: http://webdiis.unizar.es/GRPTR/pubs/2008_Williams_RSS_IDA.pdf
[paper-FAB-MAP]: http://www.robots.ox.ac.uk/~pnewman/papers/IJRRFabMap.pdf
[website-bag-of-words]: http://lingtong.de/2018/10/26/Bag-of-Words/
[website-FAST]: http://lingtong.de/2018/10/27/Keypoint-and-Descriptor/
[website-BRIEF]: http://lingtong.de/2018/10/27/Keypoint-and-Descriptor/
[paper-loop-detection]: http://perso.ensta-paristech.fr/~filliat/papers/Angeli_IEEETRO2008.pdf