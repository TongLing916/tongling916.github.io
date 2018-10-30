---
layout:     post
title:      "Review - Bags of Binary Words for Fast Place Recognition in Image Sequences"
date:       2018-10-28
author:     Tong
catalog: true
tags:
	- Review
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

1. We use FAST keypoints and the state-of-the-art BRIEF descriptors. FAST keypoints are corner-like points detected by comparing the gray intensity of some pixels in a Bresenham circle of radius 3. Since only a few pixels are checked, these points are very fast to obtain proving successful for real-time applicaitions.

2. For each FAST keypoint, we draw a square patch around them and compute a BRIEF descriptor. The BRIEF descriptor of an image patch is a binary vector where each bit is the result of an intensity comparison between two of the pixels of the patch. The patches are previously smoothed with a Gaussian kernel to reduce noise. Given beforehand the size of the patch, $$S_b$$, the pairs of pixels to test are randomly selected in an offline stage. In addition to $$S_b$$, we must set the parameter $$L_b$$: the number of tests to perform (i.e. the length of the descriptor).

3. Unlike the original BRIEF descriptor, we found that using close test pairs yielded better results (see this [paper][paper-close-test-paris]). 

4. The main advantage of BRIEF descriptors is that they are very fast to compute and to compare. Since one of these descriptors is just a vector of bits, measuring the distance between two vectors can be by counting the amount of different bits between them (Hamming distance), which is implemented with an _xor_ operation. This is more suitable in this case than calculating the Euclidean distance, as usually done with SIFT or SURF descriptors, composed of floating point values.

## 4. Image database

1. In order to detect revisited places we use an image database composed of a hierarchical [bag of words][website-bag-of-words] and direct and inverse indexes.

2. The bag of words is a technique that uses a visual vocabulary to convert an image into a sparse numerical vector, allowing to manage big sets of images.

3. The visual vocabulary is created offline by discretizing the descriptor space into $$W$$ visual words. Unlike with other features like SIFT or SURF, we discretize a binary descriptor space, creating a more compact vocabulary.

4. In the case of the hierarchical bag of words, the vocabulary is structured as a tree. To build it,  we extract a rich set of features from some training images, independently of those processed online later. The descriptors extracted are first discretized into $$k_w$$ binary clusters by performing [k-medians][website-k-medians] clustering with the [k-means++ seeding][website-k-means++]. 

5. To convert an image $$I_t$$, taken at time _t_, into a bag-of-words vector $$v_t\in \mathbb{R}^W$$, the binary descriptors of its features traverse the tree from the root to the leaves, by selecting at each level the intermediate nodes that minimize the Hamming distance.

6. To measure the similarity between two bag-of-words vectors $$v_1$$ and $$v_2$$, we calculate a $$L_1$$-score $$s(v_1,v_2)$$, whose value lies in [0..1].

7. Along with the bag of words, an inverse index is maintained. This structure stores for each word $$w_i$$ in the vocabulary a list of images $$I_t$$ where it is present. We augment the inverse index to store pairs $$<I_t,v_t^i>$$ to quickly access the weight of the word in the image. The inverse index is updated when a new image $$I_t$$ is added to the database, and accessed when the database is searched for some image.

8. We also make use of a direct index to conveniently store the features of each image.

9. We tanke advantage of the direct index and the bag-of-words tree to use them as a means to approximate nearest neighbors in the BRIEF descriptor space. The direct index allows to speed up the geometrical verification by computing correspondences only between those features that belong to the same words, or to words with common ancestors at level $$l$$. The direct index is updated when a new image is added to the database, and accessed when a candidate matching is obtained and geometrical check is necessary.


## 5. Loop detection algorithm

1. __Database query.__

2. __Match grouping.__

3. __Temporal consistency.__

4. __Efficient geometrical consistency.__


## 6. Experimental evaluation

## 7. Conlusions

1. Binary features are very effective and extremly efficient in the bag-of-words approach. In particular, our results demonstrate that FAST+BRIEF features are as reliable as SURF for solving the loop detection problem with in-plane camera motion, the usual case in mobile robots.

2. The reliability and efficiency of our proposal have been shown on five very different public datasets depicting indoor, outdoor, static and dynamic environments, with frontal or lateral cameras.

3. The main limitation of our technique is the use of features that lack rotation and scale invariance. It is enough for place recognition in indoor and urban robots, but surely not for all-terrain or aerial vehicles, humanoid robots, wearable cameras, or object recognition.

[paper-Bag-of-Words]: http://doriangalvez.com/papers/GalvezTRO12.pdf
[paper-comparison-loop-closing-tech]: http://webdiis.unizar.es/GRPTR/pubs/2008_Williams_RSS_IDA.pdf
[paper-FAB-MAP]: http://www.robots.ox.ac.uk/~pnewman/papers/IJRRFabMap.pdf
[website-bag-of-words]: http://lingtong.de/2018/10/26/Bag-of-Words/
[website-FAST]: http://lingtong.de/2018/10/27/Keypoint-and-Descriptor/
[website-BRIEF]: http://lingtong.de/2018/10/27/Keypoint-and-Descriptor/
[paper-loop-detection]: http://perso.ensta-paristech.fr/~filliat/papers/Angeli_IEEETRO2008.pdf
[paper-close-test-paris]: https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=6094885
[website-k-medians]: http://lingtong.de/2018/10/26/K-Medians-Clustering/
[website-k-means++]: http://lingtong.de/2018/10/26/K-Means-Clustering/





