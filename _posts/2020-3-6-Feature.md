---
layout:     post
title:      "Feature"
date:       2020-3-6
author:     Tong
catalog: true
tags:
    - Technique
---

> https://www.vlfeat.org/

### A combined corner and edge detector [^Harris88]

#### Abstract

Consistency of image edge filtering is of prime importance for 3D interpretation of image sequences using feature tracking algorithms. To cater for image regions containing texture and isolated features, a combined corner and edge detector based on the local auto-correlation function is utilised, and it is shown to perform with good consistency on natural imagery.

#### Corness function

$$
R = \lambda_1 \lambda_2 - k(\lambda_1 + \lambda_2)^2 = det(M) - k\text{trace}^2 (M)
$$

#### Algorithm

1. Compute derivatives in $$x$$ and $$y$$ directions ($$I_x$$, $$I_y$$) e.g. with Sobel filter
2. Compute $$I_x^2$$, $$I_y^2$$, $$I_x I_y$$
3. Convolve  $$I_x^2$$, $$I_y^2$$, $$I_x I_y$$ with a box filter to get  $$\sum I_x^2$$, $$\sum I_y^2$$, $$\sum I_x I_y$$, which are the entries of the matrix $$M$$ (optionally use a Gaussian filter instead of a box filter to avoid aliasing and give more “weight” to the central pixels)
4. Compute Harris Corner Measure $$R$$ (according to Shi Tomasi or Harris)
5. Find points with large corner response ($$R$$ >threshold)
6. Take the points of local maxima of $$R$$

#### [Implementation](http://rpg.ifi.uzh.ch/teaching.html)

```matlab
function scores = harris(img, patch_size, kappa)

sobel_para = [-1 0 1];
sobel_orth = [1 2 1];

Ix = conv2(sobel_orth', sobel_para, img, 'valid');
Iy = conv2(sobel_para', sobel_orth, img, 'valid');
Ixx = double(Ix .^ 2);
Iyy = double(Iy .^ 2);
Ixy = double(Ix .* Iy);

patch = ones(patch_size, patch_size);
pr = floor(patch_size / 2);  % patch radius
sIxx = conv2(Ixx, patch, 'valid');
sIyy = conv2(Iyy, patch, 'valid');
sIxy = conv2(Ixy, patch, 'valid');

scores = (sIxx .* sIyy - sIxy .^ 2) ... determinant
    - kappa * (sIxx + sIyy) .^ 2;  % square trace

scores(scores<0) = 0;

scores = padarray(scores, [1+pr 1+pr]);

end
```

#### [Non-maximum suppression](http://rpg.ifi.uzh.ch/teaching.html)

```matlab
function keypoints = selectKeypoints(scores, num, r)
% Selects the num best scores as keypoints and performs non-maximum 
% supression of a (2r + 1)*(2r + 1) box around the current maximum.

keypoints = zeros(2, num);
temp_scores = padarray(scores, [r r]);
for i = 1:num
    [~, kp] = max(temp_scores(:));
    [row, col] = ind2sub(size(temp_scores), kp);
    kp = [row;col];
    keypoints(:, i) = kp - r;
    temp_scores(kp(1)-r:kp(1)+r, kp(2)-r:kp(2)+r) = ...
        zeros(2*r + 1, 2*r + 1);
end

end
```

### Good Features to Track [^Shi94]

#### Abstract

No feature-based vision system can work unless good features can be identified and tracked from frame to frame. Although tracking itself is by and large a solved problem, selecting features that can be tracked well and correspond to physical points in the world is still hard. We propose a feature selection criterion that is optimal by construction because it is based on how the tracker works, and a feature monitoring method that can detect occlusions, disocclusions, and features that do not correspond to points in the world, These methods are based on a new tracking algorithm that extends previous Newton-Raphson style search methods to work under affine image transformations. We test performance with several simulations and experiments.

#### Cornerness function

A corner can be then be identified by checking whether the minimum of the two eigenvalues of $$M$$ is larger than a certain user-defined threshold.

$$
R = \min(\lambda_1, \lambda_2) > \text{threshold}
$$

#### [Implementation](http://rpg.ifi.uzh.ch/teaching.html)

```matlab
function scores = shi_tomasi(img, patch_size)

sobel_para = [-1 0 1];
sobel_orth = [1 2 1];

Ix = conv2(sobel_orth', sobel_para, img, 'valid');
Iy = conv2(sobel_para', sobel_orth, img, 'valid');
Ixx = double(Ix .^ 2);
Iyy = double(Iy .^ 2);
Ixy = double(Ix .* Iy);

patch = ones(patch_size, patch_size);
pr = floor(patch_size / 2);  % patch radius
sIxx = conv2(Ixx, patch, 'valid');
sIyy = conv2(Iyy, patch, 'valid');
sIxy = conv2(Ixy, patch, 'valid');

trace = sIxx + sIyy;
determinant = sIxx .* sIyy - sIxy .^ 2;

% the eigen values of a matrix M=[a,b;c,d] are 
% lambda1/2 = (Tr(A)/2 +- ((Tr(A)/2)^2-det(A))^.5
% The smaller one is the one with the negative sign
scores = trace/2 - ((trace/2).^2 - determinant).^0.5;
scores(scores<0) = 0;

scores = padarray(scores, [1+pr 1+pr]);

end
```

### Distinctive Image Features from Scale-Invariant Keypoints [^Lowe04]

#### Abstract

This paper presents a method for extracting distinctive invariant features from images that can be used to perform reliable matching between different views of an object or scene. The features are invariant to image scale and rotation, and are shown to provide robust matching across a a substantial range of affine distortion, change in 3D viewpoint, addition of noise, and change in illumination. The features are highly distinctive, in the sense that a single feature can be correctly matched with high probability against a large database of features from many images. This paper also describes an approach to using these features for object recognition. The recognition proceeds by matching individual features to a database of features from known objects using a fast nearest-neighbor algorithm, followed by a Hough transform to identify clusters belonging to a single object, and finally performing verification through least-squares solution for consistent pose parameters. This approach to recognition can robustly identify objects among clutter and occlusion while achieving near real-time performance.

### SURF: Speeded Up Robust Features [^Bay06]

#### Abstract

In this paper, we present a novel scale- and rotation-invariant interest point detector and descriptor, coined SURF (Speeded Up Robust Features). It approximates or even outperforms previously proposed schemes with respect to repeatability, distinctiveness, and robustness, yet can be computed and compared much faster. 

This is achieved by relying on integral images for image convolutions; by building on the strengths of the leading existing detectors and descriptors (in casu, using a Hessian matrix-based measure for the detector, and a distribution-based descriptor); and by simplifying these methods to the essential. This leads to a combination of novel detection, description, and matching steps. The paper presents experimental results on a standard evaluation set, as well as on imagery obtained in the context of a real-life object recognition application. Both show SURF’s strong performance.

#### Introduction

- Based on ideas similar to __SIFT__
- Approximated computation for detection and descriptor (using box filter instead of original second order partial derivatives of a Gaussian)
- Results comparable with SIFT, plus:
  - Faster computation
  - Generally shorter descriptors


### Fusing points and lines for high performance tracking [^Rosten05]

#### Abstract

This paper addresses the problem of real-time 3D model-based tracking by combining point-based and edge-based tracking systems. We present a careful analysis of the properties of these two sensor systems and show that this leads to some non-trivial design choices that collectively yield extremely high performance.  In particular, we present amethod for integrating the two systems and robustly combining the pose estimates they produce. Further we show how online learning can be used to improve the performance of feature tracking. Finally, to aid real-time performance, we introduce the FAST feature detector which can perform full-frame feature detection at 400Hz. The combination of these techniques results in a system which is capable of tracking average prediction errors of 200 pixels. This level of robustness allows us to track very rapid motions,such as 50◦camera shake at 6Hz.

#### Introduction

- __FAST__: Features from Accelrated Segment Test
- Studies intensity of pixels on circle around candidate pixel C
- C is a fast corner __if__ a set of __N__ contiguous pixels on circle are:
  - all brighter than __intensity of C + threshold__, or
  - all darker than __intensity of C + threshold__
- Typically tests for __9__ contiguous pixels on a __16__-pixel circumference
- __Very fas detector__ - in the order of 100 Mega-pixel/second

### BRIEF: Binary Robust Independent Elementary Features [^Calonder10]

#### Abstract

We propose to use binary strings as an efficient feature point descriptor, which we call BRIEF.We show that it is highly discriminative even when using relatively few bits and can be computed using simple intensity difference tests. Furthermore, the descriptor similarity can be evaluated using the Hamming distance, which is very efficient to compute, instead of the $$L_2$$ norm as is usually done.

As a result, BRIEF is very fast both to build and to match. We compare it against SURF and U-SURF on standard benchmarks and show that it yields a similar or better recognition performance, while running in a fraction of the time required by either.

#### Introduction

- Goal: high speed (in description and matching)
- __Binary__ descriptor formation:
  - Smooth image
  - __for each__ detected keypoints (e.g. FAST),
  - __sample__ 256 intensity pairs $$(p_1^i, p_2^i)$$ (i = 1, ..., 256) within a squared patch around the keypoint
  - Create an empty 256-element descriptor
  - for each $$i^{th}$$ pair
    - __if__ $$I_{p_1^i} < I_{p_2^i}$$, then set $$i^{th}$$ bit of desciptor to $$1$$, __else__ to 0.
- The __pattern is generated randomly__ (or by __machine learning__) only once, the same pattern is used for all patches.
- __Pro:__ binary descriptor allows __very fast__ Hamming distance matching (count of the number of bits that are different in the descriptors matches)
- __Cons__: Not scale/rotation invariant

### BRISK: Binary Robust Invariant Scalable Keypoints [^Leutenegger11]

#### Abstract

Effective and efficient generation of keypoints from an image is a well-studied problem in the literature and forms the basis of numerous Computer Vision applications. Established leaders in the field are the SIFT and SURF algorithms which exhibit great performance under a variety of image transformations, with SURF in particular considered as the most computationally efficient amongst the highperformance methods to date.

In this paper we propose BRISK, a novel method for keypoint detection, description and matching. A comprehensive evaluation on benchmark datasets reveals BRISK’s adaptive, high quality performance as in state-of-the-art algorithms, albeit at a dramatically lower computational cost (an order of magnitude faster than SURF in cases). The key to speed lies in the application of a novel scale-space FAST-based detector in combination with the assembly of a bit-string descriptor from intensity comparisons retrieved by dedicated sampling of each keypoint neighborhood.

#### Introduction

- Detect corners in scale-space using FAST
- Rotation and scale invariant
- __Binary__, formed by pairwise intensity comparisons (like BRIEF)
- __Pattern__ defined intensity comparisons in the keypoint neighbors
- Use smoothing kernels with different sizes (longer distance -> larger size)
- Use smoothed pixel values
- Compare short- and long-distance pairs for orientation assignment & descriptor formation
- Detection and descriptor speed: ~10 times faster than SURF
- Slower than BRIEF, but scale- and rotation-invariant

### ORB: an efficient alternative to SIFT or SURF [^Rublee11]

#### Abstract

Feature matching is at the base of many computer vision problems, such as object recognition or structure from motion.  Current methods rely on costly descriptors for detection and matching. In this paper, we propose a very fast binary descriptor based on BRIEF, called ORB, which is rotation invariant and resistant to noise. We demonstrate through experiments how ORB is at two orders of magnitude faster than SIFT, while performing as well in many situations. The efficiency is tested on several real-world applications, including object detection and patch-tracking on a smart phone.

#### Introduction

- Oriented FAST and Rotated BRIEF
- Keypoint detector based on __FAST__
- __BRIEF__ descriptors are _steered_ according to keypoint orientation (to provide rotation invariance)
- Good binary features are learned by minimizing the correlation on a set of trainng patches. 

#### Oriented FAST

```c++

```

#### Rotated BRIEF

#### [Implementation](https://github.com/gaoxiang12/slambook2/blob/master/ch7/orb_self.cpp)

```c++
#include <cassert>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
using namespace std;

string first_file = "./1.png";
string second_file = "./2.png";

typedef vector<uint32_t> DescType;

// ORB pattern
int ORB_pattern[256 * 4] = {
    8,   -3,  9,   5 /*mean (0), correlation (0)*/,
    4,   2,   7,   -12 /*mean (1.12461e-05), correlation (0.0437584)*/,
    -11, 9,   -8,  2 /*mean (3.37382e-05), correlation (0.0617409)*/,
    7,   -12, 12,  -13 /*mean (5.62303e-05), correlation (0.0636977)*/,
    2,   -13, 2,   12 /*mean (0.000134953), correlation (0.085099)*/,
    1,   -7,  1,   6 /*mean (0.000528565), correlation (0.0857175)*/,
    -2,  -10, -2,  -4 /*mean (0.0188821), correlation (0.0985774)*/,
    -13, -13, -11, -8 /*mean (0.0363135), correlation (0.0899616)*/,
    -13, -3,  -12, -9 /*mean (0.121806), correlation (0.099849)*/,
    10,  4,   11,  9 /*mean (0.122065), correlation (0.093285)*/,
    -13, -8,  -8,  -9 /*mean (0.162787), correlation (0.0942748)*/,
    -11, 7,   -9,  12 /*mean (0.21561), correlation (0.0974438)*/,
    7,   7,   12,  6 /*mean (0.160583), correlation (0.130064)*/,
    -4,  -5,  -3,  0 /*mean (0.228171), correlation (0.132998)*/,
    -13, 2,   -12, -3 /*mean (0.00997526), correlation (0.145926)*/,
    -9,  0,   -7,  5 /*mean (0.198234), correlation (0.143636)*/,
    12,  -6,  12,  -1 /*mean (0.0676226), correlation (0.16689)*/,
    -3,  6,   -2,  12 /*mean (0.166847), correlation (0.171682)*/,
    -6,  -13, -4,  -8 /*mean (0.101215), correlation (0.179716)*/,
    11,  -13, 12,  -8 /*mean (0.200641), correlation (0.192279)*/,
    4,   7,   5,   1 /*mean (0.205106), correlation (0.186848)*/,
    5,   -3,  10,  -3 /*mean (0.234908), correlation (0.192319)*/,
    3,   -7,  6,   12 /*mean (0.0709964), correlation (0.210872)*/,
    -8,  -7,  -6,  -2 /*mean (0.0939834), correlation (0.212589)*/,
    -2,  11,  -1,  -10 /*mean (0.127778), correlation (0.20866)*/,
    -13, 12,  -8,  10 /*mean (0.14783), correlation (0.206356)*/,
    -7,  3,   -5,  -3 /*mean (0.182141), correlation (0.198942)*/,
    -4,  2,   -3,  7 /*mean (0.188237), correlation (0.21384)*/,
    -10, -12, -6,  11 /*mean (0.14865), correlation (0.23571)*/,
    5,   -12, 6,   -7 /*mean (0.222312), correlation (0.23324)*/,
    5,   -6,  7,   -1 /*mean (0.229082), correlation (0.23389)*/,
    1,   0,   4,   -5 /*mean (0.241577), correlation (0.215286)*/,
    9,   11,  11,  -13 /*mean (0.00338507), correlation (0.251373)*/,
    4,   7,   4,   12 /*mean (0.131005), correlation (0.257622)*/,
    2,   -1,  4,   4 /*mean (0.152755), correlation (0.255205)*/,
    -4,  -12, -2,  7 /*mean (0.182771), correlation (0.244867)*/,
    -8,  -5,  -7,  -10 /*mean (0.186898), correlation (0.23901)*/,
    4,   11,  9,   12 /*mean (0.226226), correlation (0.258255)*/,
    0,   -8,  1,   -13 /*mean (0.0897886), correlation (0.274827)*/,
    -13, -2,  -8,  2 /*mean (0.148774), correlation (0.28065)*/,
    -3,  -2,  -2,  3 /*mean (0.153048), correlation (0.283063)*/,
    -6,  9,   -4,  -9 /*mean (0.169523), correlation (0.278248)*/,
    8,   12,  10,  7 /*mean (0.225337), correlation (0.282851)*/,
    0,   9,   1,   3 /*mean (0.226687), correlation (0.278734)*/,
    7,   -5,  11,  -10 /*mean (0.00693882), correlation (0.305161)*/,
    -13, -6,  -11, 0 /*mean (0.0227283), correlation (0.300181)*/,
    10,  7,   12,  1 /*mean (0.125517), correlation (0.31089)*/,
    -6,  -3,  -6,  12 /*mean (0.131748), correlation (0.312779)*/,
    10,  -9,  12,  -4 /*mean (0.144827), correlation (0.292797)*/,
    -13, 8,   -8,  -12 /*mean (0.149202), correlation (0.308918)*/,
    -13, 0,   -8,  -4 /*mean (0.160909), correlation (0.310013)*/,
    3,   3,   7,   8 /*mean (0.177755), correlation (0.309394)*/,
    5,   7,   10,  -7 /*mean (0.212337), correlation (0.310315)*/,
    -1,  7,   1,   -12 /*mean (0.214429), correlation (0.311933)*/,
    3,   -10, 5,   6 /*mean (0.235807), correlation (0.313104)*/,
    2,   -4,  3,   -10 /*mean (0.00494827), correlation (0.344948)*/,
    -13, 0,   -13, 5 /*mean (0.0549145), correlation (0.344675)*/,
    -13, -7,  -12, 12 /*mean (0.103385), correlation (0.342715)*/,
    -13, 3,   -11, 8 /*mean (0.134222), correlation (0.322922)*/,
    -7,  12,  -4,  7 /*mean (0.153284), correlation (0.337061)*/,
    6,   -10, 12,  8 /*mean (0.154881), correlation (0.329257)*/,
    -9,  -1,  -7,  -6 /*mean (0.200967), correlation (0.33312)*/,
    -2,  -5,  0,   12 /*mean (0.201518), correlation (0.340635)*/,
    -12, 5,   -7,  5 /*mean (0.207805), correlation (0.335631)*/,
    3,   -10, 8,   -13 /*mean (0.224438), correlation (0.34504)*/,
    -7,  -7,  -4,  5 /*mean (0.239361), correlation (0.338053)*/,
    -3,  -2,  -1,  -7 /*mean (0.240744), correlation (0.344322)*/,
    2,   9,   5,   -11 /*mean (0.242949), correlation (0.34145)*/,
    -11, -13, -5,  -13 /*mean (0.244028), correlation (0.336861)*/,
    -1,  6,   0,   -1 /*mean (0.247571), correlation (0.343684)*/,
    5,   -3,  5,   2 /*mean (0.000697256), correlation (0.357265)*/,
    -4,  -13, -4,  12 /*mean (0.00213675), correlation (0.373827)*/,
    -9,  -6,  -9,  6 /*mean (0.0126856), correlation (0.373938)*/,
    -12, -10, -8,  -4 /*mean (0.0152497), correlation (0.364237)*/,
    10,  2,   12,  -3 /*mean (0.0299933), correlation (0.345292)*/,
    7,   12,  12,  12 /*mean (0.0307242), correlation (0.366299)*/,
    -7,  -13, -6,  5 /*mean (0.0534975), correlation (0.368357)*/,
    -4,  9,   -3,  4 /*mean (0.099865), correlation (0.372276)*/,
    7,   -1,  12,  2 /*mean (0.117083), correlation (0.364529)*/,
    -7,  6,   -5,  1 /*mean (0.126125), correlation (0.369606)*/,
    -13, 11,  -12, 5 /*mean (0.130364), correlation (0.358502)*/,
    -3,  7,   -2,  -6 /*mean (0.131691), correlation (0.375531)*/,
    7,   -8,  12,  -7 /*mean (0.160166), correlation (0.379508)*/,
    -13, -7,  -11, -12 /*mean (0.167848), correlation (0.353343)*/,
    1,   -3,  12,  12 /*mean (0.183378), correlation (0.371916)*/,
    2,   -6,  3,   0 /*mean (0.228711), correlation (0.371761)*/,
    -4,  3,   -2,  -13 /*mean (0.247211), correlation (0.364063)*/,
    -1,  -13, 1,   9 /*mean (0.249325), correlation (0.378139)*/,
    7,   1,   8,   -6 /*mean (0.000652272), correlation (0.411682)*/,
    1,   -1,  3,   12 /*mean (0.00248538), correlation (0.392988)*/,
    9,   1,   12,  6 /*mean (0.0206815), correlation (0.386106)*/,
    -1,  -9,  -1,  3 /*mean (0.0364485), correlation (0.410752)*/,
    -13, -13, -10, 5 /*mean (0.0376068), correlation (0.398374)*/,
    7,   7,   10,  12 /*mean (0.0424202), correlation (0.405663)*/,
    12,  -5,  12,  9 /*mean (0.0942645), correlation (0.410422)*/,
    6,   3,   7,   11 /*mean (0.1074), correlation (0.413224)*/,
    5,   -13, 6,   10 /*mean (0.109256), correlation (0.408646)*/,
    2,   -12, 2,   3 /*mean (0.131691), correlation (0.416076)*/,
    3,   8,   4,   -6 /*mean (0.165081), correlation (0.417569)*/,
    2,   6,   12,  -13 /*mean (0.171874), correlation (0.408471)*/,
    9,   -12, 10,  3 /*mean (0.175146), correlation (0.41296)*/,
    -8,  4,   -7,  9 /*mean (0.183682), correlation (0.402956)*/,
    -11, 12,  -4,  -6 /*mean (0.184672), correlation (0.416125)*/,
    1,   12,  2,   -8 /*mean (0.191487), correlation (0.386696)*/,
    6,   -9,  7,   -4 /*mean (0.192668), correlation (0.394771)*/,
    2,   3,   3,   -2 /*mean (0.200157), correlation (0.408303)*/,
    6,   3,   11,  0 /*mean (0.204588), correlation (0.411762)*/,
    3,   -3,  8,   -8 /*mean (0.205904), correlation (0.416294)*/,
    7,   8,   9,   3 /*mean (0.213237), correlation (0.409306)*/,
    -11, -5,  -6,  -4 /*mean (0.243444), correlation (0.395069)*/,
    -10, 11,  -5,  10 /*mean (0.247672), correlation (0.413392)*/,
    -5,  -8,  -3,  12 /*mean (0.24774), correlation (0.411416)*/,
    -10, 5,   -9,  0 /*mean (0.00213675), correlation (0.454003)*/,
    8,   -1,  12,  -6 /*mean (0.0293635), correlation (0.455368)*/,
    4,   -6,  6,   -11 /*mean (0.0404971), correlation (0.457393)*/,
    -10, 12,  -8,  7 /*mean (0.0481107), correlation (0.448364)*/,
    4,   -2,  6,   7 /*mean (0.050641), correlation (0.455019)*/,
    -2,  0,   -2,  12 /*mean (0.0525978), correlation (0.44338)*/,
    -5,  -8,  -5,  2 /*mean (0.0629667), correlation (0.457096)*/,
    7,   -6,  10,  12 /*mean (0.0653846), correlation (0.445623)*/,
    -9,  -13, -8,  -8 /*mean (0.0858749), correlation (0.449789)*/,
    -5,  -13, -5,  -2 /*mean (0.122402), correlation (0.450201)*/,
    8,   -8,  9,   -13 /*mean (0.125416), correlation (0.453224)*/,
    -9,  -11, -9,  0 /*mean (0.130128), correlation (0.458724)*/,
    1,   -8,  1,   -2 /*mean (0.132467), correlation (0.440133)*/,
    7,   -4,  9,   1 /*mean (0.132692), correlation (0.454)*/,
    -2,  1,   -1,  -4 /*mean (0.135695), correlation (0.455739)*/,
    11,  -6,  12,  -11 /*mean (0.142904), correlation (0.446114)*/,
    -12, -9,  -6,  4 /*mean (0.146165), correlation (0.451473)*/,
    3,   7,   7,   12 /*mean (0.147627), correlation (0.456643)*/,
    5,   5,   10,  8 /*mean (0.152901), correlation (0.455036)*/,
    0,   -4,  2,   8 /*mean (0.167083), correlation (0.459315)*/,
    -9,  12,  -5,  -13 /*mean (0.173234), correlation (0.454706)*/,
    0,   7,   2,   12 /*mean (0.18312), correlation (0.433855)*/,
    -1,  2,   1,   7 /*mean (0.185504), correlation (0.443838)*/,
    5,   11,  7,   -9 /*mean (0.185706), correlation (0.451123)*/,
    3,   5,   6,   -8 /*mean (0.188968), correlation (0.455808)*/,
    -13, -4,  -8,  9 /*mean (0.191667), correlation (0.459128)*/,
    -5,  9,   -3,  -3 /*mean (0.193196), correlation (0.458364)*/,
    -4,  -7,  -3,  -12 /*mean (0.196536), correlation (0.455782)*/,
    6,   5,   8,   0 /*mean (0.1972), correlation (0.450481)*/,
    -7,  6,   -6,  12 /*mean (0.199438), correlation (0.458156)*/,
    -13, 6,   -5,  -2 /*mean (0.211224), correlation (0.449548)*/,
    1,   -10, 3,   10 /*mean (0.211718), correlation (0.440606)*/,
    4,   1,   8,   -4 /*mean (0.213034), correlation (0.443177)*/,
    -2,  -2,  2,   -13 /*mean (0.234334), correlation (0.455304)*/,
    2,   -12, 12,  12 /*mean (0.235684), correlation (0.443436)*/,
    -2,  -13, 0,   -6 /*mean (0.237674), correlation (0.452525)*/,
    4,   1,   9,   3 /*mean (0.23962), correlation (0.444824)*/,
    -6,  -10, -3,  -5 /*mean (0.248459), correlation (0.439621)*/,
    -3,  -13, -1,  1 /*mean (0.249505), correlation (0.456666)*/,
    7,   5,   12,  -11 /*mean (0.00119208), correlation (0.495466)*/,
    4,   -2,  5,   -7 /*mean (0.00372245), correlation (0.484214)*/,
    -13, 9,   -9,  -5 /*mean (0.00741116), correlation (0.499854)*/,
    7,   1,   8,   6 /*mean (0.0208952), correlation (0.499773)*/,
    7,   -8,  7,   6 /*mean (0.0220085), correlation (0.501609)*/,
    -7,  -4,  -7,  1 /*mean (0.0233806), correlation (0.496568)*/,
    -8,  11,  -7,  -8 /*mean (0.0236505), correlation (0.489719)*/,
    -13, 6,   -12, -8 /*mean (0.0268781), correlation (0.503487)*/,
    2,   4,   3,   9 /*mean (0.0323324), correlation (0.501938)*/,
    10,  -5,  12,  3 /*mean (0.0399235), correlation (0.494029)*/,
    -6,  -5,  -6,  7 /*mean (0.0420153), correlation (0.486579)*/,
    8,   -3,  9,   -8 /*mean (0.0548021), correlation (0.484237)*/,
    2,   -12, 2,   8 /*mean (0.0616622), correlation (0.496642)*/,
    -11, -2,  -10, 3 /*mean (0.0627755), correlation (0.498563)*/,
    -12, -13, -7,  -9 /*mean (0.0829622), correlation (0.495491)*/,
    -11, 0,   -10, -5 /*mean (0.0843342), correlation (0.487146)*/,
    5,   -3,  11,  8 /*mean (0.0929937), correlation (0.502315)*/,
    -2,  -13, -1,  12 /*mean (0.113327), correlation (0.48941)*/,
    -1,  -8,  0,   9 /*mean (0.132119), correlation (0.467268)*/,
    -13, -11, -12, -5 /*mean (0.136269), correlation (0.498771)*/,
    -10, -2,  -10, 11 /*mean (0.142173), correlation (0.498714)*/,
    -3,  9,   -2,  -13 /*mean (0.144141), correlation (0.491973)*/,
    2,   -3,  3,   2 /*mean (0.14892), correlation (0.500782)*/,
    -9,  -13, -4,  0 /*mean (0.150371), correlation (0.498211)*/,
    -4,  6,   -3,  -10 /*mean (0.152159), correlation (0.495547)*/,
    -4,  12,  -2,  -7 /*mean (0.156152), correlation (0.496925)*/,
    -6,  -11, -4,  9 /*mean (0.15749), correlation (0.499222)*/,
    6,   -3,  6,   11 /*mean (0.159211), correlation (0.503821)*/,
    -13, 11,  -5,  5 /*mean (0.162427), correlation (0.501907)*/,
    11,  11,  12,  6 /*mean (0.16652), correlation (0.497632)*/,
    7,   -5,  12,  -2 /*mean (0.169141), correlation (0.484474)*/,
    -1,  12,  0,   7 /*mean (0.169456), correlation (0.495339)*/,
    -4,  -8,  -3,  -2 /*mean (0.171457), correlation (0.487251)*/,
    -7,  1,   -6,  7 /*mean (0.175), correlation (0.500024)*/,
    -13, -12, -8,  -13 /*mean (0.175866), correlation (0.497523)*/,
    -7,  -2,  -6,  -8 /*mean (0.178273), correlation (0.501854)*/,
    -8,  5,   -6,  -9 /*mean (0.181107), correlation (0.494888)*/,
    -5,  -1,  -4,  5 /*mean (0.190227), correlation (0.482557)*/,
    -13, 7,   -8,  10 /*mean (0.196739), correlation (0.496503)*/,
    1,   5,   5,   -13 /*mean (0.19973), correlation (0.499759)*/,
    1,   0,   10,  -13 /*mean (0.204465), correlation (0.49873)*/,
    9,   12,  10,  -1 /*mean (0.209334), correlation (0.49063)*/,
    5,   -8,  10,  -9 /*mean (0.211134), correlation (0.503011)*/,
    -1,  11,  1,   -13 /*mean (0.212), correlation (0.499414)*/,
    -9,  -3,  -6,  2 /*mean (0.212168), correlation (0.480739)*/,
    -1,  -10, 1,   12 /*mean (0.212731), correlation (0.502523)*/,
    -13, 1,   -8,  -10 /*mean (0.21327), correlation (0.489786)*/,
    8,   -11, 10,  -6 /*mean (0.214159), correlation (0.488246)*/,
    2,   -13, 3,   -6 /*mean (0.216993), correlation (0.50287)*/,
    7,   -13, 12,  -9 /*mean (0.223639), correlation (0.470502)*/,
    -10, -10, -5,  -7 /*mean (0.224089), correlation (0.500852)*/,
    -10, -8,  -8,  -13 /*mean (0.228666), correlation (0.502629)*/,
    4,   -6,  8,   5 /*mean (0.22906), correlation (0.498305)*/,
    3,   12,  8,   -13 /*mean (0.233378), correlation (0.503825)*/,
    -4,  2,   -3,  -3 /*mean (0.234323), correlation (0.476692)*/,
    5,   -13, 10,  -12 /*mean (0.236392), correlation (0.475462)*/,
    4,   -13, 5,   -1 /*mean (0.236842), correlation (0.504132)*/,
    -9,  9,   -4,  3 /*mean (0.236977), correlation (0.497739)*/,
    0,   3,   3,   -9 /*mean (0.24314), correlation (0.499398)*/,
    -12, 1,   -6,  1 /*mean (0.243297), correlation (0.489447)*/,
    3,   2,   4,   -8 /*mean (0.00155196), correlation (0.553496)*/,
    -10, -10, -10, 9 /*mean (0.00239541), correlation (0.54297)*/,
    8,   -13, 12,  12 /*mean (0.0034413), correlation (0.544361)*/,
    -8,  -12, -6,  -5 /*mean (0.003565), correlation (0.551225)*/,
    2,   2,   3,   7 /*mean (0.00835583), correlation (0.55285)*/,
    10,  6,   11,  -8 /*mean (0.00885065), correlation (0.540913)*/,
    6,   8,   8,   -12 /*mean (0.0101552), correlation (0.551085)*/,
    -7,  10,  -6,  5 /*mean (0.0102227), correlation (0.533635)*/,
    -3,  -9,  -3,  9 /*mean (0.0110211), correlation (0.543121)*/,
    -1,  -13, -1,  5 /*mean (0.0113473), correlation (0.550173)*/,
    -3,  -7,  -3,  4 /*mean (0.0140913), correlation (0.554774)*/,
    -8,  -2,  -8,  3 /*mean (0.017049), correlation (0.55461)*/,
    4,   2,   12,  12 /*mean (0.01778), correlation (0.546921)*/,
    2,   -5,  3,   11 /*mean (0.0224022), correlation (0.549667)*/,
    6,   -9,  11,  -13 /*mean (0.029161), correlation (0.546295)*/,
    3,   -1,  7,   12 /*mean (0.0303081), correlation (0.548599)*/,
    11,  -1,  12,  4 /*mean (0.0355151), correlation (0.523943)*/,
    -3,  0,   -3,  6 /*mean (0.0417904), correlation (0.543395)*/,
    4,   -11, 4,   12 /*mean (0.0487292), correlation (0.542818)*/,
    2,   -4,  2,   1 /*mean (0.0575124), correlation (0.554888)*/,
    -10, -6,  -8,  1 /*mean (0.0594242), correlation (0.544026)*/,
    -13, 7,   -11, 1 /*mean (0.0597391), correlation (0.550524)*/,
    -13, 12,  -11, -13 /*mean (0.0608974), correlation (0.55383)*/,
    6,   0,   11,  -13 /*mean (0.065126), correlation (0.552006)*/,
    0,   -1,  1,   4 /*mean (0.074224), correlation (0.546372)*/,
    -13, 3,   -9,  -2 /*mean (0.0808592), correlation (0.554875)*/,
    -9,  8,   -6,  -3 /*mean (0.0883378), correlation (0.551178)*/,
    -13, -6,  -8,  -2 /*mean (0.0901035), correlation (0.548446)*/,
    5,   -9,  8,   10 /*mean (0.0949843), correlation (0.554694)*/,
    2,   7,   3,   -9 /*mean (0.0994152), correlation (0.550979)*/,
    -1,  -6,  -1,  -1 /*mean (0.10045), correlation (0.552714)*/,
    9,   5,   11,  -2 /*mean (0.100686), correlation (0.552594)*/,
    11,  -3,  12,  -8 /*mean (0.101091), correlation (0.532394)*/,
    3,   0,   3,   5 /*mean (0.101147), correlation (0.525576)*/,
    -1,  4,   0,   10 /*mean (0.105263), correlation (0.531498)*/,
    3,   -6,  4,   5 /*mean (0.110785), correlation (0.540491)*/,
    -13, 0,   -10, 5 /*mean (0.112798), correlation (0.536582)*/,
    5,   8,   12,  11 /*mean (0.114181), correlation (0.555793)*/,
    8,   9,   9,   -6 /*mean (0.117431), correlation (0.553763)*/,
    7,   -4,  8,   -12 /*mean (0.118522), correlation (0.553452)*/,
    -10, 4,   -10, 9 /*mean (0.12094), correlation (0.554785)*/,
    7,   3,   12,  4 /*mean (0.122582), correlation (0.555825)*/,
    9,   -7,  10,  -2 /*mean (0.124978), correlation (0.549846)*/,
    7,   0,   12,  -2 /*mean (0.127002), correlation (0.537452)*/,
    -1,  -6,  0,   -11 /*mean (0.127148), correlation (0.547401)*/
};

void ComputeORB(const cv::Mat& img, vector<cv::KeyPoint>& keypoints,
                vector<DescType>& descriptors) {
  const int half_patch_size = 8;
  const int half_boundary = 16;
  int bad_points = 0;
  for (auto& kp : keypoints) {
    if (kp.pt.x < half_boundary || kp.pt.y < half_boundary ||
        kp.pt.x >= img.cols - half_boundary ||
        kp.pt.y >= img.rows - half_boundary) {
      // outside
      ++bad_points;
      descriptors.push_back({});
      continue;
    }

    float m01 = 0, m10 = 0;
    for (int dx = -half_patch_size; dx < half_patch_size; ++dx)
      for (int dy = -half_patch_size; dy < half_patch_size; ++dy) {
        uchar pixel = img.at<uchar>(kp.pt.y + dy, kp.pt.x + dx);
        m01 += dx * pixel;
        m10 += dy * pixel;
      }

    float m_sqrt = sqrt(m01 * m01 + m10 * m10) + 1e-18;
    float sin_theta = m01 / m_sqrt;
    float cos_theta = m10 / m_sqrt;

    DescType desc(8, 0);
    for (int i = 0; i < 8; ++i) {
      uint32_t d = 0;
      for (int k = 0; k < 32; ++k) {
        int idx_pq = i * 8 + k;
        cv::Point2f p(ORB_pattern[idx_pq * 4], ORB_pattern[idx_pq * 4 + 1]);
        cv::Point2f q(ORB_pattern[idx_pq * 4 + 2], ORB_pattern[idx_pq * 4 + 3]);

        cv::Point2f pp = cv::Point2f(cos_theta * p.x - sin_theta * p.y,
                                     sin_theta * p.x + cos_theta * p.y) +
                         kp.pt;
        cv::Point2f qq = cv::Point2f(cos_theta * q.x - sin_theta * q.y,
                                     sin_theta * q.x + cos_theta * q.y) +
                         kp.pt;
        if (img.at<uchar>(pp.y, pp.x) < img.at<uchar>(qq.y, qq.x)) d |= 1 << k;
      }
      desc[i] = d;
    }
    descriptors.push_back(desc);
  }

  cout << "bad/total: " << bad_points << " / " << keypoints.size() << endl;
}

void BfMatch(const vector<DescType>& desc1, const vector<DescType>& desc2,
             vector<cv::DMatch>& matches) {
  const int d_max = 40;

  for (int i1 = 0; i1 < desc1.size(); ++i1) {
    if (desc1[i1].empty()) continue;

    cv::DMatch m{i1, 0, 256};
    for (int i2 = 0; i2 < desc2.size(); ++i2) {
      if (desc2[i2].empty()) continue;

      int distance = 0;
      for (int k = 0; k < 8; ++k)
        distance += _mm_popcnt_u32(
            desc1[i1][k] ^
            desc2[i2][k]);  // int _mm_popcnt_u32 (unsigned int a): Count the
                            // number of bits set to 1 in unsigned 32-bit
                            // integer a, and return that count.
      if (distance < d_max && distance < m.distance) {
        m.distance = distance;
        m.trainIdx = i2;
      }
    }
    if (m.distance < d_max) matches.push_back(m);
  }
}

int main(int arg, char** argv) {
  cv::Mat first_image = cv::imread(first_file, 0);
  cv::Mat second_image = cv::imread(second_file, 0);
  assert(first_image.data != nullptr && second_image.data != nullptr);

  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

  vector<cv::KeyPoint> keypoints1;
  cv::FAST(first_image, keypoints1, 40);
  vector<DescType> descriptors1;
  ComputeORB(first_image, keypoints1, descriptors1);

  vector<cv::KeyPoint> keypoints2;
  cv::FAST(second_image, keypoints2, 40);
  vector<DescType> descriptors2;
  ComputeORB(second_image, keypoints2, descriptors2);

  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<float> time_used =
      chrono::duration_cast<chrono::duration<float>>(t2 - t1);
  cout << "extract ORB cost = " << time_used.count() << " second. " << endl;

  vector<cv::DMatch> matches;
  t1 = chrono::steady_clock::now();
  BfMatch(descriptors1, descriptors2, matches);
  t2 = chrono::steady_clock::now();
  time_used = chrono::duration_cast<chrono::duration<float>>(t2 - t1);
  cout << "match ORB cost = " << time_used.count() << " seconds." << endl;
  cout << "matches: " << matches.size() << endl;

  cv::Mat image_show;
  cv::drawMatches(first_image, keypoints1, second_image, keypoints2, matches,
                  image_show);
  cv::imshow("matches", image_show);
  cv::waitKey(0);

  return 0;
}
```


### FREAK: Fast Retina Keypoint [^Alahi12]

#### Abstract

A large number of vision applications rely on matching keypoints across images. The last decade featured an arms-race towards faster and more robust keypoints and association algorithms: Scale Invariant Feature Transform (SIFT)[^Lowe04], Speed-up Robust Feature (SURF)[^Bay06], and more recently Binary Robust Invariant Scalable Keypoints (BRISK)[^Leutenegger11] to name a few. These days, the deployment of vision algorithms on smart phones and embedded devices with low memory and computation complexity has even upped the ante: the goal is to make descriptors faster to compute, more compact while remaining robust to scale, rotation and noise.

To best address the current requirements, we propose a novel keypoint descriptor inspired by the human visual system and more precisely the retina, coined Fast Retina Keypoint (FREAK). A cascade of binary strings is computed by efficiently comparing image intensities over a retinal sampling pattern. Our experiments show that FREAKs are in general faster to compute with lower memory load and also more robust than SIFT, SURF or BRISK. They are thus competitive alternatives to existing keypoints in particular for embedded applications.

#### Introduction

- Rotation and scale invariant
  - Fast, compact and robust keypoint descriptor
  - __Sampling pattern__ inspired by the human retina: higher density of points near the center.
  - __Pairwise__ intensity comparisons form __binary__ strings.
  - Pairs are __learned__ (as in ORB)
  - Use smoothing kernels with different sizes
  - Orientation mechanism similar to BRISK
  - Coarse-to-fine matching (cascaded approach): first compare the first 128 bits; if distance smaller than threshold, proceed to compare the next bits, etc.
- Faster to compute, less memory and more robust than SIFT, SURF or BRISK

### LIFT: Learned Invariant Feature Transform [^Yi16]

#### Abstract

We introduce a novel Deep Network architecture that implements the full feature point handling pipeline, that is, detection, orientation estimation, and feature description. While previous works have successfully tackled each one of these problems individually, we show how to learn to do all three in a unied manner while preserving end-to-end differentiability. We then demonstrate that our Deep pipeline outperforms state-of-the-art methods on a number of benchmark datasets, without the need of retraining.

#### Introduction

- Rotation, scale, viewpoint and illumination invariant
  - Learning-based method which does both keypoint detection (__in scale space__) and description.
  - First, a network predicts the __patch orientation__ which is used to derotate the patch.
  - Then, another neural network is used to generate a __patch descriptor__ (128 dimensional) from the derotated patch.
  - __Illumination invariance__ is achieved by randomizing illuminations during training.
  - LIFT keypoints showed the best matching score and repeatability. Even better than SIFT.

### LSD: a line segment detector [^Gioi08][^Gioi12]

#### Abstract

We propose a linear-time line segment detector that gives accurate results, a controlled number of false detections, and requires no parameter tuning. This algorithm is tested and compared to state-of-the-art algorithms on a wide set of natural images.

#### Introduction

The aim of this paper is to present a linear-time algorithm that cumulates most of the advantages of the previous algorithms without their drawbacks. The Burns et al.[^Burns86] line segment finder, that made a breakthrough in the extraction of the line segments, will be improved and combined with a validation criterion inspired from Desolneux et al[^Desolneux00][^Desolneux02][^Desolneux03][^Desolneux08]. The result is LSD, a linear-time line segment detector that requires no parameter tuning and gives accurate results.

#### Line-Support Regions

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/region_grow.PNG)

#### Rectangular approximation of regions

#### Line Segment Validation

#### The complete LSD algorithm

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/LSD_line_segment_detector.PNG)

### An efficient and robust line segment matching approach based on LBD descriptor and pairwise geometric consistency [^Zhang13]

#### Abstract

We present a line matching algorithm which utilizes both the local appearance of lines and their geometric attributes. To overcome the problem of segment fragmentation and geometric variation, we extract lines in the scale space. To depict the local appearance of lines, we design a novel line descriptor called _Line Band Descriptor_ (LBD). To evaluate the pairwise geometric consistency, we define the pairwise geometric attributes between line pairs. Then we built a relational graph for candidate line matches and employ a spectral technique to solve this matching problem efficiently. The advantages of the proposed algorithm are as follows:
    1. it’s robust to image transformations because of the multi-scale line detection strategy;
    2. it’s efficient because the designed LBD descriptor is fast to compute and the appearance similarities reduce the dimension of the graph matching problem;
    3. it’s accurate even for low-texture images because of the pairwise geometric consistency evaluation.

### SuperPoint: Self-Supervised Interest Point Detection and Description [^DeTone18]

#### Abstract    

This paper presents a self-supervised framework for training interest point detectors and descriptors suitable for a large number of multiple-view geometry problems in computer vision. As opposed to patch-based neural networks, our fully-convolutional model operates on full-sized images and jointly computes pixel-level interest point locations and associated descriptors in one forward pass. We introduce Homographic Adaptation, a multi-scale, multihomography approach for boosting interest point detection repeatability and performing cross-domain adaptation (e.g., synthetic-to-real). Our model, when trained on the MS-COCO generic image dataset using Homographic Adaptation, is able to repeatedly detect a much richer set of interest points than the initial pre-adapted deep model and any other traditional corner detector. The final system gives rise to state-of-the-art homography estimation results on HPatches when compared to LIFT, SIFT and ORB.

#### Introduction

- Joint regression of keypoint location and descriptors
- Trained on synthetic images and refined on homographies of real images
- Detector less accurate than SIFT, but descriptor shown to outperform SIFT

### Literature

[^Burns86]: J.B. Burns, A.R. Hanson, and E.M. Riseman, “Extracting Straight Lines,” IEEE Trans. Pattern Analysis and Machine Intelligence, vol. 8, no. 4, pp. 425 -455, July 1986.

[^Harris88]: Harris, Christopher G., and Mike Stephens. "A combined corner and edge detector." Alvey vision conference. Vol. 15. No. 50. 1988.

[^Shi94]: Shi, Jianbo. "Good features to track." 1994 Proceedings of IEEE conference on computer vision and pattern recognition. IEEE, 1994.

[^Desolneux00]: A. Desolneux, L. Moisan, and J.M. Morel, “Meaningful Alignments,” Int’l J. Computer Vision, vol. 40, no. 1, pp. 7-23, 2000.

[^Desolneux02]: A. Desolneux, S. Ladjal, L. Moisan, and J.M. Morel, “Dequantizing Image Orientation,” IEEE Trans. Image Processing, vol. 11, no. 10, pp. 1129-1140, Oct. 2002.

[^Desolneux03]: A. Desolneux, L. Moisan, and J.M. Morel, “Computational Gestalts and Perception Thresholds,” J. Physiology-Paris, vol. 97, pp. 311-324, 2003.

[^Lowe04]: Lowe, David G. "Distinctive image features from scale-invariant keypoints." International journal of computer vision 60.2 (2004): 91-110.

[^Rosten05]: Rosten, Drummond, "Fusing points and lines for high performance tracking", International Conference on Computer Vision (ICCV), 2005

[^Bay06]: Bay, Herbert, Tinne Tuytelaars, and Luc Van Gool. "Surf: Speeded up robust features." European conference on computer vision. Springer, Berlin, Heidelberg, 2006.

[^Gioi08]: Von Gioi, Rafael Grompone, et al. "LSD: A fast line segment detector with a false detection control." IEEE transactions on pattern analysis and machine intelligence 32.4 (2008): 722-732.

[^Desolneux08]: A. Desolneux, L. Moisan, and J.M. Morel, From Gestalt Theory to Image Analysis, A Probabilistic Approach. Springer, 2008.

[^Calonder10]: Calonder, Michael, et al. "Brief: Binary robust independent elementary features." European conference on computer vision. Springer, Berlin, Heidelberg, 2010.

[^Leutenegger11]: Leutenegger, Stefan, Margarita Chli, and Roland Y. Siegwart. "BRISK: Binary robust invariant scalable keypoints." 2011 International conference on computer vision. Ieee, 2011.

[^Rublee11]: Rublee, Ethan, et al. "ORB: An efficient alternative to SIFT or SURF." 2011 International conference on computer vision. Ieee, 2011.

[^Alahi12]: Alahi, Alexandre, Raphael Ortiz, and Pierre Vandergheynst. "Freak: Fast retina keypoint." 2012 IEEE Conference on Computer Vision and Pattern Recognition. Ieee, 2012.

[^Gioi12]: Von Gioi, Rafael Grompone, et al. "LSD: a line segment detector." Image Processing On Line 2 (2012): 35-55.

[^Zhang13]: Zhang, Lilian, and Reinhard Koch. "An efficient and robust line segment matching approach based on LBD descriptor and pairwise geometric consistency." Journal of Visual Communication and Image Representation 24.7 (2013): 794-805.

[^Yi16]: Yi, Kwang Moo, et al. "Lift: Learned invariant feature transform." European Conference on Computer Vision. Springer, Cham, 2016.

[^DeTone18]: DeTone, Daniel, Tomasz Malisiewicz, and Andrew Rabinovich. "Superpoint: Self-supervised interest point detection and description." Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition Workshops. 2018.
