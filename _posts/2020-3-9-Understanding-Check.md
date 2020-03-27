---
layout:     post
title:      "Vision Algorithms for Mobile Robotics - Understanding Check"
date:       2020-3-9
author:     Tong
catalog: true
tags:
    - SLAM
---

### 1. Introduction to Computer Vision and Visual Odometry

1. Provide a definition of Visual Odometry?
2. Explain the most important differences between VO, VSLAM and SFM?
3. Describe the needed assumptions for VO?
4. Illustrate its building blocks

### 2. Image Formation 1: perspective projection and camera models

1. Explain what a blur circle is?
2. Derive the thin lens equation and perform the pinhole approximation?
3. Define vanishing points and lines?
4. Prove that parallel lines intersect at vanishing points?
5. Explain how to build an Ames room?
6. Derive a relation between the field of view and the focal length?
7. Explain the perspective projection equation, including lens distortion and world to camera projection?

### 3. Image Formation 2: camera calibration algorithms

1. Describe the general PnP problem and derive the behavior of its solutions?
2. Explain the working principle of the P3P algorithm?
Explain and derive the DLT? What is the minimum number of point
correspondences it requires?
3. Define central and non central omnidirectional cameras?
4. What kind of mirrors ensure central projection?

### 4. Filtering & Edge detection

1. Explain the differences between convolution and correlation?
2. Explain the differences between a box filter and a Gaussian filter
3. Explain why should one increase the size of the kernel of a Gaussian filter if is large (i.e. close to the size of the filter kernel?
4. Explain when would we need a median & bilateral filter?
5. Explain how to handle boundary issues?
6. Explain the working principle of edge detection with a 1D signal?
7. Explain how noise does affect this procedure?
8. Explain the differential property of convolution?
9. Show how to compute the first derivative of an image intensity function along 𝑥and 𝑦
10. Explain why the Laplacian of Gaussian operator is useful?
11. List the properties of smoothing and derivative filters
12. Illustrate the Canny edge detection algorithm?
13. Explain what non maxima suppression is and how it is implemented?

### 5. Point Feature Detectors, Part 1

1. Explain what is template matching and how it is implemented?
    - Use filter as templates, then use correlation (or other similarity measures) to detect locations.
2. Explain what are the limitations of template matching? Can you use it to recognize cars?
    - Template matching will only work if __scale, orientation, illumination,__ and in general, __the apperance of the template and the object to detect are very similar__.
3. Illustrate the similarity metrics SSD, SAD, NCC, and Census transform?
    - __Sum of Squared Differences (SSD)__: $$S S D=\sum_{u=-k}^{k} \sum_{\nu=-k}^{k}(H(u, v)-F(u, v))^{2}$$
    - __Sum of Absolute Differences (SAD)__: 
    $$
    S A D= \sum_{u=-k}^{k} \sum_{v=-k}^{k} \left| H(u, v)-F(u, v) \right| 
    $$
    - __Normalized Cross Correlation (NCC)__: $$N C C=\frac{\sum_{u=-k}^{k} \sum_{v=-k}^{k} H(u, v) F(u, v)}{\sqrt{\sum_{u=-k}^{k} \sum_{v=-k}^{k} H(u, v)^{2}} \sqrt{\sum_{u=-k}^{k} \sum_{v=-k}^{k} F(u, v)^{2}}}$$
    - Census transform: It maps an image patch to a bit string.
        - If a pixel is greater than the center pixel, its corresponding bit is set to 1, else to 0.
        - For a $$w \times w$$ window, the string will be $$w^2 - 1$$ bits long.
4. What is the intuitive explanation behind SSD and NCC?
    - SSD: $$\langle H, F\rangle=\|H\| F \| \cos \theta$$
    - NCC: $$\cos \theta=\frac{\langle H, F\rangle}{\|H\| F \|}$$
5. Explain what are good features to track? In particular, can you explain what are corners and blobs together with their pros and cons?
    - Corner have high localization accuracy -> corners are good for VO.
    - Coerner are less distinctive than blobs.
    - Blobs have less localization accuracy than corners.
    - Blobs are more distinctive than a corner. -> blobs are better for place recognition.
6. Explain the Harris corner detector? In particular:
    1. Use the Moravec definition of corner, edge and flat region.
        - Look at a region of pixels through a small window. Shifting a window in __any direction__ should give a __large intensity change__ (e.g., in SSD) in at least 2 directions.
        - $$S S D(\Delta x, \Delta y)=\sum_{x, y \in P}(I(x, y)-I(x+\Delta x, y+\Delta y))^{2}$$
    2. Show how to get the second moment matrix from the definition of SSD and first order approximation (show that this is a quadratic expression) and what is the intrinsic interpretation of the second moment matrix using an ellipse?
        - $$S S D(\Delta x, \Delta y) \approx[\Delta x \quad \Delta y] M\left[\begin{array}{l}\Delta x \\\Delta y\end{array}\right]$$
        - $$M=R^{-1}\left[\begin{array}{ll}\lambda_{1} & 0 \\0 & \lambda_{2}\end{array}\right] R$$
        - An ellipse with axis legnths determined by the __eigenvalues__ and the two axes' orientations determined by $$R$$ (i.e., the __eigenvectors__ of M)
        - The two eigenvectors identify the directions of largest and smallest changes of SSD
    3. What is the M matrix like for an edge, for a flat region, for an axis aligned 90 degree corner and for a non axis aligned 90 degree corner?
        - Edge: $$M=\left[\begin{array}{cc}\sum I_{x}^{2} & \sum I_{x} I_{y} \\\sum I_{x} I_{y} & \sum I_{y}^{2}\end{array}\right]=\left[\begin{array}{cc}0 & 0 \\0 & \lambda_{2}\end{array}\right]$$
        - Flat region: $$M=\left[\begin{array}{ll}\sum I_{x}^{2} & \sum I_{x} I_{y} \\\sum I_{x} I_{y} & \sum I_{y}^{2}\end{array}\right]=\left[\begin{array}{ll}0 & 0 \\0 & 0\end{array}\right]$$
        - Axis-aligned corner: $$M=\left[\begin{array}{cc}\sum I_{x}^{2} & \sum I_{x} I_{y} \\\sum I_{x} I_{y} & \sum I_{y}^{2}\end{array}\right]=\left[\begin{array}{cc}\cos \frac{\pi}{4} & -\sin \frac{\pi}{4} \\\sin \frac{\pi}{4} & \cos \frac{\pi}{4}\end{array}\right]\left[\begin{array}{cc}\lambda_{1} & 0 \\0 & \lambda_{2}\end{array}\right]\left[\begin{array}{cc}\cos \frac{\pi}{4} & \sin \frac{\pi}{4} \\-\sin \frac{\pi}{4} & \cos \frac{\pi}{4}\end{array}\right]$$
    4. What do the eigenvalues of M reveal?
    5. Can you compare Harris detection with Shi Tomasi detection?
        - Harris: $$R = \lambda_1 \lambda_2 - k(\lambda_1 + \lambda_2)^2 = det(M) - k\text{trace}^2 (M)$$
        - Shi-Tomasi: $$R = \min(\lambda_1, \lambda_2) > \text{threshold}$$
    6. Can you explain whether the Harris detector is invariant to illumination or scale changes? Is it invariant to view point changes?
        - Invariant to __affine__ illumination changes.
        - Invariant to __non-linear, but monotonic__ illumination changes.
        - It depends on the view point change.
    7. What is the repeatability of the Harris detector after rescaling by a factor of 2?

### 6. Point Feature Detectors, Part 2

1. How does automatic scale selection work?
    - Design a function on the image patch, which is "sclae invariant" (i.e., which has the same value for corresponding patches, even if they are at different scales)
    - Approach
        - Take a local maximum or minimum of this function
        - The patch size for which the maximum or minimum is achieved should be _invariant_ to image rescaling.
        - __Important__: this scale invariant patch size is found in each image __independently__!
        - When the right scale is found, the patches must be normalized so that they can be compared by SSD.
2. What are the good and the bad properties that a function for automatic scale selection should have or not have?
    - A "good" function for scale function should have a single & sharp peak.
    - __Sharp, local intensity changes__ are good regions to monitor in order to identify the scale 
        - __Blobs and corners__ are the __ideal locations__!
3. How can we implement scale invariant detection efficiently? (show that we can do this by resampling the image vs rescaling the kernel)
    - The __idea function__ for determining the scale is __one that highlights sharp discontinuities__.
    - __Solution:__ convolve image with a __kernel that highlights edges__ $$f = \text{Kernel} * \text{Image}$$
    - It has been shown that the __Laplacian of Gaussian kernel__ is optimal under certain assumptions [^Lindeberg94]: $$\operatorname{LoG}(x, y, \sigma)=\nabla^{2} G_{\sigma}(x, y)=\frac{\partial^{2} G_{\sigma}(x, y)}{\partial x^{2}}+\frac{\partial^{2} G_{\sigma}(x, y)}{\partial y^{2}}$$
    - Correct scale is found as local maxima or minima across consecutive smoothed images
4. How do we match descriptors?
    - Descriptor matching can be done using __(Z)SSD, (Z)SAD, or (Z)NCC, or Hamming Distance (Note: Hamming distance can only be used for binary descriptors__, like Census transform, or ORB, BRIEF, BRISK, FREAK).
5. How does a patch descriptor work?
    1. __Find correct scale__ using LoG operator
    2. __Rescale the patch__ to a default size (e.g., 8x8 pixels)
    3. __Find local orientation__: Dominant direction of gradient for the image patch (e.g., Harriseigenvectors)
    4. __De-rotate patch through "patch warping"__: This puts the patches into a __canonical orientation__.
6. How to warp a patch?
    1. Start with an "__empty__" canonical patch (all pixels set to 0)
    2. For each pixel (x, y) in the empty patch, apply the __warping function W(x, y)__ to compute the corresponding position in the source image. It will be in floating point and will fall between the image pixels.
    3. __Interpolate__ the intensity values of the 4 closest pixels in the detected image. E.g., __bilinear interpolation__: $$I(x, y) = I(0, 0)(1-x)(1-y) + I(0, 1)(1 - x)(y) + I(1, 0)(x)(1-y) + I(1, 1)(x)(y)$$
7. How does a HOG (Histogram of Oriented Gradients) descriptor?
    1. Firstly, __multiply the patch by a Gaussian kernel__ to make the shape circular rather than square.
    2. Then, __compute gradient vectors__ at each pixel.
    3. Build a __histogram of gradient orientations__, weighted by the gradient magnitudes. The histogram represents the HOG descriptor.
    4. Extract all local maxima of HOG.
        - Each local maximum above a threshold is a candidate dominant orientation. In this case, __construct a different keypoint desciptor (with different dominant orientation)for each__
    5. To __make the descriptor rotation invariant__, apply circular shift to the descriptor elements such that the dominant orientation coincides with 0 radians
8. How is the keypoint detection done in SIFT ([Scale Invariant Feature Transform](https://www.vlfeat.org/overview/sift.html)) and how does this differ from Harris?
    1. __Build a space-scale pyramid__:
        - The initial image is __incrementally convolved with Gaussians__ $$G(k^i \sigma)$$ to produce blurred images separated by a constant factor $$k$$ in scale space
            - The initial Gaussian $$G(\sigma)$$ has $$\sigma = 1.6$$
            - $$k$$ is chosen: $$k=2^{\frac{1}{s}}$$, where $$s$$ is the number of intervals into which each octave of scale space is divided.
            - For efficiency reasons, when $$k^i$$ equals $$2$$, the image is downsampled by a factor of $$2$$, and then procedure is repeated again up to $$5$$ octaves (pyramid levels).
    2. __Scale-space__ extrema detection
        - Detect maxima and minima of difference-of-Gaussian ($$DoG(x,y) = G_{k \sigma}(x, y) - G_{\sigma}(x, y)$$) in scale space
        - Each point is compared to its 8 neighbors in the current image and 9 neighbors in each of the two adjacent scales. (above and below). (Note: for each max and min found, output is the __location__ $$(x, y)$$ and the scale)
9. How does SIFT achieve orientation invariance?
   - The last step of HOG
10. How does SIFT achieve affine illumination invariance?
    - __Intensity normalization__: The descriptor vector $$v$$ is then normalized such that its $$l_2$$ norm is $$1$$. This guarantees that the descriptor is invariant to linear illumination changes (the descriptor is already invariant to additive illumination because it is based on gradients; so, overall, the SIFT descriptor is invariant to affine illumination changes.
11. How is the SIFT descriptor built?
    - Multiply the patch by a Gaussian filter
    - Divide patch into 4 × 4 sub patches = 16 cells
    - Compute HOG (8 bins, i.e., 8 directions) for all pixels inside each sub patch
    - Concatenate all HOGs into a single 1D vector:
        - Resulting SIFT descriptor: 4 × 4 × 8 = 128 values
    - Descriptor Matching: SSD (i.e., Euclidean distance)
12. What is the repeatability of the SIFT detector after a rescaling of 2? And for a 50 degrees viewpoint change?
    - The highest repeatability (repeatability = correspondences detected / correspondences present) is obtained when sampling 3 scales per octave.
    - SIFT features are __invariant__ to 2D __rotation__, and __reasonbly invariant to rescaling, viewpoint changes__ (up to 50 degrees), and __illumination__.
13. How does feature matching work?
    1. __Define distance function__ that compares two descriptors (SSD, SAD, NCC or Hamming distance for binary descriptors, e.g., Census, BRIEF, BRISK)
    2. __Brute-force matching__
        1. Test all the features in the second image
        2. Take the one at min distance, i.e., the __closest descriptor__
14. Explaint the problem of __closest descriptor__?
    - It can give good scores to very ambiguous (bad) matches (curse of dimensionality)
    - __Better approach__: compute ratio of distances to $$1^{st}$$ and $$2^{nd}$$ closest descriptor
    $$\frac{d_1}{d_2} < \text{Threshold (usually 0.8)}$$
15. Illustrate the 1st to 2nd closest ratio of SIFT detection: what’s the intuitive reasoning behind it? Where does the 0.8 factor come from?
    - (From SIFT paper) A threshold of 0.8 eliminates 90% of the false matches while discarding less than 5% of the correct matches.

### 7. Multiple-view geometry

### 8. Multiple-view geometry 2

### 9. Multiple-view geometry 3

### 10. Multiple-view geometry 3 continued

### 11. Optical Flow and Tracking (Lucas-Kanade)

### 12a. Dense 3D Reconstruction

1. Are you able to describe the multi view stereo working principle? (aggregated photometric error)
    - Step 1: Local methods
        - Estimate depth independently for each pixel
    - Step 2: Global methods
        - Refine the _depth map_ as a whole by enforcing smoothness. This process is called _regularization_.
    - Solution: Aggregated Photometric Error
        - Set the first image as reference and estimate depth at each pixel by minimizing the Aggregated Photometric Error in all subsequent frames
2. What are the differences in the behavior of the aggregated photometric error for corners , flat regions, and edges?
    - Corners: one clear minimum
    - Flat regions and edges __parallel to the epipolar line__: flat valleys
    - Non distinctive features: multiple minima
3. What is the disparity space image (DSI) and how is it built in practice?
    - For a given image point $$(u, v)$$ and for discrete depth hypotheses $$d$$, the __Aggregated Photometric Error__ $$C(u,v,d)$$ with respect to the reference image $$I_R$$ can be stored in a volumetric 3D grid called the __Disparity Space Image (DSI)__, where each voxel has value: $$C(u, v, d)=\sum_{k=R+1}^{R+n-1} \rho\left(I_{R}(u, v)-I_{k}\left(u^{\prime}, v^{\prime}, d\right)\right)$$.
4. How do we extract the depth from the DSI?
    - $$d(u, v)=\arg \min _{d} \sum_{(u, v)} C(u, v, d(u, v))$$
5. How do we enforce smoothness (regularization) and how do we incorporate depth discontinuities (mathematical expressions)?
    - $$E(d)=E_{d}(d)+\lambda E_{s}(d)$$
    - $$E_{d}(d)=\sum_{(u, v)} C(u, v, d(u, v))$$
    - $$E_{s}(d)=\sum_{(u, v)}\left(\frac{\partial d}{\partial u}\right)^{2} \rho_{I}\left(\frac{\partial I}{\partial u}\right)^{2}+\left(\frac{\partial d}{\partial v}\right)^{2} \rho_{I}\left(\frac{\partial I}{\partial v}\right)^{2}$$
    - $$\rho_I$$ is a monotonically descreasing function (e.g., logistic) of image gradients:
        - high for small image gradients (i.e., regularization term dominates)
        - low for high image gradients (i.e., data term dominates)
6. What happens if we increase lambda (the regularization term)? What if lambda is 0 ? And if lambda is too big?
    - if lambda is 0, data term dominates.
    - if lambda is too big, the regularization term $$E_s(d)$$ 
        - smooths non smooth surfaces (results of noisy measurements and ambiguous texture) as well as discontinuities
        - fill holes
7. What is the optimal baseline for multi view stereo?
    - Obtain depth map from small baselines
    - When baseline becomes large (e.g., >10% of the average scene depth), then create new reference frame (keyframe) and start a new depth map computation
8. What are the advantages of GPUs?
    - GPUs run thousands of lightweight threads in parallel
    - Well suited for data parallelism

### 12b. Place recognition

### 12c. Deep Learning Tutorial

### 13. Visual inertial fusion

### 14. Event based vision


### Literature

[^Lindeberg94]: Lindeberg, Scale-space theory: A basic tool for analysing structures at different scales, Journal of Applied Statistics, 1994.