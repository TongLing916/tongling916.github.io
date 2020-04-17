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
3. Explain and derive the DLT? What is the minimum number of point correspondences it requires?
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

1. Can you relate Structure from Motion to 3D reconstruction? In what they differ?
   - __3D reconstruction from multiple views__
     - Assumptions: K, T and R are __known__.
     - Goal: Recover the 3D structure from images
   - __Structure from Motion__
     - Assumptions: none (K, T and R are __unknown__)
     - Goal: Recover simutaneously 3D scene structure and camera poses (up to scale) from multiple images
2. Can you define disparity in both the simplified and the general case?
   - Simplified case: identical cameras and aligned with the x-axis
     - Disparity: difference in image location of the projection of a 3D point on two image planes
   - General case: non identical cameras and not aligned
3. Can you provide a mathematical expression of depth as a function of the baseline, the disparity and the focal length?
   - $$u_l - u_r = \frac{bf}{z_p}$$
4. Can you apply error propagation to derive an expression for depth uncertainty? How can we improve the uncertainty?
5. Can you analyze the effects of a large/small baseline?
   - Too large:
     - Minimum measurable depth increases
     - Difficul search problem for close objects
   - Too small:
     - Large depth error
6. What is the closest depth that a stereo camera can measure?
   - focal length
7. Are you able to show mathematically how to compute the intersection of two lines (linearly and non
linearly)?
    - Linear: Cross product (SVD)
    - Non linear: Minimize the __Sum of Squared Reprojection Error__
8. What is the geometric interpretation of the linear and non linear approaches and what error do they minimize?
   - Linear: SVD
   - Non linear: Sum of Squared Reprojection Error 
9.  Are you able to provide a definition of epipole, epipolar line and epipolar plane?
    - The __epipole__ is the projection of the optical center on the other camera image
    - Given two camera centers and one image point, an __epipolar plane__ can be uniquely defined. 
    - The intersections of the epipolar plane with the two image planes are called __epipolar lines__
10. Are you able to draw the epipolar lines for two converging cameras, for a forward motion situation, and for a side moving camera?
    - __Remember:__ all the epipolar lines intersect at the epipole
    - As the position of the 3D point changes, the epipolar lines _rotate_ about the baseline
11. Are you able to define stereo rectification and to derive mathematically the rectifying homographies?
    - Stereo rectification warps original image planes onto a coplanar planes parallel to the baseline
    - It works by computing two homographies, one for each input image reprojection
    - As a result, the new __epipolar lines__ are __horizontal__ and the __scanlines__ of the left and right image __are aligned__
    - Paper: A compact algorithm for rectification of stereo pairs
12.  How is the disparity map computed?
    1. For each pixel on the left image, find its corresponding point on the right image
    2. Compute the disparity for each pair of correspondences
    3. Visualize it in gray-scale or color coded image
13.  How can one establish stereo correspondences with subpixel accuracy?
    - Interpolation of intensity
14.  Describe one or more simple ways to reject outliers in stereo correspondences.
    - Uniqueness: only one match in right image for every point in left image
    - Ordering: Points on __same surface__ will be in same order in both views
    - Disparity gradient: disparity changes smoothly between points on the same surface
15.  Is stereo vision the only way of estimating depth information? If not, are you able to list alternative options?
    - Use deep learning (e.g. [Pyramid Stereo Matching Network](http://openaccess.thecvf.com/content_cvpr_2018/papers/Chang_Pyramid_Stereo_Matching_CVPR_2018_paper.pdf))

### 8. Multiple-view geometry 2

1. What's the minimum number of correspondences required for calibrated SFM and why?
   - 4n knowns
     - n correspondences; each one $$(u_1, v_1)$$ and $$(u_2, v_2)$$
   - 5 + 3n unknowns
     - 5 for motion up to a scale (3 for rotation, 2 for translation)
     - 3n = number of coordinates of the n 3D points
   - If and only if the number of independent equations >= number of unknowns
     - 4n >= 5 + 3n => n >= 5
2. Are you able to derive the epipolar constraint?
   - $$\bar{p}_{2}^{T} E \bar{p}_{1}=0$$
   - normalized image coordinates
3. Are you able to define the essential matrix?
   - $$\mathrm{E}=[T]_{\times} R$$
   - E: essential matrix
   - T: translation vector
   - R: rotation matrix
4. Are you able to derive the 8 point algorithm?
   - Each pair of point correspondences provides a linear equation
   - SVD to solve it
   - Degenerate configurations: when the 3D points are coplanar (Conversely, 5-point algorithm works also for coplanar points).
5. How many rotation translation combinations can the essential matrix be decomposed into?
   - 4 possible solutions of R and T
   - Only one solution where points are in front of both cameras
6. Are you able to provide a geometrical interpretation of the epipolar constraint?
   - $$\overline{\boldsymbol{p}}_{2}^{\top} \cdot E \overline{\boldsymbol{p}}_{1}=\left\|\overline{\boldsymbol{p}}_{2}\right\|\left\|\boldsymbol{E} \overline{\boldsymbol{p}}_{1}\right\| \cos (\theta)$$
   - This product depends on the angle $$\theta$$ between $$\overline{\boldsymbol{p}}_{1}$$ and the normal $$n=E p_{1}$$ to the epipolar plane. It is non zero when $$\overline{\boldsymbol{p}}_{1}$$, $$\overline{\boldsymbol{p}}_{2}$$, and $$T$$ are not coplanar.
7. Are you able to describe the relation between the essential and the fundamental matrix?
   - $$\mathbf{F} = \mathbf{K}^{-T}_{2} \mathbf{E} \mathbf{K}^{-1}_{1}$$
8. Why is it important to normalize the point coordinates in the 8 point algorithm?
   - Orders of magnitude difference between column of data matrix --> least squares yield poor results
   - Poor numerical conditioning, which makes results very sensitive to noise
9.  Describe one or more possible ways to achieve this normalization.
    - Transform image coordinates so that they are in the range ~ `[-1, 1] x [-1, 1]`
    - Hartley proposed to rescale the two point sets such that the centroid of each set is 0 and the mean standard deviation $$\sqrt{2}$$, so that the "average" point is equal to $$\begin{bmatrix}0 & 0 & 1\end{bmatrix}^{T}$$ (in homogeneous coordinates)
    - This can be done for every point as follows $$\widehat{p}^{i}=\frac{\sqrt{2}}{\sigma}\left(p^{i}-\mu\right)$$
    - where $$\mu=\left(\mu_{x}, \mu_{y}\right)=\frac{1}{N} \sum_{i=1}^{n} p^{i}$$ is the centroid and $$\sigma=\frac{1}{N} \sum_{i=1}^{n}\left\|p^{i}-\mu\right\|^{2}$$ is the mean standard deviation of the point set.
    - This transformation can be expressed in matrix form using homogeneous coordinates $$\widehat{p^{i}}=\left[\begin{array}{ccc}\frac{\sqrt{2}}{\sigma} & 0 & -\frac{\sqrt{2}}{\sigma} \mu_{x} \\0 & \frac{\sqrt{2}}{\sigma} & -\frac{\sqrt{2}}{\sigma} \mu_{y} \\0 & 0 & 1\end{array}\right] p^{i}$$
10. Are you able to describe the normalized 8 point algorithm[^Hartley97]?
    1. __Normalize__ point correspondences: $$\widehat{p_{1}}=B_{1} p_{1}, \quad \widehat{p_{2}}=B_{2} p_{2}$$
    2. Estimate __normalized__ $$\widehat{F}$$ with 8-point algorithm using normalized coordinates $$\widehat{p}_{1}, \widehat{p_{2}}$$
    3. Compute __unnormalized__ $$\mathbf{F}$$ from $$\widehat{F}$$: $$\mathrm{F}=\mathrm{B}_{2}^{\top} \widehat{\mathrm{F}} \mathrm{B}_{1}$$
11. Are you able to provide quality metrics for the essential matrix estimation?
    - __Algebraic error__: defined directly by the Epipolar Constraint: $$e r r=\sum_{i=1}^{N}\left({\bar{p}_{2}^{i}}^{T} \boldsymbol{E} \bar{p}_{1}^{i}\right)^{2}$$
    - __Direcitonal Error__: sum of __squared consines of the angle from the epipolar plane__ $$\operatorname{err}=\sum_{i}\left(\cos \left(\theta_{i}\right)\right)^{2}$$, where $$\cos (\theta)=\frac{\boldsymbol{p}^{T}_{2} \cdot \boldsymbol{E} \boldsymbol{p}_{1}}{\left\|\boldsymbol{p}_{2}\right\|\left\|\boldsymbol{E} \boldsymbol{p}_{1}\right\|}$$
    - __Epipolar Line Distance__: sum of __squared epipolar-line-to-point distances__ $$e r r=\sum_{i=1}^{N} d^{2}\left(p_{1}^{i}, l_{1}^{i}\right)+d^{2}\left(p_{2}^{i}, l_{2}^{i}\right)$$ 
      - Cheaper than reprojection error because it does not require point triangulation
    - __Reprojeciton Error__: sum of the __squared reprojection errors__ $$e r r=\sum_{i=1}^{N}\left\|p_{1}^{i}-\pi_{1}\left(P^{i}\right)\right\|^{2}+\left\|p_{2}^{i}-\pi_{2}\left(P^{i}, R, T\right)\right\|^{2}$$
      - Computation is expensive because it requires point triangulation.
      - However, it is the most popular because __more accurate__.
12. Why do we need RANSAC[^Fischler81]?
    - RANSAC is the __standard method for model fitting in the presence of outliers__.
13. What is the theoretical maximum number of combinations to explore?
    - N(N-1)/2 for computation of a line
14. After how many iterations can RANSAC be stopped to guarantee a given success probability?
    - $$k=\frac{\log (1-p)}{\log \left(1-w^{s}\right)}$$
    - where $$k$$ is the maximum number of iterations, $$p$$ is the probability of success, and $$w$$ is the fraction of inliers ($$1-w$$ is the fraction of outliers), $$s$$ is the number required to estimate a model.
15. What is the trend of RANSAC vs. iterations, vs . the fraction of outliers, vs. the number of points to estimate the model?
    - $$k$$ is exponential in the number of points $$s$$ necessary to estimate the model
16. How do we apply RANSAC to the 8 point algorithm, DLT, P3P?
17. How can we reduce the number of RANSAC iterations for the SFM problem?
    - Use __motion constraints__.
    - Wheeled vehicles, like cars, follow locally planar circular motion about the Instantaneous Center of Rotation (ICR).[^Scaramuzza11]

### 9. Multiple-view geometry 3

1. Are you able to define Bundle Adjustment (via mathematical expression and illustration)?
   - Non-linear, simultaneous refinement of structure and motion.
   - Minimize the sum of squared reprojection errors.
   - Levenberg-Marquardt is more robust than Gauss-Newton to local minima.
   - In order to not get stuck in local minima, the __initialization should be close the minimum__.
   - To prevent that large reprojection erros can negatively influence the optimization, a robust cost function (__Huber__ or __Tukey__) is used to penalize wrong matches.
2. Are you able to describe hierarchical and sequential SFM for monocular VO?
   - __Hierarchical SFM__
     1. Extract and match features between nearby frames
     2. Identify clusters consisting of 3 nearby frames
     3. Compute SFM for 3 views.
        1. Compute SFM between 1 and 2 and build point cloud
        2. Then merge the third view by running 3-point RANSAC between point cloud and the third view
     4. Merge clusters pairwise and refine (BA) both structure and motion
   - __Sequential SFM (Visual Odometry)__
     1. Initialize structure and motion from 2 views (__bootstrapping__)
     2. For each additional view
        1. Determine pose (__localization__)
        2. Extend structure, i.e., extract and triangulate new features (__mapping__)
        3. Refine structure and motion through Bundle Adjustment (BA) (__optimization__)
3. What are keyframes? Why do we need them and how can we select them?
   - When frames are taken at nearby positions compared to the scene distance, 3D points will exibit large uncertainty
   - One way to avoid this consists of __skipping frames__ until the average uncertainty of the 3D points decreases below a certain threshold. The selected frames are called __keyframes__.
   - __Rule of the thumb__: add a keyframe when $$\frac{\text{keyframe distance}}{\text{average depth}} > \text{threshold} (10-20\%)$$
4. Are you able to define loop closure detection? Why do we need loops?
   - __Loop detection__ to avoid map duplication
   - __Loop correction__ to compensate the acccumulated drift
5. Are you able to provide a list of the most popular open source VO and VSLAM algorithms?
6. Are you able to describe the differences between feature based methods and direct methods?

### 10. Multiple-view geometry 3 continued

1. How do we benchmark VO/SLAM algorithms?
   - [Devon Island](http://asrl.utias.utoronto.ca/datasets/devon-island-rover-navigation/): Stereo + D-GPS + inclinometer + sun sensor
   - [KITTI](www.cvlibs.net/datasets/kitti/): Automobile, Laser + stereo + GPS
   - [EuRoC](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets): MAV with synchronized IMU and stereo
   - [Blackbird](https://github.com/mit-fast/Blackbird-Dataset): MAV indoor aggressive flight with rendered images and real dynamics + IMU
   - [MVSEC](https://daniilidis-group.github.io/mvsec/): Events, frame, lidar, GPS, IMU from cars, drones, and motorcycles
   - [UZH Drone Racing](http://rpg.ifi.uzh.ch/uzh-fpv.html): MAV aggressive flight, standard + event cameras, IMU, indoors and outdoors
2. Along which axes can we evaluate them?
   - Robustness (HDR, motion blur, low texture)
   - Efficiency (speed, memory, CPU load)
   - Accuracy (ATE, RTE)
3. Benchmarking accuracy: Can we use the end pose error? What are ATE and RTE?
   - ATE: Absolute Trajectory Error
   - RTE: Relative Trajectory Error
4. How can we quantify Efficiency? And Robustness?
5. What are open research opportunities?

### 11. Optical Flow and Tracking (Lucas-Kanade)

1. Are you able to illustrate tracking with block matching?
   - Search for the corresponding patch in a $$D \times D$$ region around the point to track
   - Use SSD, SAD, or NCC
2. Are you able to explain the underlying assumptions behind differential methods, derive their mathematical expression and the meaning of the M matrix?
   - Brightness constancy: The intensity of the pixels around the point to track in image $$I_0$$ should be the same of its corresponding pixels in image $$I_1$$
   - Temporal consistency: The motion between two frames must be small (1-2 pixels at the most)
   - Spatial coherency: Neighboring pixels belonging to the same surface and therefore undergo similar motion
   - Method: find the motion vector $$(u, v)$$ that minimizes the sum of squared differences (SSD)
   - M matrix is the same as the one in Harris detector. 
3. When is this matrix invertible and when not?
   - $$\text{det}(M)$$ should be non zero, which means that its eigenvalues should be large (i.e., not a flat region, not an edge)
   - In practice, it __should be a corner or more generally contain texture!__
4. What is the aperture problem and how can we overcome it?
   - Look at the local brightness chnages __through a small aperture__
   - We __cannot always determine__ the motion direction -> __Infinite motion solutions__ may exist
   - Solution: increase aperture size
5. What is optical flow?
   - Optical flow is the pattern of apparent motion of objects in a visual scene caused by the relative motion between the observer and the scene.
6. Can you list pros and cons of block based vs. differential methods for tracking?
   - Block-based method
     - Pros: __Robust__ to large motions
     - Cons: Can be __computationally expensive__ ($$D \times D$$ validations need to be made for a single point for track)
   - Differential method
     - Pros: Much more __efficient__ than block-based methods.
     - Cons: Works only for __small motions__. For larger motion, multi-scale implementations are used but are more expensive.
7. Are you able to describe the working principle of KLT?
   - Uses the Gauss-Newton method for minimization, that is:
     - Applies a first-order approximation of the warp
     - Attempts to minimize the SSD iteratively
8. Are you able to derive the main mathematical expression for KLT?
   - NO :P
9.  What is the Hessian matrix and for which warping function does it coincide to that used for point tracking?
10. Can you list Lukas Kanade failure cases and how to overcome them?
    - If the initial estimate is too far, then the linear approximation does not longer hold.
      - Solution: pyramidal implementations
    - Deviations from the mathematical model: object deformations, illumination changes, etc.
    - Occlusions
      - Solution: Update the template with the last image
11. How does one get the initial guess?
    - Start at rest
    - Use SIFT to find the object and then track it
12. Can you illustrate the coarse to fine Lucas Kanade implementation?
13. Can you illustrate alternative tracking procedures using point features?
    - Step 1: Keypoint detection and matching
      - Invariant to scale, rotation, perspective
    - Step 2: Geometric verification (RANSAC)

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

1. What is an inverted file index?
   - We want to find all images in which a feature occurs
   - Inverted File Index lists all visual words in the vocabulary
2. What is a visual word?
   1. Collect a large enough dataset
   2. Extract fetures and descriptors from each image and map them into the descriptor space
   3. Cluster the descriptor space into K clusters
   4. The centroid of each cluster is a __visual word__. (A visual word is the __centroid__ of a cluster of similar features)
3. How does K means clustering work?
   1. Randomly initialize k cluster centers
   2. Iterate until convergence
      - Assign each data point to the nearest center
      - Recomptue each cluster center as the mean of all points assigned to it
4. Why do we need hierarchical clustering?
   - With hierarchical clustering, we do __NOT__ need to compare every feature in the query image against all features in the vocabulary.
5. Explain and illustrate image retrieval using Bag of Words.
6. Discussion on place recognition: what are the open challenges and what solutions have been proposed?
   - Problem: Visual vocabulary discards the spatial relationships between features.
   - Solution: This can be overcome using __geometric verification__.

### 12c. Deep Learning Tutorial

### 13. Visual inertial fusion

1. Why is it recommended to use an IMU for Visual Odometry?
2. Why not just an IMU?
3. How does a MEMS IMU work?
4. What is the drift of an industrial IMU?
5. What is the IMU measurement model?
6. What causes the bias in an IMU?
7. How do we model the bias?
8. How do we integrate the acceleration to get the position formula?
9. What is the definition of loosely coupled and tightly coupled visual inertial fusions?
10. How can we use non linear optimization based approaches to solve for visual inertial fusion?

### 14. Event based vision

1. What is a DVS and how does it work?
2. What are its pros and cons vs. standard cameras?
3. Can we apply standard camera calibration techniques?
4. How can we compute optical flow with a DVS?
5. Could you intuitively explain why we can reconstruct the intensity?
6. What is the generative model of a DVS and how to derive it?
7. What is a DAVIS sensor?
8. What is the focus maximization framework and how does it work? What is its advantage compared with the generative model?
9. How can we get color events?


### Literature

[^Lindeberg94]: Lindeberg, Scale-space theory: A basic tool for analysing structures at different scales, Journal of Applied Statistics, 1994.

[^Fischler81]: M. A.Fischler and R. C.Bolles. Random sample consensus: A paradigm for model fitting with applications to image analysis and automated cartography. Graphics and Image Processing, 1981.

[^Hartley97]: Hartley, In defense of the eight-point algorithm, PAMI’97.

[^Scaramuzza11]: Scaramuzza, 1 Point RANSAC Structure from Motion for Vehicle Mounted Cameras by Exploiting Non holonomic Constraints , International Journal of Computer Vision, 2011.