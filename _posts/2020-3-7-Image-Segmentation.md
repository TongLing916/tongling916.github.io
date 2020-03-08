---
layout:     post
title:      "Image Segmentation"
date:       2020-3-7
author:     Tong
catalog: true
tags:
    - Technique
---

### Mean Shift: A Robust Approach toward Feature Space Analysis[^Comaniciu2002]

### SLIC Superpixels Compared to State-of-the-art Superpixel Methods

#### Abstract

Computer vision applications have come to rely increasingly on superpixels in recent years, but it is not always clear what constitutes a good superpixel algorithm. In an effort to understand the benefits and drawbacks of existing methods, we empirically compare five state-of-the-art superpixel algorithms for their ability to adhere to image boundaries, speed, memory efficiency, and their impact on segmentation performance. We then introduce a new superpixel algorithm, _simple linear iterative clustering_ (SLIC), which adapts a k-means clustering approach to efficiently generate superpixels. Despite its simplicity, SLIC adheres to boundaries as well as or better than previous methods. At the same time, it is faster and more memory efficient, improves segmentation performance, and is straightforward to extend to supervoxel generation.

#### Existing superpixel methods

- Graph-based algorithms: Graph-based approaches to superpixel generation treat each pixel as a node in a graph. Edge weights between two nodes are proportional to the similarity between neighboring pixels. Superpixels are created by minimizing a cost function defined over the graph.
    - NC05[^Shi2000]: The Normalized cuts algorithm recursively partitions a graph of all pixels in the image using contour and texture cues, globally minimizing a cost function defined on the edges at the partition boundaries.
    - GS04[^Felzenszwalb2004]: It performs an agglomerative clustering of pixels as nodes on a graph, such that each superpixel is the minimum spanning tree of the constituent pixels.
    - SL08[^Moore2008]: Moore et al. propose a method to generate superpixels that conform to a grid by finding optimal paths, or seams, that split the image into smaller vertical or horizontal regions.
    - GCa10 and GCb10[^Veksler2010]: Superpixels are obtained by stitching together overlapping image patches such that each pixel belongs to only one of the overlapping regions. They suggest two variants of their method, one for generating compact superpixels (GCa10) and one for constant-intensity superpixels (GCb10).
- Gradient-ascent-based algorithms: Starting from a rough initial clustering of pixels, gradient ascent methods iteratively refine the clusters until some convergence criterion is met to form superpixels.
    - MS02[^Comaniciu2002]: mean shift, an iterative mode-seeking procedure for locating local maxima of a density function, is applied to find modes in the color or intensity feature space of an image.
    - QS08[^Vedaldi2008]: Quick shift also uses a mode-seeking segmentation scheme. It initializes the segmentation using a medoid shift procedure. It then moves each point in the feature space to the nearest neighbor that increases the Parzen density estimate.
    - WS91[^Vincent1991]: The watershed approach performs a gradient ascent starting from local minima to produce watersheds, lines that separate catchment basins.
    - TP09[^Levinshtein2009]: The Turbopixel method progressively dilates a set of seed locations using level-set based geometric flow.

#### SLIC Superpixels

- __Algorithm__

- __Distance measure__

- __Post-processing__

- __Complexity__

### Literature

[^Achanta2012]: Achanta, Radhakrishna, et al. "SLIC superpixels compared to state-of-the-art superpixel methods." IEEE transactions on pattern analysis and machine intelligence 34.11 (2012): 2274-2282.

[^Shi2000]: Shi, Jianbo, and Jitendra Malik. "Normalized cuts and image segmentation." IEEE Transactions on pattern analysis and machine intelligence 22.8 (2000): 888-905.

[^Felzenszwalb2004]: Felzenszwalb, Pedro F., and Daniel P. Huttenlocher. "Efficient graph-based image segmentation." International journal of computer vision 59.2 (2004): 167-181.

[^Comaniciu2002]: Comaniciu, Dorin, and Peter Meer. "Mean shift: A robust approach toward feature space analysis." IEEE Transactions on pattern analysis and machine intelligence 24.5 (2002): 603-619.

[^Moore2008]: Alastair Moore, Simon Prince, Jonathan Warrell, Umar Mohammed, and Graham Jones. Superpixel Lattices. IEEE Computer Vision and Pattern Recognition (CVPR), 2008.

[^Veksler2010]: O. Veksler, Y. Boykov, and P. Mehrani. Superpixels and supervoxels in an energy optimization framework. In European Conference on Computer Vision (ECCV), 2010.

[^Vedaldi2008]: A. Vedaldi and S. Soatto. Quick shift and kernel methods for mode seeking. In European Conference on Computer Vision (ECCV), 2008.

[^Vincent1991]: Luc Vincent and Pierre Soille. Watersheds in digital spaces: An efficient algorithm based on immersion simulations. IEEE Transactions on Pattern Analalysis and Machine Intelligence, 13(6):583–598, 1991.

[^Levinshtein2009]: A. Levinshtein, A. Stere, K. Kutulakos, D. Fleet, S. Dickinson, and K. Siddiqi. Turbopixels: Fast superpixels using geometric flows. IEEE Transactions on Pattern Analysis and Machine Intelligence (PAMI), 2009.
