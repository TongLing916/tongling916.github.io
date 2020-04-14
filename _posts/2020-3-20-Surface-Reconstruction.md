---
layout:     post
title:      "Surface Reconstruction"
date:       2020-3-20
author:     Tong
catalog: true
tags:
    - Reconstruction
---

> [Regard 3D](http://www.regard3d.org/index.php)

> [CGAL](https://www.cgal.org/)

### [Open3D](https://github.com/intel-isl/Open3D)[^Zhou18]

#### Abstract

### [Voxblox](https://github.com/ethz-asl/voxblox)[^Oleynikova17]

#### Abstract

### [OpenChisel](https://github.com/personalrobotics/OpenChisel)[^Klingensmith15]

#### Abstract

### A survey of surface reconstruction from point clouds [^Berger17]

#### Abstract

### Floating scale surface reconstruction [^Fuhrmann14]

#### Abstract

### Reconstructing the World’s Museums [^Xiao14]

#### Abstract

### Exploiting visibility information in surface reconstruction to preserve weakly supported surfaces [^Jancosek14]

#### Abstract

We present a novel method for 3D surface reconstruction from __an input cloud of 3D points__ augmented with visibility information. We observe that it is possible to reconstruct surfaces that do not contain input points. Instead of modeling the surface from input points, we model free space from visibility information of the input points. The complement of the modeled free space is considered full space. The surface occurs at interface between the free and the full space. We show that under certain conditions a part of the full space surrounded by the free space must contain a real object also when the real object does not contain any input points; that is, an occluder reveals itself through occlusion. Our key contribution is the proposal of a new interface classifier that can also detect the occluder interface just from the visibility of input points. We use the interface classifier to modify the state-of-the-art surface reconstruction method so that it gains the ability to reconstruct weakly supported surfaces. We evaluate proposed method on datasets augmented with different levels of noise, undersampling, and amount of outliers. We show that the proposed method outperforms other methods in accuracy and ability to reconstruct weakly supported surfaces.

#### Contribution

The method proposed in this work is a generalization of the method proposed in [^Jancosek11]. The main contribution of the newly proposed method is that it produces more accurate results than the method proposed in [^Jancosek11] and accordingly is able to reconstruct weakly supported surfaces better. We propose a new interface classifier and we justify the design of the interface classifier by experiments.

### Let there be color! Large-scale texturing of 3D reconstructions [^Waechter14]

#### Abstract

3D reconstruction pipelines using structure-from-motion and multi-view stereo techniques are today able to reconstruct impressive, large-scale geometry models from images but do not yield textured results. Current texture creation methods are unable to handle the complexity and scale of these models. We therefore present the first comprehensive texturing framework for large-scale, real-world 3D reconstructions. Our method addresses most challenges occurring in such reconstructions: the large number of input images, their drastically varying properties such as image scale, (out-of-focus) blur, exposure variation, and occluders (e.g., moving plants or pedestrians). Using the proposed technique, we are able to texture datasets that are several orders of magnitude larger and far more challenging than shown in related work.

### [Screened poisson surface reconstruction](http://hhoppe.com/proj/poissonrecon/) [^Kazhdan13]

#### Abstract

### [SSD: Smooth Signed Distance Surface Reconstruction](http://mesh.brown.edu/ssd/index.html) [^Calakli11]

#### Abstract

### Unconstrained isosurface extraction on arbitrary octrees [^Kazhdan07]

#### Abstract

### Poisson Surface Reconstruction [^Kazhdan06]

#### Abstract

### Delaunay triangulation based surface reconstruction [^Cazals06]

#### Abstract

### Reconstruction and representation of 3D objects with radial basis functions [^Carr01]

#### Abstract

### Truncated Signed Distance Function [^Curless96]

#### Abstract

### [Marching Cubes](http://graphics.stanford.edu/~mdfisher/MarchingCubes.html) [^Lorensen87]

#### Abstract

### Literature

[^Xiao14]: Xiao, Jianxiong, and Yasutaka Furukawa. "Reconstructing the World's Museums." International Journal of Computer Vision 110.3 (2014): 243-258.

[^Waechter14]: Waechter, Michael, Nils Moehrle, and Michael Goesele. "Let there be color! Large-scale texturing of 3D reconstructions." European conference on computer vision. Springer, Cham, 2014.

[^Jancosek14]: Jancosek, Michal, and Tomas Pajdla. "Exploiting visibility information in surface reconstruction to preserve weakly supported surfaces." International scholarly research notices 2014 (2014).

[^Jancosek11]: M. Jancosek and T. Pajdla, “Multi-view reconstruction preserving weakly-supported surfaces,” in Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition (CVPR ’11), pp. 3121–3128, June 2011.

[^Berger17]: Berger, Matthew, et al. "A survey of surface reconstruction from point clouds." Computer Graphics Forum. Vol. 36. No. 1. 2017.

[^Cazals06]: Cazals, Frédéric, and Joachim Giesen. "Delaunay triangulation based surface reconstruction." Effective computational geometry for curves and surfaces. Springer, Berlin, Heidelberg, 2006. 231-276.

[^Fuhrmann14]: Fuhrmann, Simon, and Michael Goesele. "Floating scale surface reconstruction." ACM Transactions on Graphics (ToG) 33.4 (2014): 1-11.

[^Carr01]: Carr, J. C., et al. "Reconstruction and representation of 3D objects with radial basis functions." international conference on computer graphics and interactive techniques (2001): 67-76.

[^Kazhdan13]: Kazhdan, Michael, and Hugues Hoppe. "Screened poisson surface reconstruction." ACM Transactions on Graphics (ToG) 32.3 (2013): 1-13.

[^Kazhdan07]: Kazhdan, Michael, et al. "Unconstrained isosurface extraction on arbitrary octrees." symposium on geometry processing (2007): 125-133.

[^Kazhdan06]: Kazhdan, Michael, Matthew Bolitho, and Hugues Hoppe. "Poisson surface reconstruction." Proceedings of the fourth Eurographics symposium on Geometry processing. Vol. 7. 2006.

[^Lorensen87]: Lorensen, William E., and Harvey E. Cline. "Marching cubes: A high resolution 3D surface construction algorithm." ACM siggraph computer graphics 21.4 (1987): 163-169.

[^Curless96]: Curless, Brian, and Marc Levoy. "A volumetric method for building complex models from range images." Proceedings of the 23rd annual conference on Computer graphics and interactive techniques. 1996.

[^Klingensmith15]: Klingensmith, Matthew, et al. "Chisel: Real Time Large Scale 3D Reconstruction Onboard a Mobile Device using Spatially Hashed Signed Distance Fields." Robotics: science and systems. Vol. 4. 2015.

[^Oleynikova17]: Oleynikova, Helen, et al. "Voxblox: Incremental 3d euclidean signed distance fields for on-board mav planning." 2017 Ieee/rsj International Conference on Intelligent Robots and Systems (iros). IEEE, 2017.

[^Zhou18]: Zhou, Qian-Yi, Jaesik Park, and Vladlen Koltun. "Open3D: A modern library for 3D data processing." arXiv preprint arXiv:1801.09847 (2018).

[^Calakli11]: Calakli, Fatih, and Gabriel Taubin. "SSD: Smooth Signed Distance Surface Reconstruction." Computer Graphics Forum 30.7 (2011): 1993-2002.