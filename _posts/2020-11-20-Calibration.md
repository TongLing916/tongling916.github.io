---
layout:     post
title:      "Calibration"
date:       2020-11-20
author:     Tong
catalog: true
tags:
    - VIO
---

> [kalibr](https://github.com/ethz-asl/kalibr)

> [kalibr allan](https://github.com/rpng/kalibr_allan)

> [AprilTag](https://github.com/AprilRobotics/apriltag)

### Intrinsics

#### Camera 

1. [Projection model](https://tongling916.github.io/2020/03/26/Camera-Model/)
2. Distortion model[^Brown66]
3. [Photometric calibration](https://github.com/tum-vision/online_photometric_calibration)[^Engel16][^Bergmann17]

#### IMU

1. [kalibr](https://github.com/ethz-asl/kalibr/wiki/Multi-IMU-and-IMU-intrinsic-calibration)
2. [IMU noise](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model)
3. [Intrinsic calibration](http://www.roboticsproceedings.org/rss16/p026.pdf)[^Yang20]
4. [Degenerate motion](http://udel.edu/~pgeneva/downloads/papers/r05.pdf)[^Yang19]


### Extrinsics

#### Camera-Camera

1. [kalibr](https://github.com/ethz-asl/kalibr/wiki/multiple-camera-calibration)
2. [CamOdoCal](https://github.com/hengli/camodocal)[^Heng13][^Heng14][^Heng15]
3. AprilTag[^Olson11][^Wang16][^Krogius19]
   - As each AprilTag marker has a unique identifier, calibration will work even if multiple cameras observe different parts of the target.

#### Camera-IMU

1. [kalibr](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration)
2. [vmav-ros-pkg](https://github.com/hengli/vmav-ros-pkg)[^Heng14b] [^Heng15b] [^Heng15c]

#### IMU-IMU

1. [kalibr](https://github.com/ethz-asl/kalibr/wiki/Multi-IMU-and-IMU-intrinsic-calibration)[^Furgale13] [^Furgale12] [^Rehder16b] [^Zhang02]

#### Camera-LiDAR

1. [laser-cam-calib](https://github.com/zhixy/Laser-Camera-Calibration-Toolbox)[^Zhang04] [^Vasconcelos12]
2. [kalibr](https://github.com/ethz-asl/kalibr/wiki/Camera-IMU-LRF-calibration)[^Zhang04] [^Rehder16a]
3. [LIBCBDETECT](http://www.cvlibs.net/software/libcbdetect/)[^Geiger12]

#### Camera-Radar


### Literature

[^Yang19]: Yang, Yulin; Geneva, Patrick; Eckenhoff, Kevin; Huang, Guoquan. "Degenerate Motion Analysis for Aided INS with Online Spatial and Temporal Sensor Calibration." 2019 RAL.

[^Yang20]: Yang, Yulin, et al. "Online IMU Intrinsic Calibration: Is It Necessary?." 2020 RSS.

[^Brown66]: Brown, Duane C. "Decentering distortion of lenses." Photogrammetric Engineering and Remote Sensing (1966).

[^Engel16]: Engel, Jakob, Vladyslav Usenko, and Daniel Cremers. "A photometrically calibrated benchmark for monocular visual odometry." arXiv preprint arXiv:1607.02555 (2016).

[^Bergmann17]: Bergmann, Paul, Rui Wang, and Daniel Cremers. "Online photometric calibration of auto exposure video for realtime visual odometry and slam." IEEE Robotics and Automation Letters 3.2 (2017): 627-634.

[^Olson11]: Olson, Edwin. "AprilTag: A robust and flexible visual fiducial system." 2011 IEEE International Conference on Robotics and Automation. IEEE, 2011.

[^Wang16]: Wang, John, and Edwin Olson. "AprilTag 2: Efficient and robust fiducial detection." 2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2016.

[^Krogius19]: Krogius, Maximilian, Acshi Haggenmiller, and Edwin Olson. "Flexible Layouts for Fiducial Tags." IROS. 2019.

[^Zhang04]: Q. Zhang and R. Pless, “Extrinsic calibration of a camera and laser range finder (improves camera calibration),” in Intelligent Robots and Systems, 2004.(IROS 2004). Proceedings. 2004 IEEE/RSJ International Conference on, 2004, vol. 3, pp. 2301–2306.

[^Vasconcelos12]: F. Vasconcelos, J. P. Barreto, and U. Nunes, “A Minimal Solution for the Extrinsic Calibration of a Camera and a Laser-Rangefinder,” IEEE Transactions on Pattern Analysis and Machine Intelligence, vol. 34, no. 11, pp. 2097–2107, Nov. 2012.

[^Rehder16a]:  Joern Rehder, Roland Siegwart, and Paul Furgale. "A General Approach to Spatiotemporal Calibration in Multisensor Systems." IEEE Transactions on Robotics 32.2 (2016): 383-398.

[^Furgale13]: Paul Furgale, Joern Rehder, Roland Siegwart (2013). Unified Temporal and Spatial Calibration for Multi-Sensor Systems. In Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Tokyo, Japan.

[^Furgale12]: Paul Furgale, T D Barfoot, G Sibley (2012). Continuous-Time Batch Estimation Using Temporal Basis Functions. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pp. 2088–2095, St. Paul, MN.

[^Rehder16b]: Joern Rehder, Janosch Nikolic, Thomas Schneider, Timo Hinzmann, Roland Siegwart (2016). Extending kalibr: Calibrating the Extrinsics of Multiple IMUs and of Individual Axes. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), Stockholm, Sweden.

[^Zhang02]: Zhang, Li, Zhen Liu, and C. Honghui Xia. (2002). Clock synchronization algorithms for network measurements. In the Proceedings of the IEEE Twenty-First Annual Joint Conference of the IEEE Computer and Communications Societies.

[^Heng13]: Lionel Heng, Bo Li, and Marc Pollefeys, CamOdoCal: Automatic Intrinsic and Extrinsic Calibration of a Rig with Multiple Generic Cameras and Odometry, In Proc. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2013.

[^Heng14]: Lionel Heng, Mathias Bürki, Gim Hee Lee, Paul Furgale, Roland Siegwart, and Marc Pollefeys, Infrastructure-Based Calibration of a Multi-Camera Rig, In Proc. IEEE International Conference on Robotics andAutomation (ICRA), 2014.
    
[^Heng15]: Lionel Heng, Paul Furgale, and Marc Pollefeys, Leveraging Image-based Localization for Infrastructure-basedCalibration of a Multi-camera Rig, Journal of Field Robotics (JFR), 2015.

[^Heng15b]: Heng, Lionel, Gim Hee Lee, and Marc Pollefeys. "Self-calibration and visual slam with a multi-camera system on a micro aerial vehicle." Autonomous robots 39.3 (2015): 259-277.

[^Heng14b]: Heng, Lionel, Gim Hee Lee, and Marc Pollefeys. "Self-calibration and visual slam with a multi-camera system on a micro aerial vehicle." Autonomous robots 39.3 (2015): 259-277.

[^Heng15c]: Heng, Lionel, Paul Furgale, and Marc Pollefeys. "Leveraging Image‐based Localization for Infrastructure‐based Calibration of a Multi‐camera Rig." Journal of Field Robotics 32.5 (2015): 775-802.

[^Geiger12]: Geiger, Andreas, et al. "Automatic camera and range sensor calibration using a single shot." 2012 IEEE International Conference on Robotics and Automation. IEEE, 2012.