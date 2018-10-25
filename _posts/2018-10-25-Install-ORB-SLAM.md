---
layout:     post
title:      "Install ORB-SLAM"
date:       2018-10-25
author:     Tong
catalog: true
tags:
    - SLAM
---

今天根据[ORB-SLAM][github-orb-slam]上面的提示，在VMWare中的Ubuntu16.04上安装了ORB-SLAM，下面是安装过程中遇到的一些问题。（注意：由于是Ubuntu16.04，所以本人ROS安装的版本是kinect，而不是indigo）

## Error 1

{% highlight bash %}
/home/tong/ORB_SLAM/src/ORBextractor.cc: In member function ‘void ORB_SLAM::ORBextractor::ComputeKeyPoints(std::vector<std::vector<cv::KeyPoint> >&)’:
/home/tong/ORB_SLAM/src/ORBextractor.cc:607:63: error: ‘FAST’ was not declared in this scope
                 FAST(cellImage,cellKeyPoints[i][j],fastTh,true);
                                                               ^
/home/tong/ORB_SLAM/src/ORBextractor.cc:616:34: error: ‘ORB’ has not been declared
                 if( scoreType == ORB::HARRIS_SCORE )
                                  ^
/home/tong/ORB_SLAM/src/ORBextractor.cc:683:17: error: ‘KeyPointsFilter’ has not been declared
                 KeyPointsFilter::retainBest(keysCell,nToRetain[i][j]);
                 ^
/home/tong/ORB_SLAM/src/ORBextractor.cc:699:13: error: ‘KeyPointsFilter’ has not been declared
             KeyPointsFilter::retainBest(keypoints,nDesiredFeatures);
             ^
/home/tong/ORB_SLAM/src/ORBextractor.cc: In member function ‘void ORB_SLAM::ORBextractor::operator()(cv::InputArray, cv::InputArray, std::vector<cv::KeyPoint>&, cv::OutputArray)’:
/home/tong/ORB_SLAM/src/ORBextractor.cc:760:82: error: ‘GaussianBlur’ was not declared in this scope
     GaussianBlur(workingMat, workingMat, Size(7, 7), 2, 2, BORDER_REFLECT_101);
                                                                              ^
/home/tong/ORB_SLAM/src/ORBextractor.cc: In member function ‘void ORB_SLAM::ORBextractor::ComputePyramid(cv::Mat, cv::Mat)’:
/home/tong/ORB_SLAM/src/ORBextractor.cc:800:78: error: ‘INTER_LINEAR’ was not declared in this scope
    resize(mvImagePyramid[level-1], mvImagePyramid[level], sz, 0, 0, INTER_LINEA
                                                                     ^
/home/tong/ORB_SLAM/src/ORBextractor.cc:800:90: error: ‘resize’ was not declared in this scope
 resize(mvImagePyramid[level-1], mvImagePyramid[level], sz, 0, 0, INTER_LINEAR);
                                                                              ^
/home/tong/ORB_SLAM/src/ORBextractor.cc:803:80: error: ‘INTER_NEAREST’ was not declared in this scope
      resize(mvMaskPyramid[level-1], mvMaskPyramid[level], sz, 0, 0, INTER_NEARE
                                                                     ^
CMakeFiles/ORB_SLAM.dir/build.make:443: recipe for target 'CMakeFiles/ORB_SLAM.dir/src/ORBextractor.cc.o' failed
make[2]: *** [CMakeFiles/ORB_SLAM.dir/src/ORBextractor.cc.o] Error 1
CMakeFiles/Makefile2:227: recipe for target 'CMakeFiles/ORB_SLAM.dir/all' failed
make[1]: *** [CMakeFiles/ORB_SLAM.dir/all] Error 2
Makefile:127: recipe for target 'all' failed
make: *** [all] Error 2
{% endhighlight %}

Solution: （安装的opencv版本为2.4.13.6） 此错误出现的原因是一些函数没有找到，需要修改`/src/ORBextractor.cc`，在include处加上两行

{% highlight bash %}
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
{% endhighlight %}

## Error 2

{% highlight bash %}
/usr/include/eigen3/Eigen/src/Core/util/StaticAssert.h:119:9: error: ‘YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY’ is not a member of ‘Eigen::internal::static_assertion<false>’
         if (Eigen::internal::static_assertion<static_cast<bool>(CONDITION)>::MSG) {}
         ^
/usr/include/eigen3/Eigen/src/Core/util/XprHelper.h:707:3: note: in expansion of macro ‘EIGEN_STATIC_ASSERT’
   EIGEN_STATIC_ASSERT((internal::functor_is_product_like<BINOP>::ret \
   ^
/usr/include/eigen3/Eigen/src/Core/AssignEvaluator.h:745:3: note: in expansion of macro ‘EIGEN_CHECK_BINARY_COMPATIBILIY’
   EIGEN_CHECK_BINARY_COMPATIBILIY(Func,typename ActualDstTypeCleaned::Scalar,typename Src::Scalar);
   ^
CMakeFiles/ORB_SLAM.dir/build.make:1091: recipe for target 'CMakeFiles/ORB_SLAM.dir/src/Optimizer.cc.o' failed
make[2]: *** [CMakeFiles/ORB_SLAM.dir/src/Optimizer.cc.o] Error 1
CMakeFiles/Makefile2:324: recipe for target 'CMakeFiles/ORB_SLAM.dir/all' failed
make[1]: *** [CMakeFiles/ORB_SLAM.dir/all] Error 2
Makefile:127: recipe for target 'all' failed
make: *** [all] Error 2
{% endhighlight %}

Solution: 第二个错误的原因就像这个[回答][stackoverflow-eigen3]里说的,需要下载一个旧版本的Eigen3，这里推荐libeigen3-dev_3.2.0-8。 下载前可以参考这篇[博客][blog-eigen3-uninstall]删除已经安装好的Eigen3。

## Error 3

{% highlight bash %}
/usr/bin/ld: CMakeFiles/ORB_SLAM.dir/src/main.cc.o: undefined reference to symbol '_ZN5boost6system15system_categoryEv'
/usr/lib/x86_64-linux-gnu/libboost_system.so: error adding symbols: DSO missing from command line
collect2: error: ld returned 1 exit status
CMakeFiles/ORB_SLAM.dir/build.make:1609: recipe for target '../bin/ORB_SLAM' failed
make[2]: *** [../bin/ORB_SLAM] Error 1
CMakeFiles/Makefile2:324: recipe for target 'CMakeFiles/ORB_SLAM.dir/all' failed
make[1]: *** [CMakeFiles/ORB_SLAM.dir/all] Error 2
Makefile:127: recipe for target 'all' failed
make: *** [all] Error 2
{% endhighlight %}

Solution: 参考这个[issue][github-bug-lboost_system]，修改`CMakeLists.txt`， 添加`-lboost_system`。

{% highlight bash %}
target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
-lboost_system
)
{% endhighlight %}


## Error 4

{% highlight bash %}
ORB-SLAM Copyright (C) 2014 Raul Mur-Artal
This program comes with ABSOLUTELY NO WARRANTY;
This is free software, and you are welcome to redistribute it
under certain conditions. See LICENSE.txt.
[ERROR] [1540478413.140194980]: Wrong path to settings. Path must be absolut or relative to ORB_SLAM package directory.
{% endhighlight %}

Solution: 打开`~/.bashrc`, 在最后添加一行
{% highlight bash %}
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/tong/ORB_SLAM
{% endhighlight %}

并且运行的时候使用相对路径，比如

{% highlight bash %}
rosrun ORB_SLAM ORB_SLAM /Data/ORBvoc.txt /Data/Settings.yaml 
{% endhighlight %}

[github-orb-slam]: https://github.com/raulmur/ORB_SLAM
[stackoverflow-eigen3]: https://stackoverflow.com/questions/38647114/orb-slam-installation-on-ubuntu-xenial-16-04
[blog-eigen3-uninstall]: https://blog.csdn.net/j_____j/article/details/80622570
[github-bug-lboost_system]: https://github.com/raulmur/ORB_SLAM2/issues/535
