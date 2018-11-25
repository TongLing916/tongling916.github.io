---
layout:     post
title:      "Install Code for TUM Monocular Visual Odometry Dataset"
date:       2018-11-25
author:     Tong
catalog: true
tags:
    - Installation
---
> [source code][tum-code]

在安装thirdparty/aruco-1.3.0,碰到下面的问题

## Error 1

{% highlight bash %}
/home/tong/Applications/mono_dataset_code/thirdparty/aruco-1.3.0/utils/aruco_test_board_gl_mask.cpp:171:3: error: ‘rectangle’ is not a member of ‘cv’
   cv::rectangle(mask,cv::Rect(img.cols/4, img.rows/4, img.cols/2, img.rows/2),

/home/tong/Applications/mono_dataset_code/thirdparty/aruco-1.3.0/utils/aruco_test_board_gl.cpp: In function ‘void vIdle()’:
/home/tong/Applications/mono_dataset_code/thirdparty/aruco-1.3.0/utils/aruco_test_board_gl.cpp:273:9: error: ‘cvtColor’ is not a member of ‘cv’
   cv::cvtColor(TheInputImage,TheInputImage,CV_BGR2RGB);
{% endhighlight %}

Solution: 这主要是使用的opencv版本不同带来的问题，打开```aruco_test_board_gl_mask.cpp.cpp```和```utilsaruco_test_board_gl.cpp.cpp```，加入下面头文件

{% highlight bash %}
#include "opencv2/imgproc.hpp"
{% endhighlight %}


[tum-code]: https://github.com/tum-vision/mono_dataset_code
