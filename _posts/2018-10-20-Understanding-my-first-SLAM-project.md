---
layout: post
title: "Understanding My First SLAM Project"
date:       2018-10-20
author:     Tong
catalog: true
tags:
    - SLAM
---

最近看完了《视觉SLAM十四讲》，感觉这本书对入门SLAM还是有一定的帮助，
但感觉里面内容实在太多，而且自己的论文可能也只涉及到ORB-SLAM和DSO。所以，自己并不想话太多时间一步步推导以及把各种论文读一遍，
导致并没有深刻理解里面的一些概念。为了更加理解Visual SLAM的应用原理，
我会好好研究《Multiple View Geometry for Robotics》，
相信历史遗留问题还是会慢慢解决的。最重要的，我个人偏向于通过写代码了解某样东西，
所以今天在网上找来了一个小程序，就当打响研究SLAM的第一枪吧。

该项目[MonoVO][monovo-python]严格来说并不是一个完整的SLAM，它只是一个一个Mono Odometry,
不包含Loop Closure等，为了便于理解，我特意先研究了python的代码，
类似的一种C++的实现方式也可以在[这个网址][monovo-c++]找到。 
其中，所有程序都在Ubuntu 16.04 下运行。

# Requirements
## Libraries
为了运行这个python程序，要求都栽Readme中可以看到，要注意的是，如果电脑同时装了ROS和多个版本的Python，
直接运行`python test.py`可能会报出现下面的错误：
{% highlight bash %}
ImportError: /opt/ros/kinetic/lib/python2.7/dist-packages/cv2.so: undefined symbol: PyCObject_Type
{% endhighlight %}

根据[这个解答][pycobject]，可以知道，是python版本的问题，可以根据解答做相应的改动
或者运行`python2.7 test.py`就没问题了。

## Datasets
要运行这个程序必须有相应的数据，在[KITTI][kitti]上，可以下载`odometry_data_set(grayscale)`
和`odometry_ground_truth_poses`。
本人把那两个解压后的文件存放在/home/XXX/datasets/KITTI下面，便于管理，
其中XXX是用户名。

再稍微解释一下两个压缩包，其中`odometry_data_set(grayscale)`里面是各种录制好的图片，
还有`calib.txt`和`times.txt`，里面是相机的参数和每幅图的时间。
而`odometry_ground_truth_poses`解压后是一堆关于<b>poses</b>的<b>txt</b>文件，代表了图片
拍摄时的真实位置，通过我们运行程序后得到的轨迹和真是轨迹做对比，我们便能评价monoVO的好坏了。

## Codes
代码里面也要做相应修改。其中`test.py`中作如下改动：

{% highlight python%}
#some codes...
vo = VisualOdometry(cam, '/home/tong/datasets/KITTI/poses/00.txt')
#some codes...
for img_id in xrange(4541):
	img = cv2.imread('/home/tong/datasets/KITTI/sequences/00/image_0/'+str(img_id).zfill(6)+'.png', 0)
#some codes...
{% endhighlight %}

其中文件目录要根据自己电脑内的存储路径做相应改动。

# Run
![Image of monoVO trajectory][monovo-trajectory]

实际运行过程中，发现轨迹的转向与实际视频中的转向正好相反，这是为什么？

# Discussion
要想搞清楚，我们先来一步一步分析代码。


[monovo-python]: https://github.com/uoip/monoVO-python
[monovo-c++]: https://github.com/avisingh599/mono-vo
[pycobject]: https://stackoverflow.com/questions/43019951/after-install-ros-kinetic-cannot-import-opencv
[kitti]: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
[monovo-trajectory]: https://raw.githubusercontent.com/uoip/monoVO-python/master/map.png
















