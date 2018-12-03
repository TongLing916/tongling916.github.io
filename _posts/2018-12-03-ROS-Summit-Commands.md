---
layout:     post
title:      "Useful commands for Summit"
date:       2018-12-03
author:     Tong
catalog: true
tags:
    - SLAM
---

今天尝试了通过[Summit机器人][page-summit]获取图像，以便之后建立自己的datasets，用来测试ORB-SLAM2和LDSO。下面是用到的一些指令：

## Record Bag File 

{% highlight bash %}
rosbag record -O test.bag /axis_camera/image_mono /axis_camera/image_raw /orbbec_astra/rgb/image_raw /orbbec_astra/rgb/image_rect_color
{% endhighlight %}


## Play Bag File 

{% highlight bash %}
#play a loop of the bag file  
rosbag play -l /home/tong/Datasets/summit/2018_12_03/robolab_2018_12_3.bag
#no loop
rosbag play /home/tong/Datasets/summit/2018_12_03/robolab_2018_12_3.bag
{% endhighlight %}


## Extract Images From a Bag File 

{% highlight bash %}
#enter the folder where you want to save the images 
#open a terminal to run the expected bag file
rosrun image_view extract_images _sec_per_frame:=0.01 image:=/axis_camera/image_mono
rosrun image_view extract_images _sec_per_frame:=0.01 image:=/axis_camera/image_raw
rosrun image_view extract_images _sec_per_frame:=0.01 image:=/orbbec_astra/rgb/image_raw
rosrun image_view extract_images _sec_per_frame:=0.01 image:=/orbbec_astra/rgb/image_rect_color
{% endhighlight %}


## Calibrate a Camera using OpenCV

{% highlight bash %}

{% endhighlight %}


## Calibrate a Camera using MATLAB

{% highlight bash %}

{% endhighlight %}

[page-summit]: https://www.robotnik.eu/mobile-robots/summit-xl-hl/
