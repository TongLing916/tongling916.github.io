---
layout:     post
title:      "Test Examples"
date:       2018-12-08
author:     Tong
catalog: true
tags:
    - SLAM
---

### ORB-SLAM

> cd /home/tong/Applications/ORB_SLAM

1. ExampleGroovyOrNewer.launch文件的内容
{% highlight bash %}
<launch>

	<node pkg="image_view" type="image_view" name="image_view" respawn="false" output="log">
		<remap from="/image" to="/ORB_SLAM/Frame" />
		<param name="autosize" value="true"/>
	</node>

	<node pkg="rviz" type="rviz" name="rviz" args="-d $(find ORB_SLAM)/Data/rviz.rviz" output="log">
	</node>

 	<node pkg="ORB_SLAM" type="ORB_SLAM" name="ORB_SLAM"  args="Data/ORBvoc.txt Data/Settings.yaml" cwd="node" output="screen">
   </node>

</launch>
{% endhighlight %}

2. 打开一个terminal，输入
{% highlight bash %}
roslaunch /home/tong/ORB_SLAM/ExampleGroovyOrNewer.launch
{% endhighlight %}

3. 再打开一个terminal，输入
{% highlight bash %}
rosbag play --pause /home/tong/Datasets/ORB_SLAM_Example/Example.bag
{% endhighlight %}

（其中，Example.bag下载地址为 http://webdiis.unizar.es/~raulmur/orbslam/downloads/Example.bag.tar.gz）



### ORB-SLAM 2

> cd /home/tong/Applications/ORB_SLAM2

#### [TUM Dataset][dataset-tum]
{% highlight bash %}
./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml /home/tong/Datasets/tum/rgbd/Testing\ and\ Debugging/rgbd_dataset_freiburg1_xyz/
{% endhighlight %}

#### [KITTI Dataset][dataset-kitti]
{% highlight bash %}
./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/KITTI00-02.yaml /home/tong/Datasets/kitti/sequences/00/
{% endhighlight %}

#### [Summit Dataset][dataset-summit]
{% highlight bash %}
./Examples/Monocular/mono_summit Vocabulary/ORBvoc.txt Examples/Monocular/SUMMIT.yaml /home/tong/Datasets/summit/2018_12_03/axis_camera/
{% endhighlight %}

### DSO

> cd /home/tong/Applications/dso/build

#### [TUM Dataset][dataset-tum]
{% highlight bash %}
bin/dso_dataset \
files=/home/tong/Datasets/tum/mono/sequence_01/images.zip \
calib=/home/tong/Datasets/tum/mono/sequence_01/camera.txt \
gamma=/home/tong/Datasets/tum/mono/sequence_01/pcalib.txt \
vignette=/home/tong/Datasets/tum/mono/sequence_01/vignette.png \
preset=0 \
mode=0
{% endhighlight %}

### LDSO

> cd /home/tong/Applications/LDSO

#### [TUM Dataset][dataset-tum]
{% highlight bash %}
./bin/run_dso_tum_mono \
preset=0 \
files=/home/tong/Datasets/tum/mono/sequence_34/images.zip \
vignette=/home/tong/Datasets/tum/mono/sequence_34/vignette.png \
calib=/home/tong/Datasets/tum/mono/sequence_34/camera.txt \
gamma=/home/tong/Datasets/tum/mono/sequence_34/pcalib.txt
{% endhighlight %}

#### [KITTI Dataset][dataset-kitti]
{% highlight bash %}
./bin/run_dso_kitti preset=0 files=/home/tong/Datasets/kitti/sequences/00/ calib=./examples/Kitti/Kitti00-02.txt
{% endhighlight %}

#### [EuRoC Dataset][dataset-euroc]
{% highlight bash %}
./bin/run_dso_euroc preset=0 files=/home/tong/Datasets/euroc/MH_01_easy/mav0/cam0/
{% endhighlight %}

#### [Summit Dataset][dataset-summit]
{% highlight bash %}
./bin/run_dso_summit preset=0 files=/home/tong/Datasets/summit/2018_12_03/axis_camera/ calib=./examples/Summit/axis_camera.txt
{% endhighlight %}

### [Code for Monocular Visual Odometry Dataset][github-mono-tum]

#### playbackDataset: read images, photometric undistortion & rectification
{% highlight bash %}
cd ~/Desktop/LearningSLAM/test_dataset/sequence_01
../../../../Applications/mono_dataset_code/build/bin/playDataset ./
{% endhighlight %}

#### responseCalib: calibrate response function
{% highlight bash %}
cd ~/Desktop/LearningSLAM/test_dataset/kitti_00
../../../../Applications/mono_dataset_code/build/bin/responseCalib ./
{% endhighlight %}

#### vignetteCalib: calibrate vignette
{% highlight bash %}
cd ~/Desktop/LearningSLAM/test_dataset/kitti_00
../../../../Applications/mono_dataset_code/build/bin/vignetteCalib ./
{% endhighlight %}

### [evo][github-evo]
{% highlight bash %}
evo_traj kitti 00.txt --plot --plot_mode=xz
evo_traj tum KeyFrameTrajectory.txt --plot --plot_mode=xz
{% endhighlight %}

[dataset-tum]: https://vision.in.tum.de/data/datasets/rgbd-dataset/download
[dataset-kitti]: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
[dataset-euroc]: https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
[github-mono-tum]: https://github.com/tum-vision/mono_dataset_code
[dataset-summit]: http://lingtong.de/2018/12/08/ROS-Summit-Commands/
[github-evo]: https://github.com/MichaelGrupp/evo
