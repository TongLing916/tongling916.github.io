---
layout:     post
title:      "Commands for Summit"
date:       2018-12-03
author:     Tong
catalog: true
tags:
    - SLAM
---

> Images of /axis_camera/image_mono and /axis_camera/image_raw: 1280 x 720 <br>
> Images of /orbbec_astra/rgb/image_raw /orbbec_astra/rgb/image_rect_color: 640 x 480

今天尝试了通过[Summit机器人][page-summit]获取图像，以便之后建立自己的datasets，用来测试ORB-SLAM2和LDSO。下面是用到的一些指令：

### Rosbag

#### Record Bag File

{% highlight bash %}
rosbag record -O test.bag /axis_camera/image_mono /axis_camera/image_raw /orbbec_astra/rgb/image_raw /orbbec_astra/rgb/image_rect_color
{% endhighlight %}


#### Play Bag File

{% highlight bash %}
#play a loop of the bag file  
rosbag play -l /home/tong/Datasets/summit/2018_12_03/robolab_2018_12_3.bag
#no loop
rosbag play /home/tong/Datasets/summit/2018_12_03/robolab_2018_12_3.bag
{% endhighlight %}


#### Extract Images From a Bag File

{% highlight bash %}
#enter the folder where you want to save the images
#open a terminal to run the expected bag file
rosrun image_view extract_images _sec_per_frame:=0.01 image:=/axis_camera/image_mono
rosrun image_view extract_images _sec_per_frame:=0.01 image:=/axis_camera/image_raw
rosrun image_view extract_images _sec_per_frame:=0.01 image:=/orbbec_astra/rgb/image_raw
rosrun image_view extract_images _sec_per_frame:=0.01 image:=/orbbec_astra/rgb/image_rect_color
{% endhighlight %}

### Obtain Timestamps from Headers

#### axis_mono.py

{% highlight python %}
#!/usr/bin/env python

from __future__ import division   #avoid rounding when doing integer/integer
import rospy
import sys
from sensor_msgs.msg import Image

start_time = 0.0
date = ""

def callback(img):
	global start_time
	#rospy.loginfo(str(img.header.stamp.to_nsec()))   #header.stamp unit: nanosecond
	if start_time == 0.0:
		start_time = img.header.stamp.to_nsec()
		with open("/home/tong/Datasets/summit/"+ date +"/axis_camera/mono/times.txt", "a") as f:
			f.write("0.000000000") 	
	else:
		temp_time = (img.header.stamp.to_nsec() - start_time)/1000000000
		#append text at the end of the file
		with open("/home/tong/Datasets/summit/"+ date +"/axis_camera/mono/times.txt", "a") as f:
			f.write("\n" + str(temp_time)) 	

def timestamp_extractor():
	rospy.init_node("axis_mono_extractor", anonymous=True)
	img_sub = rospy.Subscriber("/axis_camera/image_mono", Image, callback)
	rospy.spin()

if __name__ == "__main__":
	date = sys.argv[1]
	timestamp_extractor()
{% endhighlight %}

#### axis_raw.py

{% highlight python %}
#!/usr/bin/env python

from __future__ import division   #avoid rounding when doing integer/integer
import rospy
import sys
from sensor_msgs.msg import Image

start_time = 0.0
date = ""

def callback(img):
	global start_time
	#rospy.loginfo(str(img.header.stamp.to_nsec()))   #header.stamp unit: nanosecond
	if start_time == 0.0:
		start_time = img.header.stamp.to_nsec()
		with open("/home/tong/Datasets/summit/"+ date +"/axis_camera/raw/times.txt", "a") as f:
			f.write("0.000000000") 	
	else:
		temp_time = (img.header.stamp.to_nsec() - start_time)/1000000000
		#append text at the end of the file
		with open("/home/tong/Datasets/summit/"+ date +"/axis_camera/raw/times.txt", "a") as f:
			f.write("\n" + str(temp_time)) 	

def timestamp_extractor():
	rospy.init_node("axis_raw_extractor", anonymous=True)
	img_sub = rospy.Subscriber("/axis_camera/image_raw", Image, callback)
	rospy.spin()

if __name__ == "__main__":
	date = sys.argv[1]
	timestamp_extractor()
{% endhighlight %}

#### orbbec_raw.py

{% highlight python %}
#!/usr/bin/env python

from __future__ import division   #avoid rounding when doing integer/integer
import rospy
import sys
from sensor_msgs.msg import Image

start_time = 0.0
date = ""

def callback(img):
	global start_time
	#rospy.loginfo(str(img.header.stamp.to_nsec()))   #header.stamp unit: nanosecond
	if start_time == 0.0:
		start_time = img.header.stamp.to_nsec()
		with open("/home/tong/Datasets/summit/"+ date +"/orbbec_astra/raw/times.txt", "a") as f:
			f.write("0.000000000") 	
	else:
		temp_time = (img.header.stamp.to_nsec() - start_time)/1000000000
		#append text at the end of the file
		with open("/home/tong/Datasets/summit/"+ date +"/orbbec_astra/raw/times.txt", "a") as f:
			f.write("\n" + str(temp_time)) 	

def timestamp_extractor():
	rospy.init_node("orbbec_raw_extractor", anonymous=True)
	img_sub = rospy.Subscriber("/orbbec_astra/rgb/image_raw", Image, callback)
	rospy.spin()

if __name__ == "__main__":
	date = sys.argv[1]
	timestamp_extractor()
{% endhighlight %}


#### orbbec_rect.py

{% highlight python %}
#!/usr/bin/env python

from __future__ import division   #avoid rounding when doing integer/integer
import rospy
import sys
from sensor_msgs.msg import Image

start_time = 0.0
date = ""

def callback(img):
	global start_time
	#rospy.loginfo(str(img.header.stamp.to_nsec()))   #header.stamp unit: nanosecond
	if start_time == 0.0:
		start_time = img.header.stamp.to_nsec()
		with open("/home/tong/Datasets/summit/"+ date +"/orbbec_astra/rect_color/times.txt", "a") as f:
			f.write("0.000000000") 	
	else:
		temp_time = (img.header.stamp.to_nsec() - start_time)/1000000000
		#append text at the end of the file
		with open("/home/tong/Datasets/summit/"+ date +"/orbbec_astra/rect_color/times.txt", "a") as f:
			f.write("\n" + str(temp_time)) 	

def timestamp_extractor():
	rospy.init_node("orbbec_rect_extractor", anonymous=True)
	img_sub = rospy.Subscriber("/orbbec_astra/rgb/image_rect_color", Image, callback)
	rospy.spin()

if __name__ == "__main__":
	date = sys.argv[1]
	timestamp_extractor()
{% endhighlight %}

#### timestamp_extraction.launch
{% highlight bash %}
<launch>
	<arg name="arg_date"/>
	<node pkg="master_thesis" type="axis_mono.py" name="axis_mono_extractor" args="$(arg arg_date)"/>
	<node pkg="master_thesis" type="axis_raw.py" name="axis_raw_extractor" args="$(arg arg_date)"/>
	<node pkg="master_thesis" type="orbbec_raw.py" name="orbbec_raw_extractor" args="$(arg arg_date)"/>
	<node pkg="master_thesis" type="orbbec_rect.py" name="orbbec_rect_extractor" args="$(arg arg_date)"/>
</launch>
{% endhighlight %}

#### Test example

{% highlight bash %}
roslaunch master_thesis timestamp_extraction.launch arg_date:="2018_12_03"
{% endhighlight %}

### Calibration

#### Calibrate a Camera using OpenCV

{% highlight python %}
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import glob

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(4,8,0)
objp = np.zeros((4 * 8, 3), np.float32)   #for axis_camera: 4 vertices x 8 vertices
objp[:, :2] = np.mgrid[0:4, 0:8].T.reshape(-1, 2)
# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.
images = glob.glob("./data/axis_camera/frame*.jpg")
for fname in images:
	print(fname)
	img = cv2.imread(fname)   #default: BGR order
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
	ret, corners = cv2.findChessboardCorners(gray, (4, 8), None)
    # If found, add object points, image points (after refining them)
	if ret == True:
		objpoints.append(objp)
    	corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    	imgpoints.append(corners2)
        # Draw and display the corners
    	cv2.drawChessboardCorners(img, (4, 8), corners2, ret)
    	cv2.imshow('img', img)
    	cv2.waitKey(500)
cv2.destroyAllWindows()

#print(objpoints[0])
#print(imgpoints[0])

#let's calibrate the camera
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print("\n---------- camera matrix 3x3 ----------\n")
print(mtx)
print("\n---------- distortion coefficients k1 k2 p1 p2 k3----------\n")
print(dist)
print("\n---------- rotation vector ----------\n")
print(rvecs)
print("\n---------- translation vector ----------\n")
print(tvecs)
{% endhighlight %}


#### Calibrate a Camera using MATLAB

{% highlight bash %}

{% endhighlight %}

[page-summit]: https://www.robotnik.eu/mobile-robots/summit-xl-hl/
