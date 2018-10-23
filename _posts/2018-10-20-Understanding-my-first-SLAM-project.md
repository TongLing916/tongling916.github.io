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
要想搞清楚，我们先来一步一步分析代码。首先来看`visual_odometry.py`.

{% highlight python%}
#some codes...
import numpy as np 
import cv2

STAGE_FIRST_FRAME = 0
STAGE_SECOND_FRAME = 1
STAGE_DEFAULT_FRAME = 2
kMinNumFeature = 1500

lk_params = dict(winSize  = (21, 21), 
				#maxLevel = 3,
             	criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

def featureTracking(image_ref, image_cur, px_ref):
	#https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_video/py_lucas_kanade/py_lucas_kanade.html
	#<<视觉SLAM十四讲>> P.187-188
	#learning opencv 3, P507
	#kp2: next points(2d end points, CV_32F)
	#st: status, for each points, found = 1, else = 0
	#err: Error measure for found points
	kp2, st, err = cv2.calcOpticalFlowPyrLK(image_ref, image_cur, px_ref, None, **lk_params)  #shape: [k,2] [k,1] [k,1]

	st = st.reshape(st.shape[0])
	kp1 = px_ref[st == 1]
	kp2 = kp2[st == 1]

	return kp1, kp2


class PinholeCamera:
	def __init__(self, width, height, fx, fy, cx, cy, 
				k1=0.0, k2=0.0, p1=0.0, p2=0.0, k3=0.0):
		self.width = width
		self.height = height
		self.fx = fx
		self.fy = fy
		self.cx = cx
		self.cy = cy
		self.distortion = (abs(k1) > 0.0000001)
		self.d = [k1, k2, p1, p2, k3]


class VisualOdometry:
	def __init__(self, cam, annotations):
		self.frame_stage = 0
		self.cam = cam
		self.new_frame = None
		self.last_frame = None
		self.cur_R = None
		self.cur_t = None
		self.px_ref = None
		self.px_cur = None
		self.focal = cam.fx
		self.pp = (cam.cx, cam.cy)   #Principle point of the camera. 光圈中心
		self.trueX, self.trueY, self.trueZ = 0, 0, 0
		
		#the basics of FAST algorithm: https://docs.opencv.org/3.4.3/df/d0c/tutorial_py_fast.html
		#nonmaxSuppression 设置成true，来解决多个feature在相邻的位置被检测到
		#https://blog.csdn.net/tengfei461807914/article/details/79492012
		#opencv-python-tutroals P.156
		self.detector = cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True)
		with open(annotations) as f:
			self.annotations = f.readlines()

	def getAbsoluteScale(self, frame_id):  #specialized for KITTI odometry dataset
		ss = self.annotations[frame_id-1].strip().split()
		x_prev = float(ss[3])
		y_prev = float(ss[7])
		z_prev = float(ss[11])
		ss = self.annotations[frame_id].strip().split()
		x = float(ss[3])
		y = float(ss[7])
		z = float(ss[11])
		self.trueX, self.trueY, self.trueZ = x, y, z
		return np.sqrt((x - x_prev)*(x - x_prev) + (y - y_prev)*(y - y_prev) + (z - z_prev)*(z - z_prev))

	def processFirstFrame(self):
		self.px_ref = self.detector.detect(self.new_frame)
		self.px_ref = np.array([x.pt for x in self.px_ref], dtype=np.float32)
		self.frame_stage = STAGE_SECOND_FRAME

	def processSecondFrame(self):
		self.px_ref, self.px_cur = featureTracking(self.last_frame, self.new_frame, self.px_ref)
		E, mask = cv2.findEssentialMat(self.px_cur, self.px_ref, focal=self.focal, pp=self.pp, method=cv2.RANSAC, prob=0.999, threshold=1.0)
		_, self.cur_R, self.cur_t, mask = cv2.recoverPose(E, self.px_cur, self.px_ref, focal=self.focal, pp = self.pp)
		self.frame_stage = STAGE_DEFAULT_FRAME 
		self.px_ref = self.px_cur

	def processFrame(self, frame_id):
		self.px_ref, self.px_cur = featureTracking(self.last_frame, self.new_frame, self.px_ref)
		#Essential Matrix, fundamental matrix. See <<Visual SLAM 14 Lectures>> P.145
		E, mask = cv2.findEssentialMat(self.px_cur, self.px_ref, focal=self.focal, pp=self.pp, method=cv2.RANSAC, prob=0.999, threshold=1.0)
		
		#https://docs.opencv.org/3.0-beta/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#nister03
		_, R, t, mask = cv2.recoverPose(E, self.px_cur, self.px_ref, focal=self.focal, pp = self.pp)
		absolute_scale = self.getAbsoluteScale(frame_id)
		if(absolute_scale > 0.1):
			self.cur_t = self.cur_t + absolute_scale*self.cur_R.dot(t) 
			self.cur_R = R.dot(self.cur_R)
		if(self.px_ref.shape[0] < kMinNumFeature):      #为什么小于某个值后更新px_cur?
			self.px_cur = self.detector.detect(self.new_frame)
			self.px_cur = np.array([x.pt for x in self.px_cur], dtype=np.float32)
		self.px_ref = self.px_cur

	def update(self, img, frame_id):
		#确认是grayscale，以及有相同的height(x方向，index0)和width(y方向，index1)
		assert(img.ndim==2 and img.shape[0]==self.cam.height and img.shape[1]==self.cam.width), "Frame: provided image has not the same size as the camera model or image is not grayscale"
		self.new_frame = img
		if(self.frame_stage == STAGE_DEFAULT_FRAME):
			self.processFrame(frame_id)
		elif(self.frame_stage == STAGE_SECOND_FRAME):
			self.processSecondFrame()
		elif(self.frame_stage == STAGE_FIRST_FRAME):
			self.processFirstFrame()
		self.last_frame = self.new_frame

{% endhighlight %}

然后是`test.py`.

{% highlight python%}
import numpy as np 
import cv2

from visual_odometry import PinholeCamera, VisualOdometry


cam = PinholeCamera(1241.0, 376.0, 718.8560, 718.8560, 607.1928, 185.2157)    #init a pinhole camera (width, height, fx, fy, cx, cy)
vo = VisualOdometry(cam, '/home/tong/datasets/KITTI/poses/00.txt')    #init a visual odometry (cam, ground truth poses) 

traj = np.zeros((600,600,3), dtype=np.uint8)

for img_id in xrange(4541):   #we have total 4541 images in image_0
	#Python zfill() 方法返回指定长度的字符串，原字符串右对齐，前面填充0。
	#cv.imread(path_to_image, flag_how_image_should_be_read) https://docs.opencv.org/3.1.0/dc/d2e/tutorial_py_image_display.html
	#https://docs.opencv.org/3.0-beta/modules/imgcodecs/doc/reading_and_writing_images.html
	#cv2.IMREAD_COLOR：    1, default
	#cv2.IMREAD_GRAYSCALE：0
	#cv2.IMREAD_UNCHANGED：-1, 包括alpha
	#The depth of the grayscale image is 1, but grayscale is actually composed of 2 dimentions: x and y. Color image composed of 3 dimentions. x, y, and depth of 3
	img = cv2.imread('/home/tong/datasets/KITTI/sequences/00/image_0/'+str(img_id).zfill(6)+'.png', 0)  

	vo.update(img, img_id)

	cur_t = vo.cur_t
	if(img_id > 2):
		x, y, z = cur_t[0], cur_t[1], cur_t[2]
	else:
		x, y, z = 0., 0., 0.
	draw_x, draw_y = int(x)+290, int(z)+90
	true_x, true_y = int(vo.trueX)+290, int(vo.trueZ)+90

	cv2.circle(traj, (draw_x,draw_y), 1, (img_id*255/4540,255-img_id*255/4540,0), 1)
	cv2.circle(traj, (true_x,true_y), 1, (0,0,255), 2)
	cv2.rectangle(traj, (10, 20), (600, 60), (0,0,0), -1)
	text = "Coordinates: x=%2fm y=%2fm z=%2fm"%(x,y,z)
	cv2.putText(traj, text, (20,40), cv2.FONT_HERSHEY_PLAIN, 1, (255,255,255), 1, 8)

	cv2.imshow('Road facing camera', img)
	cv2.imshow('Trajectory', traj)
	cv2.waitKey(1)

cv2.imwrite('map.png', traj)
{% endhighlight %}

[monovo-python]: https://github.com/uoip/monoVO-python
[monovo-c++]: https://github.com/avisingh599/mono-vo
[pycobject]: https://stackoverflow.com/questions/43019951/after-install-ros-kinetic-cannot-import-opencv
[kitti]: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
[monovo-trajectory]: https://raw.githubusercontent.com/uoip/monoVO-python/master/map.png
















