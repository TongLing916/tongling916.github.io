---
layout:     post
title:      "Install ORB-SLAM 2"
date:       2018-10-27
author:     Tong
catalog: true
tags:
    - Installation
---

今天根据[ORB-SLAM 2][github-orb-slam-2]上面的提示，在VMWare中的Ubuntu16.04上安装了ORB-SLAM 2，下面是安装过程中遇到的一些问题。

## Error 1

第一个错误出现在安装[Pangolin][github-pangolin]时。

{% highlight bash %}
Build type not set (defaults to release)
-DCMAKE_BUILD_TYPE=Debug for debug
CMake Error at CMakeModules/FindGLEW.cmake:51 (MESSAGE):
  Could not find GLEW
Call Stack (most recent call first):
  src/CMakeLists.txt:160 (find_package)
{% endhighlight %}

Solution: 根据[issue][issue-glew]，只需要先运行`sudo apt-get install libglew-dev`就行。

{% highlight bash %}

{% endhighlight %}

## Error 2

发生在安装ORB-SLAM 2时，运行`./build.sh`，结果出现错误

{% highlight bash %}
virtual memory exhausted: Cannot allocate memory
{% endhighlight %}

Solution: 根据这个[issue][issue-swap], 了解到要扩大swap，具体操作可以参考这个[教程][create-swap]。如果swap正在被占用，可以先运行`sudo swapoff -a`来关闭swap，再按照教程进行修改。 (最后设置的count=12000)


[github-orb-slam-2]: https://github.com/raulmur/ORB_SLAM2
[github-pangolin]: https://github.com/stevenlovegrove/Pangolin
[issue-glew]: https://github.com/CPFL/Autoware/issues/578
[issue-swap]: https://github.com/Project-OSRM/osrm-backend/issues/1170
[create-swap]: https://digitizor.com/create-swap-file-ubuntu-linux/
