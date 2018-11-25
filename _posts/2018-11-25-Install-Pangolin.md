---
layout:     post
title:      "Install Pangolin"
date:       2018-11-25
author:     Tong
catalog: true
tags:
    - Installation
---

今天不小心把安装好的Pangolin删掉了，而且还把ORB-SLAM2移到了别的文件夹，导致需要重新编译pangolin和ORB-SLAM2。Pangolin的下载地址是这个[网站][github-pangolin]。下面是

## Error 1

{% highlight bash %}
/usr/local/include/TooN/internal/allocator.hh:68:19: error: ‘debug_initialize’ was not declared in this scope, and no declarations were found by argument-dependent lookup at the point of instantiation [-fpermissive] 
	debug_initialize(my_data, Size); 
{% endhighlight %}

{% highlight bash %}
/usr/local/include/TooN/internal/operators.hh:54:83: error: expected ‘)’ before ‘(’ token
{% endhighlight %}

Solution: 根据这个[issue][issue-pangolin]，了解到可能是TooN出了问题，所以我们要把pangolin文件夹下的src/CMakeLists.txt文件进行相应修改，即注释到一些代码。之后就正常编译就行。

{% highlight bash %}
#option(BUILD_PANGOLIN_TOON "Build support for TooN matrix types" ON)
#if(BUILD_PANGOLIN_TOON)
#  find_package(TooN QUIET)
#  if(TooN_FOUND)
#    set(HAVE_TOON 1)
#    list(APPEND USER_INC  ${TooN_INCLUDE_DIR} )
#    message(STATUS "TooN Found and Enabled")
#  endif()
#endif()
{% endhighlight %}


[github-pangolin]: https://github.com/stevenlovegrove/Pangolin
[issue-pangolin]: https://github.com/stevenlovegrove/Pangolin/issues/287
