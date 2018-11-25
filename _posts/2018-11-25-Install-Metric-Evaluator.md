---
layout:     post
title:      "Install Metric Evaluator"
date:       2018-11-25
author:     Tong
catalog: true
tags:
    - Installation
---

进入[metricEvaluator][freiburg-metricEvaluator]文件夹，直接运行make,碰到下面的问题

## Error 1

{% highlight bash %}
linalg/runtimeError.cpp: In constructor ‘RuntimeError::RuntimeError(const char*, ...)’:
linalg/runtimeError.cpp:49:43: error: ‘vasprintf’ was not declared in this scope
   int b = vasprintf(&auxPtr, fmt, arg_list);
{% endhighlight %}

Solution: 打开```linalg/runtimeError.cpp```，加入下面头文件

{% highlight bash %}
#include <stdarg.h>
#include <stdio.h>
{% endhighlight %}


[freiburg-metricEvaluator]: http://ais.informatik.uni-freiburg.de/slamevaluation/software.php
