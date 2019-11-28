---
layout:     post
title:      "C++: Error with multiple definitions of function"
date:       2019-11-28
author:     Tong
catalog: true
tags:
    - Language
---

## Summary

### 原因

所有的declaration和definition都写在了.h里面


### 解决办法

1.	Inline
2.	CC
3.	static
4.	namespace {}
5.	把那些调用它的全归到一个文件里面
