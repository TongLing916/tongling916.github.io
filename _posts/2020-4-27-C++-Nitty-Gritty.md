---
layout:     post
title:      "C++ - Nitty Gritty"
date:       2020-4-27
author:     Tong
catalog: true
tags:
    - Language
---

### [numeric_limits](https://en.cppreference.com/w/cpp/types/numeric_limits/min)

1. `std::numeric_limits<float>::min()` returns the minimum positive normalized value. To find the value that has no values less than it, use `std::numeric_limits<float>::lowest()`. 

