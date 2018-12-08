---
layout:     post
title:      "Bugs in ORB SLAM2"
date:       2018-12-09
author:     Tong
catalog: true
tags:
    - SLAM
---

### Bug 1: Initializer.cc at about line 890

```cpp
// nGood++;

if(cosParallax<0.99998){
  vbGood[vMatches12[i].first]=true;
  nGood++;        // here is the change

}
```


### Bug 2: ORBmatcher.cc at about line 485

```cpp
float rot = F1.mvKeysUn[i1].angle-F2.mvKeysUn[bestIdx2].angle;
if(rot<0.0)
    rot+=360.0f;
    // int bin = round(rot*factor);

       int bin = round(rot/(360.0f*factor));  // here is the change

if(bin==HISTO_LENGTH)
    bin=0;
```
