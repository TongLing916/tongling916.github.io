---
layout:     post
title:      "Bugs in ORB SLAM2"
date:       2018-12-09
author:     Tong
catalog: true
tags:
    - SLAM
---

> The issue about Bug 1 and 2 can be found in this [website][github-bug-1-2].

### Bug 1: Initializer.cc at about line 890 (?)

Report:
1. for complex scene like kitti sequence 00, fixing this bug causes more time for initialization.
2. for simple scene like rgbd_dataset_freiburg1_xyz and summit dataset, fixing this bug makes a quicker initialization possible.
3. TO-DO: IS THIS REALLY A BUG???

```cpp
// nGood++;

if(cosParallax<0.99998){
  vbGood[vMatches12[i].first]=true;
  nGood++;        // here is the change

}
```


### Bug 2: ORBmatcher.cc at about line 485

> A tutorial about Histogram of gradients is found in this [website][web-hog]

```cpp
float rot = F1.mvKeysUn[i1].angle-F2.mvKeysUn[bestIdx2].angle;
if(rot<0.0)
    rot+=360.0f;
    // int bin = round(rot*factor);

       int bin = round(rot/(360.0f*factor));  // here is the change

if(bin==HISTO_LENGTH)
    bin=0;
```

Because HISTO_LENGTH=30, we get 30 bin-Histogram (each bin is for a range of 360/30=12 degrees). To find the index for each rot, we need to calculate round(rot/12), which means round(rot/(360/HISTO_LENGTH)).

[web-hog]: https://www.learnopencv.com/histogram-of-oriented-gradients/
[github-bug-1-2]: https://github.com/raulmur/ORB_SLAM2/issues/59
