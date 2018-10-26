---
layout:     post
title:      "Keypoint and Descriptor"
date:       2018-10-27
author:     Tong
catalog: true
tags:
    - SLAM
---
## [FAST][paper-FAST]
<br> You can find a short description of this description on the [opencv tutorial][opencv-FAST].

## [BRIEF][paper-BRIEF]
The BRIEF descriptor is a binary vector where each bit is the result of an intesity comparison between a given pair of pixels around the keypoint. They are hardly invariant to scale and rotation.
<br> You can find a short description of this description on the [opencv tutorial][opencv-BRIEF].

[paper-FAST]: https://www.edwardrosten.com/work/rosten_2006_machine.pdf
[opencv-FAST]: https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_feature2d/py_fast/py_fast.html
[paper-BRIEF]: https://www.cs.ubc.ca/~lowe/525/papers/calonder_eccv10.pdf
[opencv-BRIEF]: https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_feature2d/py_brief/py_brief.html