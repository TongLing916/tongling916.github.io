---
layout:     post
title:      "Super Egg Drop"
date:       2020-3-11
author:     Tong
catalog: true
tags:
    - Algorithm
---

### [887\. Super Egg Drop](https://leetcode.com/problems/super-egg-drop/)

Difficulty: **Hard**


You are given `K` eggs, and you have access to a building with `N` floors from `1` to `N`. 

Each egg is identical in function, and if an egg breaks, you cannot drop it again.

You know that there exists a floor `F` with `0 <= F <= N` such that any egg dropped at a floor higher than `F` will break, and any egg dropped at or below floor `F` will not break.

Each _move_, you may take an egg (if you have an unbroken one) and drop it from any floor `X` (with `1 <= X <= N`). 

Your goal is to know **with certainty** what the value of `F` is.

What is the minimum number of moves that you need to know with certainty what `F` is, regardless of the initial value of `F`?


**Example 1:**

```
Input: K = 1, N = 2
Output: 2
Explanation:
Drop the egg from floor 1\.  If it breaks, we know with certainty that F = 0.
Otherwise, drop the egg from floor 2\.  If it breaks, we know with certainty that F = 1.
If it didn't break, then we know with certainty F = 2.
Hence, we needed 2 moves in the worst case to know what F is with certainty.
```


**Example 2:**

```
Input: K = 2, N = 6
Output: 3
```


**Example 3:**

```
Input: K = 3, N = 14
Output: 4
```

**Note:**

1.  `1 <= K <= 100`
2.  `1 <= N <= 10000`


#### Solution

Language: **C++**

```c++
class Solution {
public:
    int superEggDrop(int K, int N) {
        
    }
};
```

### [Please Do Break the Crystal](https://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-s095-programming-for-the-puzzled-january-iap-2018/puzzle-3-you-can-read-minds/)


#### Solution (Python)

```python
#Programming for the Puzzled -- Srini Devadas
#Please Do Break the Crystal
#This is an interactive procedure that given n floors and d balls determines
#what floors to drop the balls from to minimize the worst-case number of
#drops required to determine the hardness coefficient of the crystal.
#The hardness coefficient will range from 0 (breaks at Floor 1) or n (does not
#break at n.
def howHardIsTheCrystal(n, d):

    #First determine the radix r
    r = 1
    while (r**d <= n):
        r = r + 1
    print('Radix chosen is', r)

    numDrops = 0
    floorNoBreak = [0] * d
    for i in range(d):
        #Begin phase i
        for j in range(r-1):
            #increment ith digit of representation
            floorNoBreak[i] += 1
            Floor = convertToDecimal(r, d, floorNoBreak)
            #Make sure you aren't higher than the highest floor
            if Floor > n:
                floorNoBreak[i] -= 1
                break
            print ('Drop ball', i+1, 'from Floor', Floor)
            yes = input('Did the ball break (yes/no)?:')
            numDrops += 1
            if yes == 'yes':
                floorNoBreak[i] -= 1
                break


    hardness = convertToDecimal(r, d, floorNoBreak)
    print('Hardness coefficient is', hardness)
    print('Total number of drops is', numDrops)

    return

def convertToDecimal(r, d, rep):
    number = 0
    for i in range(d-1):
        number = (number + rep[i]) * r
    number += rep[d-1]

    return number
```

### [飞机最低可俯冲高度 ](https://www.acwing.com/problem/content/description/884/)
