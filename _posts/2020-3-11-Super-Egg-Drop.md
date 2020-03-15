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

#### Naive Solution (TLE)

假设`moves[i][j]`代表`i`个鸡蛋和`j`层楼的最小移动数，那么如果我们把这第`i`个鸡蛋在第`k`层扔下，那么
    - 如果鸡蛋碎了，那么`floor[i][j] = 1 + floor[i - 1][k - 1]`.
    - 如果鸡蛋没碎，那么`floor[i][j] = 1 + floor[i][j - k]`

```c++
class Solution {
 public:
  int superEggDrop(int K, int N) {
    // moves[i][j]: min moves when i eggs and j floors
    vector<vector<int>> moves(K + 1, vector<int>(N + 1, 0));
    for (int j = 1; j <= N; ++j) {
      moves[1][j] = j;
    }
    for (int i = 1; i <= K; ++i) {
      moves[i][1] = 1;
    }

    for (int i = 2; i <= K; ++i) {
      for (int j = 2; j <= N; ++j) {
        moves[i][j] = moves[i - 1][j];
        for (int k = 1; k <= j; ++k) {
          moves[i][j] = std::min(
              moves[i][j], 1 + std::max(moves[i - 1][k - 1], moves[i][j - k]));
        }
      }
    }

    return moves[K][N];
  }
};

```

#### Solution

`floors[i][j]`代表`i`个鸡蛋和`j`次移动，能检测的最多楼层数。所以，状态转移方程是`floors[i][j] = floors[i - 1][j - 1] + floors[i - 1][j] + 1`. 这意味着当我们移动一次时，
    - 如果鸡蛋碎了，我们就可以检验`floors[i - 1][j - 1]`层。
    - 如果鸡蛋没碎，我们就可以检测`floors[i][j - 1]`层.

Language: **C++**

```c++
class Solution {
 public:
  int superEggDrop(int K, int N) {
    vector<int> floors(K + 1, 0);
    int m = 0;
    while (floors[K] < N) {
      for (int i = K; i > 0; --i) {
        floors[i] += floors[i - 1] + 1;
      }
      ++m;
    }
    return m;
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

### [飞机最低可俯冲高度](https://www.nowcoder.com/questionTerminal/19a79b34213649ab97eb721675750d1a)

近日，埃航空难的新闻牵动了无数人的心。据悉，空难很可能是由于波音737MAX飞机的失速保护系统错误触发所致。在飞机进行高空飞行时，驾驶辅助系统如果检测到飞机失速，无法维持足够的飞行升力，会压低机头进行俯冲，以重新获得速度，进而获取足够的飞行升力，维持飞行高度。但是在飞机进行低空飞行时，触发俯冲机制极有可能在飞机还未获得足够飞行速度并上升之前已经撞击地面。鉴于半年内的两起事故，波音公司决定在低于一定高度时屏蔽自动俯冲机制，现提供K架飞机用于测试最低可俯冲高度，设定需要测试的海拔范围为1~H（单位米），请问最不理想情况下，至少需要多少次才能求出飞机的最低可俯冲高度？

**输入描述:**
```bash
输入为整数K, H，用空格分隔

K代表用于测试的飞机数量，H代表需要测试的高度范围为1~H米（包含H）
```

**输出描述:**
```bash
输出整数N，代表最坏情况下需要测试的次数
```

**示例1**
```bash
输入
1 1000

输出
1000

说明
只有一架飞机用来测试的情况下，从最高高度1000米，逐次减1m进行测试，直到飞机坠毁。
```


**示例2**
```bash
输入
15 1000

输出
10

说明
飞机数量足够多，每次均使用二分法进行测试
```

**备注:**
```bash
1-H为低空飞行高度范围，所有大于H的高度都不属于低空飞行，不会在俯冲过程中撞击地面，不需要进行测试。

如果飞机俯冲测试过程中撞击地面坠毁，可以推断本次测试高度低于飞机实际最低可俯冲高度，可测试飞机数量减1。

如果飞机俯冲测试过程中撞击地面前顺利拉升，可以推断本次测试高度高于或等于飞机最低可俯冲高度，本次试验所用飞机可继续用来测试。

tips:
测试飞机中使用无人驾驶远程操控，无需担心飞行员安全。
无需考虑物理学上的合理性。
```

#### Solution

```c++
#include <iostream>
#include <vector>

using namespace std;

int main() {
    int K, H;
    cin >> K >> H;
    // floors[i][j]: max floors when i eggs
    vector<int> floors(K + 1, 0);
    int m = 0;
    while (floors[K] < H) {
        for (int i = K; i > 0; --i) {
            floors[i] += floors[i - 1] + 1;
        }
        ++m;
    }

    cout << m << endl;
    return 0;
}
```
