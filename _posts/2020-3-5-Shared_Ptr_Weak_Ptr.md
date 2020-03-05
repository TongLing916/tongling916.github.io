---
layout:     post
title:      "shared_ptr and weak_ptr"
date:       2020-2-22
author:     Tong
catalog: true
tags:
    - Language
---

### 循环引用

如果两个`class`中互相含有对方的`shared_ptr`，那么这两个类最后都不会删除，结果造成__内存泄漏__。

#### 问题代码
```c++
#include <iostream>
#include <memory>
#include <unordered_map>

using namespace std;

class B;

class A {
 public:
  A() = default;
  ~A() { cout << "Deleted A!" << endl; }

 public:
  std::shared_ptr<B> b;
};

class B {
 public:
  B() = default;
  ~B() { cout << "Deleted B!" << endl; }

 public:
  std::unordered_map<int, std::shared_ptr<A>> id_to_a;
};

int main() {
  auto pa = std::make_shared<A>();
  auto pb = std::make_shared<B>();

  pa->b = pb;
  pb->id_to_a[0] = pa;

  return 0;
}
```

运行时会发现析构函数并不会被调用。

#### 改进代码

```c++
#include <iostream>
#include <memory>
#include <unordered_map>

using namespace std;

class B;

class A {
 public:
  A(const int v) : val(v) {}
  ~A() { cout << "Deleted A!" << endl; }

 public:
  std::shared_ptr<B> b;
  int val;
};

class B {
 public:
  B() = default;
  ~B() { cout << "Deleted B!" << endl; }

 public:
  std::unordered_map<int, std::weak_ptr<A>> id_to_a;    // 这里改成了使用weak_ptr
};

int main() {
  auto pa = std::make_shared<A>(250);

  auto pb = std::make_shared<B>();

  pa->b = pb;

  pb->id_to_a[pa->val] = pa;

  cout << pa.use_count() << endl;  // 1

  if (!pb->id_to_a[pa->val].expired()) {
    auto pa2 = pb->id_to_a[pa->val].lock();

    cout << pa2.use_count() << endl;  // 2

    cout << pa2->val << endl;  // 250
  }

  return 0;
}
```
