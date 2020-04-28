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

### Circular Dependency

If both two classes contain the `shared_ptr` of the other class, then these two classes will not be deleted eventually, causing __memory leak__.

#### Problematic Codes

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

When we run the codes, we will notice the destruction will not be called.

#### Improved Codes

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
  std::unordered_map<int, std::weak_ptr<A>> id_to_a;    // Hier we use 'weak_ptr'
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
