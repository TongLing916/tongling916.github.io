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

- `std::numeric_limits<float>::min()` returns the minimum positive normalized value. To find the value that has no values less than it, use `std::numeric_limits<float>::lowest()`. 

### accumulate

- The type of the initial value is very __important__.

```c++
#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

int main() {
  vector<float> v{0.1f, 0.2f, 0.3f};
  cout << accumulate(v.begin(), v.end(), 0) << endl;    // Output: 0
  cout << accumulate(v.begin(), v.end(), 0.f) << endl;  // Output: 0.6
  return 0;
}
```

### Comparator

> https://stackoverflow.com/questions/32263560/errorinvalid-comparator-when-sorting-using-custom-comparison-function

- `std::sort` requires a [__strict weak ordering__](https://en.cppreference.com/w/cpp/named_req/Compare). For a strict weak ordering, `comp(x, x)` must be false.

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

### static_cast vs. dynamic_cast

> [Different cast](https://stackoverflow.com/questions/332030/when-should-static-cast-dynamic-cast-const-cast-and-reinterpret-cast-be-used)

#### Summary

1. [`dynamic_cast`](http://tongling916.github.io/2019/08/25/C++/#%E8%BF%90%E8%A1%8C%E6%97%B6%E7%B1%BB%E5%9E%8B%E8%AF%86%E5%88%AB)主要在基类和子类间转换起作用．
2. `static_cast`可以将一个指向基类的基类指针转换成子类指针，但是，__如果子类包含新的`virtual`函数__，我们调用这个函数时，程序会崩溃，否则，没关系（__但是基类中的`virtual`函数不会被重载__）．
3. 如果用`dynamic_cast`将一个指向基类的基类指针强行转换成子类指针，我们会得到一个空指针．

#### Example 1 - 指向基类的基类指针->子类指针

- 子类不包含新的virtual函数

```c++
#include <iostream>

using namespace std;

class Parent {
 public:
  virtual void Print() { cout << "I am a Parent!" << endl; }
};

class Child : public Parent {
 public:
  void Print() override { cout << "I am a Child!" << endl; }

  // 这个函数是否为virtual会有不同的结果
  void PrintPretty() { cout << "Child Only!" << endl; }
};

int main() {
  Parent* parent = new Parent();
  parent->Print();

  cout << endl;
  Child* child_dynamic = dynamic_cast<Child*>(parent);
  if (child_dynamic == nullptr) {
    cout << "parent cannot be converted to child" << endl;
  } else {
    child_dynamic->Print();
    child_dynamic->PrintPretty();
  }

  cout << endl;
  Child* child_static = static_cast<Child*>(parent);
  child_static->Print();　// I am a Parent! 函数并不会被重载
  child_static->PrintPretty();

  return 0;
}
```

```bash
I am a Parent!

parent cannot be converted to child

I am a Parent!
Child Only!
```

- 子类包含新的virtual函数

```c++
#include <iostream>

using namespace std;

class Parent {
 public:
  virtual void Print() { cout << "I am a Parent!" << endl; }
};

class Child : public Parent {
 public:
  void Print() override { cout << "I am a Child!" << endl; }

  // 这个函数是否为virtual不会有不同的结果
  void PrintPretty() { cout << "Child Only!" << endl; }
};

int main() {
  Parent* parent = new Parent();
  parent->Print();

  cout << endl;
  Child* child_dynamic = dynamic_cast<Child*>(parent);
  if (child_dynamic == nullptr) {
    cout << "parent cannot be converted to child" << endl;
  } else {
    child_dynamic->Print();
    child_dynamic->PrintPretty();
  }

  cout << endl;
  Child* child_static = static_cast<Child*>(parent);
  child_static->Print();  // I am a Parent! 函数并不会被重载
  child_static->PrintPretty();  // segmentation fault

  return 0;
}
```

```bash
I am a Parent!

parent cannot be converted to child

I am a Parent!
Segmentation fault (core dumped)
```

#### Example 2 - 指向子类的基类指针->子类指针

- 子类不包含新的virtual函数

```c++
#include <iostream>

using namespace std;

class Parent {
 public:
  virtual void Print() { cout << "I am a Parent!" << endl; }
};

class Child : public Parent {
 public:
  void Print() override { cout << "I am a Child!" << endl; }
  void PrintPretty() { cout << "Child Only!" << endl; }
};

int main() {
  Parent* parent = new Child();
  parent->Print();  // I am a Child! 函数会被重载!

  cout << endl;
  Child* child_dynamic = dynamic_cast<Child*>(parent);
  if (child_dynamic == nullptr) {
    cout << "parent cannot be converted to child" << endl;
  } else {
    child_dynamic->Print();
    child_dynamic->PrintPretty();
  }

  cout << endl;
  Child* child_static = static_cast<Child*>(parent);
  child_static->Print();
  child_static->PrintPretty();

  return 0;
}
```

```bash
I am a Child!

I am a Child!
Child Only!

I am a Child!
Child Only!
```

- 子类包含新的virtual函数

```c++
#include <iostream>

using namespace std;

class Parent {
 public:
  virtual void Print() { cout << "I am a Parent!" << endl; }
};

class Child : public Parent {
 public:
  void Print() override { cout << "I am a Child!" << endl; }
  virtual void PrintPretty() { cout << "Child Only!" << endl; }
};

int main() {
  Parent* parent = new Child();
  parent->Print();  // I am a Child! 函数会被重载!

  cout << endl;
  Child* child_dynamic = dynamic_cast<Child*>(parent);
  if (child_dynamic == nullptr) {
    cout << "parent cannot be converted to child" << endl;
  } else {
    child_dynamic->Print();
    child_dynamic->PrintPretty();
  }

  cout << endl;
  Child* child_static = static_cast<Child*>(parent);
  child_static->Print();
  child_static->PrintPretty();

  return 0;
}
```

```bash
I am a Child!

I am a Child!
Child Only!

I am a Child!
Child Only!
```

### Error with multiple definitions of function

#### Problematic source code

```c++
// main.cpp
#include <iostream>
#include "caller_a.h"
#include "caller_b.h"
int main()
{
	caller_a a;
	caller_b b;
	a.call();
	b.call();
}
```

```c++
// caller_a.h
#pragma once

class caller_a {
public:
	void call();
};
```

```c++
// caller_a.cpp
#include "caller_a.h"
#include "common.h"

void caller_a::call() {
	print("caller_a");
}
```

```c++
// caller_b.h
#pragma once

class caller_b {
public:
	void call();
};
```

```c++
// caller_b.cpp
#include "caller_b.h"
#include "common.h"

void caller_b::call() {
	print("caller_b");
}
```

```c++
// common.h
#pragma once

#include <iostream>
#include <string>

void print(const std::string& s) {
	std::cout << s << " is called." << std::endl;
}
```

#### Error Info

```bash
multiple definition of 'print()'
```


#### [Reason](https://stackoverflow.com/questions/17904643/error-with-multiple-definitions-of-function)

#### Solution 1: `static`

```c++
// main.cpp
#include <iostream>
#include "caller_a.h"
#include "caller_b.h"
int main()
{
	caller_a a;
	caller_b b;
	a.call();
	b.call();
}
```

```c++
// caller_a.h
#pragma once

class caller_a {
public:
	void call();
};
```

```c++
// caller_a.cpp
#include "caller_a.h"
#include "common.h"

void caller_a::call() {
	print("caller_a");
}
```

```c++
// caller_b.h
#pragma once

class caller_b {
public:
	void call();
};
```

```c++
// caller_b.cpp
#include "caller_b.h"
#include "common.h"

void caller_b::call() {
	print("caller_b");
}
```

```c++
// common.h
#pragma once

#include <iostream>
#include <string>

static void print(const std::string& s) {
	std::cout << s << " is called. (Solution 1)" << std::endl;
}
```

#### Solution 2: `inline`

```c++
// main.cpp
#include "caller_a.h"
#include "caller_b.h"
int main()
{
	caller_a a;
	caller_b b;
	a.call();
	b.call();
}
```

```c++
// caller_a.h
#pragma once

class caller_a {
public:
	void call();
};
```

```c++
// caller_a.cpp
#include "caller_a.h"
#include "common.h"

void caller_a::call() {
	print("caller_a");
}
```

```c++
// caller_b.h
#pragma once

class caller_b {
public:
	void call();
};
```

```c++
// caller_b.cpp
#include "caller_b.h"
#include "common.h"

void caller_b::call() {
	print("caller_b");
}
```

```c++
// common.h
#pragma once

#include <iostream>
#include <string>

inline void print(const std::string& s) {
	std::cout << s << " is called. (Solution 2)" << std::endl;
}
```

#### Solution 3: separate declaration and definition

```c++
// main.cpp
#include "caller_a.h"
#include "caller_b.h"
int main()
{
	caller_a a;
	caller_b b;
	a.call();
	b.call();
}
```

```c++
// caller_a.h
#pragma once

class caller_a {
public:
	void call();
};
```

```c++
// caller_a.cpp
#include "caller_a.h"
#include "common.h"

void caller_a::call() {
	print("caller_a");
}
```

```c++
// caller_b.h
#pragma once

class caller_b {
public:
	void call();
};
```

```c++
// caller_b.cpp
#include "caller_b.h"
#include "common.h"

void caller_b::call() {
	print("caller_b");
}
```

```c++
// common.h
#pragma once
#include <string>

class Common
{
public:
	void print(const std::string& s);
};
```

```c++
// common.cpp
#include "common.h"
#include <iostream>

void Common::print(const std::string& s) {
	std::cout << s << " is called. (Solution 3)" << std::endl;
}
```

#### Solution 4: `namespace`

```c++
// main.cpp
#include "caller_a.h"
#include "caller_b.h"
int main()
{
	caller_a a;
	caller_b b;
	a.call();
	b.call();
}
```

```c++
// caller_a.h
#pragma once

class caller_a {
public:
	void call();
};
```

```c++
// caller_a.cpp
#include "caller_a.h"
#include "common.h"

void caller_a::call() {
	print("caller_a");
}
```

```c++
// caller_b.h
#pragma once

class caller_b {
public:
	void call();
};
```

```c++
// caller_b.cpp
#include "caller_b.h"
#include "common.h"

void caller_b::call() {
	print("caller_b");
}
```

```c++
// common.h
#pragma once

#include <iostream>
#include <string>

namespace {
	void print(const std::string& s) {
		std::cout << s << " is called. (Solution 4)" << std::endl;
	}
}
```

#### Solution 5: multiple classes in one file

```c++
// main.cpp
#include "caller.h"
int main()
{
	caller_a a;
	caller_b b;
	a.call();
	b.call();
}
```

```c++
// caller.h
#pragma once

class caller_a {
public:
	void call();
};

class caller_b {
public:
	void call();
};
```

```c++
// caller.cpp
#include "caller.h"
#include "common.h"

void caller_a::call() {
	print("caller_a");
}

void caller_b::call() {
	print("caller_b");
}
```

```c++
// common.h
#pragma once

#include <iostream>
#include <string>

void print(const std::string& s) {
	std::cout << s << " is called. (Solution 5)" << std::endl;
}
```

#### Solution 6: `class` + `static`

```c++
// main.cpp
#include "caller_a.h"
#include "caller_b.h"
int main()
{
	caller_a a;
	caller_b b;
	a.call();
	b.call();
}
```

```c++
// caller_a.h
#pragma once

class caller_a {
public:
	void call();
};
```

```c++
// caller_a.cpp
#include "caller_a.h"
#include "common.h"

void caller_a::call() {
	Common::print("caller_a");
}
```

```c++
// caller_b.h
#pragma once

class caller_b {
public:
	void call();
};
```

```c++
// caller_b.cpp
#include "caller_b.h"
#include "common.h"

void caller_b::call() {
	Common::print("caller_b");
}
```

```c++
// common.h
#pragma once

#include <iostream>
#include <string>

class Common
{
public:
	static void print(const std::string& s) {
		std::cout << s << " is called. (Solution 6)" << std::endl;
	}
};
```

### [`emplace_back` vs. `push_back`](https://en.cppreference.com/w/cpp/container/vector/emplace_back)

> [Why would I ever use push_back instead of emplace_back?](https://stackoverflow.com/questions/10890653/why-would-i-ever-use-push-back-instead-of-emplace-back)

```c++
#include <iostream>
#include <vector>
using namespace std;

int main() {
  vector<vector<int>> res;
  res.push_back({1, 2, 3});     // Correct
  res.emplace_back({1, 2, 3});  // Wrong: Compile Error
  return 0;
}
```

### [Operator Overloading](https://en.cppreference.com/w/cpp/language/operators)

1. `operator[]` can __only__ have one argument, while `operator()` can have multiple.