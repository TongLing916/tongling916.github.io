---
layout:     post
title:      "C++ - Nitty Gritty"
date:       2020-4-27
author:     Tong
catalog: true
tags:
    - C++
---

### [Rule of Three](https://stackoverflow.com/questions/4172722/what-is-the-rule-of-three)

### [Copy-and-Swap Idiom](https://stackoverflow.com/questions/3279543/what-is-the-copy-and-swap-idiom)

### [`explicit`](https://stackoverflow.com/questions/121162/what-does-the-explicit-keyword-mean/121163#121163)

### [`initialization list`](https://stackoverflow.com/questions/12697625/how-the-try-catch-in-initialization-list-works)

### [public friend swap member function](https://stackoverflow.com/questions/5695548/public-friend-swap-member-function)

### [numeric_limits](https://en.cppreference.com/w/cpp/types/numeric_limits/min)

- `std::numeric_limits<float>::min()` returns the minimum positive normalized value. To find the value that has no values less than it, use `std::numeric_limits<float>::lowest()`. 

### `accumulate`

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


### Thread safe

1. [Is `std::shared_ptr` thread-safe?](https://stackoverflow.com/questions/9127816/stdshared-ptr-thread-safety-explained)

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

### [new](https://en.cppreference.com/w/cpp/language/new)

#### `new`

> [new operator](https://docs.microsoft.com/en-us/cpp/cpp/new-operator-cpp?view=msvc-160)

- Call `operator new` and corresponding constructors
- Cannot be overloaded

#### `operator new`

> [operator new](https://en.cppreference.com/w/cpp/memory/new/operator_new)

- Allocate memory but does not call constructor
- Can be overloaded

```c++
#include <cstdio>
#include <cstdlib>
#include <new>
// replacement of a minimal set of functions:
void* operator new(std::size_t sz) {
    std::printf("global op new called, size = %zu\n", sz);
    void *ptr = std::malloc(sz);
    if (ptr)
        return ptr;
    else
        throw std::bad_alloc{};
}
void operator delete(void* ptr) noexcept
{
    std::puts("global op delete called");
    std::free(ptr);
}
int main() {
     int* p1 = new int;
     delete p1;
 
     int* p2 = new int[10]; // guaranteed to call the replacement in C++11
     delete[] p2;
}
```

```c++
#include <iostream>
// class-specific allocation functions
struct X {
    static void* operator new(std::size_t sz)
    {
        std::cout << "custom new for size " << sz << '\n';
        return ::operator new(sz);
    }
    static void* operator new[](std::size_t sz)
    {
        std::cout << "custom new[] for size " << sz << '\n';
        return ::operator new(sz);
    }
};
int main() {
     X* p1 = new X;
     delete p1;
     X* p2 = new X[10];
     delete[] p2;
}
```


#### `placement new`

> [What are uses of placement new](https://stackoverflow.com/questions/362953/what-are-uses-of-the-c-construct-placement-new)

> [What is “placement new” and why would I use it?](https://isocpp.org/wiki/faq/dtors#placement-new)

- `Placement new` allows you to construct an object in memory that's already allocated.


```c++
// pre-allocated buffer
char *buf  = new char[sizeof(string)]; 

// placement new
string *p = new (buf) string("hi");    

// ordinary heap allocation
string *q = new string("hi");          
```

```c++
#include <stdexcept>
#include <iostream>
struct X {
    X() { throw std::runtime_error(""); }
    // custom placement new
    static void* operator new(std::size_t sz, bool b) {
        std::cout << "custom placement new called, b = " << b << '\n';
        return ::operator new(sz);
    }
    // custom placement delete
    static void operator delete(void* ptr, bool b)
    {
        std::cout << "custom placement delete called, b = " << b << '\n';
        ::operator delete(ptr);
    }
};
int main() {
   try {
     X* p1 = new (true) X;
   } catch(const std::exception&) { }
}
```


```c++
#include <new>        // Must #include this to use "placement new"

#include "Fred.h"     // Declaration of class Fred

void someCode() {
  char memory[sizeof(Fred)];     // Line #1
  void* place = memory;          // Line #2
  Fred* f = new(place) Fred();   // Line #3 (see "DANGER" below)
  // The pointers f and place will be equal
  // ...
  f->~Fred();   // Explicitly call the destructor for the placed object
}
```

### [enable_if](https://en.cppreference.com/w/cpp/types/enable_if)

> [enable_if](https://docs.microsoft.com/en-us/cpp/standard-library/enable-if-class?view=msvc-160)

### [SFINAE](https://en.cppreference.com/w/cpp/language/sfinae) - Substitution Failure Is Not An Error

###