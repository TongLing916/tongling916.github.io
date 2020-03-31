---
layout:     post
title:      "static_cast vs. dynamic_cast"
date:       2020-3-31
author:     Tong
catalog: true
tags:
    - Language
---

> [不同cast的区别](https://stackoverflow.com/questions/332030/when-should-static-cast-dynamic-cast-const-cast-and-reinterpret-cast-be-used)

### Summary

1. [`dynamic_cast`](http://www.lingtong.de/2019/08/25/C++/#%E8%BF%90%E8%A1%8C%E6%97%B6%E7%B1%BB%E5%9E%8B%E8%AF%86%E5%88%AB)主要在基类和子类间转换起作用．
2. `static_cast`可以将一个指向基类的基类指针转换成子类指针，但是，__如果子类包含新的`virtual`函数__，我们调用这个函数时，程序会崩溃，否则，没关系（__但是基类中的`virtual`函数不会被重载__）．
3. 如果用`dynamic_cast`将一个指向基类的基类指针强行转换成子类指针，我们会得到一个空指针．


### Example 1 - 指向基类的基类指针->子类指针

#### 子类不包含新的virtual函数

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

#### 子类包含新的virtual函数

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




### Example 2 - 指向子类的基类指针->子类指针

#### 子类不包含新的virtual函数

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

#### 子类包含新的virtual函数

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

