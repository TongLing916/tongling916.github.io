---
layout:     post
title:      "C++ Serialization"
date:       2019-12-15
author:     Tong
catalog: true
tags:
    - Language
---

> http://www.ocoudert.com/blog/2011/07/09/a-practical-guide-to-c-serialization/
> https://www.boost.org/doc/libs/1_71_0/libs/serialization/doc/serialization.html
> https://www.boost.org/doc/libs/1_62_0/boost/archive/binary_oarchive.hpp


## Basic serialization (`text_iarchive`)

### `obj.h`

```c++
#pragma once

namespace boost {
namespace serialization {
class access;
}
}  // namespace boost

class Obj {
 public:
  // Serialization expects the object to have a default constructor
  Obj() : d1_(-1), d2_(false) {}
  Obj(int d1, bool d2) : d1_(d1), d2_(d2) {}
  bool operator==(const Obj& o) const { return d1_ == o.d1_ && d2_ == o.d2_; }

 private:
  int d1_;
  bool d2_;

  // Allow serialization to access non-public data members.
  friend class boost::serialization::access;

  template <typename Archive>
  void serialize(Archive& ar, const unsigned version) {
    ar& d1_& d2_;  // Simply serialize the data members of Obj
  }
};

```

### `main.cc`

```c++
#include "obj.h"

#include <assert.h>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <fstream>

int main() {
  const char* fileName = "saved.txt";

  // Create some objects
  const Obj o1(-2, false);
  const Obj o2;
  const Obj o3(21, true);
  const Obj* const p1 = &o1;

  // Save data
  {
    // Create an output archive
    std::ofstream ofs(fileName);
    boost::archive::text_oarchive ar(ofs);

    // Write data
    ar& o1& o2& o3& p1;
  }

  // Restore data
  Obj restored_o1;
  Obj restored_o2;
  Obj restored_o3;
  Obj* restored_p1;
  {
    // Create and input archive
    std::ifstream ifs(fileName);
    boost::archive::text_iarchive ar(ifs);

    // Load data
    ar& restored_o1& restored_o2& restored_o3& restored_p1;
  }

  // Make sure we restored the data exactly as it was saved
  assert(restored_o1 == o1);
  assert(restored_o2 == o2);
  assert(restored_o3 == o3);
  assert(restored_p1 != p1);
  assert(restored_p1 == &restored_o1);

  return 0;
}
```

### `CMakeLists.txt`

```bash
cmake_minimum_required(VERSION 2.8)
project(serialization)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(Boost COMPONENTS serialization REQUIRED)

include_directories(
    ${Boost_INCLUDE_DIRS})

add_executable( ${PROJECT_NAME} main.cc )

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
```

## More on pointer seiralization

- 我们一旦对一个指针或者引用进行serialization, 那么它所指向的object也会被serialization.因此，我们不需要显性地serialize被指向的变量。

### `main.cc`

```c++
#include <assert.h>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <fstream>
#include "obj.h"

int main() {
  const char* fileName = "saved.txt";

  // Create one object o1.
  const Obj o1(-2, false);
  const Obj* const p1 = &o1;

  // Save data
  {
    // Create an output archive
    std::ofstream ofs(fileName);
    boost::archive::text_oarchive ar(ofs);
    // Save only the pointer. This will trigger serialization
    // of the object it points too, i.e., o1.
    ar& p1;
  }

  // Restore data
  Obj* restored_p1;
  {
    // Create and input archive
    std::ifstream ifs(fileName);
    boost::archive::text_iarchive ar(ifs);
    // Load
    ar& restored_p1;
  }

  // Make sure we read exactly what we saved.
  assert(restored_p1 != p1);
  assert(* restored_p1 == o1);

  return 0;
}
```

- 当deserialize一个指针时，如果它所指的对象还没有被deserialize, 那么这个被指的对象也会自动被deserialize.这意味着，我们不能在deserialize一个指针之后，再对该指针所指对象进行deserialization. (The reason is that once the pointer deserialization has forced the object deserialization, one cannot rebuild this object at a different address.)

```c++
#include "obj.h"
#include <fstream>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

int main()
{
  const char* fileName = "saved.txt";
  std::ofstream ofs(fileName);

  // Create one object o1 and a pointer p1 to that object.
  const Obj o1(-2, false);
  const Obj* const p1 = &o1;

  // Serialize object, then pointer.
  // This works fine: after the object is deserialized, we can
  // deserialize the pointer by assigning it to the object’s address.
  {
    boost::archive::text_oarchive ar(ofs);
    ar & o1 & p1;
  }

  // Serialize pointer, then object.
  // This does not work: once p1 has been serialized, the object
  // has already been deserialized and its address cannot change.
  // This will throw an instance of 'boost::archive::archive_exception'
  // at runtime.
  {
    boost::archive::text_oarchive ar(ofs);
    ar & p1 & o1;
  }

  return 0;
}
```

## Explicit save and load function definitions

### `obj.h`

```c++
#pragma once

#include <boost/serialization/split_member.hpp>

class Obj {
 public:
  Obj() : d1_(-1), d2_(false) {}
  Obj(int d1, bool d2) : d1_(d1), d2_(d2) {}
  bool operator==(const Obj& o) const { return d1_ == o.d1_ && d2_ == o.d2_; }

 private:
  int d1_;
  bool d2_;

  friend class boost::serialization::access;

  template <class Archive>
  void save(Archive& ar, const unsigned int version) const {
    ar& d1_& d2_;
  }

  template <class Archive>
  void load(Archive& ar, const unsigned int version) {
    ar& d1_& d2_;
  }

  BOOST_SERIALIZATION_SPLIT_MEMBER()  // responsible for calling save/load when
                                      // using an output/input archive.
};
```

## Serialization of C-strings

- C-string不能被直接序列化，因为它是一种以`\0`结尾的`char`数组。

### `SerializeCStringHelper.h`

```c++
#pragma once
// File SerializeCStringHelper.h

#include <boost/serialization/split_member.hpp>
#include <boost/serialization/string.hpp>
#include <string>

class SerializeCStringHelper {
 public:
  SerializeCStringHelper(char*& s) : s_(s) {}
  SerializeCStringHelper(const char*& s) : s_(const_cast<char*&>(s)) {}

 private:
  friend class boost::serialization::access;

  template <class Archive>
  void save(Archive& ar, const unsigned version) const {
    bool isNull = (s_ == 0);
    ar& isNull;
    if (!isNull) {
      std::string s(s_);
      ar& s;
    }
  }

  template <class Archive>
  void load(Archive& ar, const unsigned version) {
    bool isNull;
    ar& isNull;
    if (!isNull) {
      std::string s;
      ar& s;
      s_ = strdup(s.c_str());
    } else {
      s_ = 0;
    }
  }

  BOOST_SERIALIZATION_SPLIT_MEMBER();

 private:
  char*& s_;
};
```

### `main.cc`

```c++
#include <assert.h>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <fstream>

#include "SerializeCStringHelper.h"

int main() {
  const char* fileName = "saved.txt";
  const char* str = "This is an example a C-string";

  // Save data
  {
    // Create an output archive
    std::ofstream ofs(fileName);

    boost::archive::text_oarchive ar(ofs);
    // Save
    SerializeCStringHelper helper(str);
    ar& helper;
  }

  // Restore data
  char* restored_str;
  {
    // Create and input archive
    std::ifstream ifs(fileName);
    boost::archive::text_iarchive ar(ifs);

    // Load
    SerializeCStringHelper helper(restored_str);
    ar& helper;
  }

  // Make sure we read exactly what we saved
  assert(restored_str != str);
  assert(strcmp(restored_str, str) == 0);

  return 0;
}
```


## Non-intrusive serialization

### `obj_public.h`

```c++
#pragma once

class ObjPublic {
 public:
  ObjPublic() : d1_(-1), d2_(false) {}
  ObjPublic(int d1, bool d2) : d1_(d1), d2_(d2) {}
  bool operator==(const ObjPublic& o) const {
    return d1_ == o.d1_ && d2_ == o.d2_;
  }

 public:
  int d1_;
  bool d2_;
};

namespace boost {
namespace serialization {

template <typename Archive>
void serialize(Archive& ar, ObjPublic& o, const unsigned int version) {
  ar& o.d1_& o.d2_;
}

}  // namespace serialization
}  // namespace boost
```

### `obj_private.h`

```c++
#pragma once

// Declaration of the template
class ObjPrivate;

namespace boost {
namespace serialization {

template <typename Archive>
void serialize(Archive& ar, ObjPrivate& o, const unsigned int version);

}  // namespace serialization
}  // namespace boost

// Definition of the class
class ObjPrivate {
 public:
  ObjPrivate() : d1_(-1), d2_(false) {}
  ObjPrivate(int d1, bool d2) : d1_(d1), d2_(d2) {}
  bool operator==(const ObjPrivate& o) const {
    return d1_ == o.d1_ && d2_ == o.d2_;
  }

 private:
  int d1_;
  bool d2_;

  // Allow serialization to access data members.
  template <typename Archive>
  friend void boost::serialization::serialize(Archive& ar, ObjPrivate& o,
                                              const unsigned int version);
};

//// Definition of the template
namespace boost {
namespace serialization {

template <typename Archive>
void serialize(Archive& ar, ObjPrivate& o, const unsigned int version) {
  ar& o.d1_& o.d2_;
}

}  // namespace serialization
}  // namespace boost
```

### `obj.h`

```c++
#pragma once

#include <boost/serialization/split_free.hpp>

class Obj {
 public:
  Obj() : d1_(-1), d2_(false) {}
  Obj(int d1, bool d2) : d1_(d1), d2_(d2) {}
  bool operator==(const Obj& o) const { return d1_ == o.d1_ && d2_ == o.d2_; }

 public:
  int d1_;
  bool d2_;
};

namespace boost {
namespace serialization {

template <class Archive>
void save(Archive& ar, const Obj& o, const unsigned int version) {
  ar& o.d1_& o.d2_;
}

template <class Archive>
void load(Archive& ar, Obj& o, const unsigned int version) {
  ar& o.d1_& o.d2_;
}

}  // namespace serialization
}  // namespace boost

BOOST_SERIALIZATION_SPLIT_FREE(Obj)
```


### `main.cc`

```c++
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <fstream>

#include "obj_private.h"
#include "obj_public.h"

int main() {
  const char* fileName = "saved.txt";
  std::ofstream ofs(fileName);

  const ObjPublic o1(-2, false);
  const ObjPublic* const p1 = &o1;
  const ObjPrivate o2(-2, false);
  const ObjPrivate* const p2 = &o2;
  {
    boost::archive::text_oarchive ar(ofs);
    ar& p1& p2;
  }

  // Restore data
  ObjPublic* restored_p1;
  ObjPrivate* restored_p2;
  {
    // Create and input archive
    std::ifstream ifs(fileName);
    boost::archive::text_iarchive ar(ifs);
    // Load
    ar& restored_p1& restored_p2;
  }

  // Make sure we read exactly what we saved.
  assert(restored_p1 != p1);
  assert(*restored_p1 == o1);
  assert(restored_p2 != p2);
  assert(*restored_p2 == o2);
  return 0;
}
```

## Serialization of STL containers

- STL containers会被自动序列化，只需要加入以下头文件。

```c++
#include <boost/serialization/array.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/hash_map.hpp>
#include <boost/serialization/hash_set.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/slist.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/bitset.hpp>
#include <boost/serialization/string.hpp>
```

以下是两种写法：

```c++
#include <boost/serialization/vector.hpp>

template<typename Archive>
void serialize(Archive& ar, std::vector<Obj>& objs, const unsigned version) {
  ar & objs;
}
```

```c++
template<typename Archive>
void save(Archive& ar, const std::vector<Obj>& objs, const unsigned version) {
  ar << objs.size();
  for (size_t i = 0; i < objs.size(); ++i) {
    ar << objs[i];
  }
}

template<typename Archive>
void load(Archive& ar, std::vector<Obj>& objs, const unsigned version) {
  size_t size;
  ar >> size;
  objs.resize(size);
  for (size_t i = 0; i < size; ++i) {
    ar >> objs[i];
  }
```

## Serialization of base class

- 如果一个class是从另一个继承过来的，那么base class也需要被序列化。

```c++
#include <boost/serialization/base_object.hpp>

class Base {
public:
  Base() : c_('\0') {}
  Base(char c) : c_(c) {}
  bool operator==(const Base& o) const { return c_ == o.c_; }

private:
  char c_;

  friend class boost::serialization::access;

  template <typename Archive>
  void serialize(Archive& ar, const unsigned version) {
    ar & c_;
  }
};

class Obj : public Base {
private:
  typedef Base _Super;
public:
  Obj() : _Super(), d1_(-1), d2_(false) {}
  Obj(int d1, bool d2) : _Super('a'), d1_(d1), d2_(d2) {}
  bool operator==(const Obj& o) const {
    return _Super::operator==(o) && d1_ == o.d1_ && d2_ == o.d2_;
  }

private:
  int  d1_;
  bool d2_;

  friend class boost::serialization::access;

  template <typename Archive>
  void serialize(Archive& ar, const unsigned version) {
    ar & boost::serialization::base_object<_Super>(*this);
    ar & d1_ & d2_;
  }
};
```

## Versioning

- We want maintain back-compatibility when the class Obj evolves. For instance, if a new data member ‘ID_’ is added, we want to read an old archive and build new Obj, with the missing data member taking the default value.

```c++
#pragma once

#include <boost/serialization/split_member.hpp>
#include <boost/serialization/version.hpp>

class Obj {
public:
  Obj() : d1_(-1), d2_(false), ID_(0) {}
  Obj(int d1, bool d2, unsigned ID id) : d1_(d1), d2_(d2), ID_(id) {}
  bool operator==(const Obj& o) const {
    return d1_ == o.d1_ && d2_ == o.d2_ && ID_ == o.ID_;
  }

private:
  int  d1_;
  bool d2_;
  unsigned ID_;

  friend class boost::serialization::access;

  template<class Archive>
  void save(Archive & ar, const unsigned int version) const {
    ar & d1_ & d2_ & ID_;
  }

  template<class Archive>
  void load(Archive & ar, const unsigned int version) {
    ar & d1_ & d2_;
    // If archive’s version is 0 (i.e., is old), ID_ keeps
    // its default value from the new data model,
    // else we read ID_’s value from the archive.
    if (version > 0) {
      ar & ID_;
    }
  }

  BOOST_SERIALIZATION_SPLIT_MEMBER()

};
```

## Serialization of const data or objects

```c++
#pragma once

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

class Obj {
public:
  Obj() : d1_(-1), d2_(false) {}
  Obj(int d1, bool d2) : d1_(d1), d2_(d2) {}

private:
  const int d1_;
  bool d2_;

  // Allow serialization to access data members.
  friend class boost::serialization::access;

  template<typename A>
  void serialize(A& ar, const unsigned version) {
    ar & const_cast<int&>(d1_) & d2_;
  }
};
```

## Text, XML, and binary archives

```c++
// Text archive that defines boost::archive::text_oarchive
// and boost::archive::text_iarchive
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

// XML archive that defines boost::archive::xml_oarchive
// and boost::archive::xml_iarchive
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>

// XML archive which uses wide characters (use for UTF-8 output ),
// defines boost::archive::xml_woarchive
// and boost::archive::xml_wiarchive
#include <boost/archive/xml_woarchive.hpp>
#include <boost/archive/xml_wiarchive.hpp>

// Binary archive that defines boost::archive::binary_oarchive
// and boost::archive::binary_iarchive
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
```
