---
layout:     post
title:      "C++: Error with multiple definitions of function"
date:       2019-11-28
author:     Tong
catalog: true
tags:
    - Language
---

### Problematic source code

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

### Error Info

```bash
multiple definition of 'print()'
```


### [Reason](https://stackoverflow.com/questions/17904643/error-with-multiple-definitions-of-function)



### Solution 1: `static`

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

### Solution 2: `inline`

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

### Solution 3: separate declaration and definition

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

### Solution 4: `namespace`

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

### Solution 5: multiple classes in one file

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

### Solution 6: `class` + `static`

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
