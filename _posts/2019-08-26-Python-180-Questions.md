---
layout:     post
title:      "Python 180 Questions"
date:       2019-8-26
author:     Tong
catalog: true
tags:
    - Language
---

### 基础

1. 列出 5 个常用 Python 标准库？
os, random, collections, sys, math

2. Python 内建数据类型有哪些？
Numbers (list, float, complex), list, dict, tuple, set, string, bytes, bool

3. 简述 with 方法打开处理文件帮我我们做了什么？
with语句开始运行时，会在上下文管理器对象上调用__enter__方法。with语句运行结束后，会在上下文管理器对象上调用__exit__方法，以此扮演finally子句的角色。

4. 列出 Python 中可变数据类型和不可变数据类型，为什么？
可变：set, list, dict
不可变：numbers, tuple, string

5. Python 获取当前日期？
import datetime
today = datetime.date.today()
print(today)
formatted_today = today.strftime('%y%m%d') # 191007

6. 统计字符串每个单词出现的次数

```python
class Count:
    def __init__(self, string):
        self.string = string

    def get_count(self):
        words = " ".join(self.string.split()).split()
        word_count = {}
        for word in words:
            if word not in word_count:
                word_count[word] = 1
            else:
                word_count[word] += 1
        return word_count


if __name__ == "__main__":
    string = input("Enter a string: ")
    obj = Count(string)
    counts = obj.get_count()
    for word in counts:
        print(str(word) + "\t" + str(counts[word]))

```

7. 用 python 删除文件和用 linux 命令删除文件方法

```python
#!/usr/bin/python
import os
myfile="/tmp/foo.txt"

## If file exists, delete it ##
if os.path.isfile(myfile):
    os.remove(myfile)
else:    ## Show an error ##
    print("Error: %s file not found" % myfile)
```

```bash
rm <file_name>.txt

-f, --force
ignore nonexistent files, never prompt
-i
prompt before every removal
-I
prompt once before removing more than three files, or when removing recursively. Less intrusive than -i, while still giving protection against most mistakes
--interactive[=WHEN]
prompt according to WHEN: never, once (-I), or always (-i). Without WHEN, prompt always
--one-file-system
when removing a hierarchy recursively, skip any directory that is on a file system different from that of the corresponding command line argument
--no-preserve-root
do not treat '/' specially
--preserve-root
do not remove '/' (default)
-r, -R, --recursive
remove directories and their contents recursively
-v, --verbose
explain what is being done
--help
display this help and exit
--version
output version information and exit
```

```bash
To remove a file whose name starts with a '-', for example '-foo', use one of these commands:

rm -- -foo
rm ./-foo
```

8. 写一段自定义异常代码

```python
class CalcError(Exception):
    pass

class NumError(CalcError):
    def __init__(self, numA, numB):
        self.numA = numA
        self.numB = numB
    def __str__(self):
        return r"本计算器只接收整数!"

def calculator(a, b):
    try:
        if type(a) != int or type(b) != int:
            raise NumError(a, b)
    except Exception as e:
        print(e)
    else:
        print(str(a) + " + " + str(b) + " = " + str(a + b))

if __name__ == "__main__":
    calculator(1, 2)
    calculator(1.5, 2)
```

9. 举例说明异常模块中 try except else finally 的相关意义

见上题。

10. 遇到 bug 如何处理


### 语言特性
1. [谈谈对 Python 和其他语言的区别](https://www.zhihu.com/question/21976478)

Python属于比较“自由”的语言，首先变量使用前不需要声明类型，其次语句结束不需要使用分号作为结尾，同时不需要大括号进行代码块的标注，使用缩进对大括号进行代替。

2. [简述解释型和编译型编程语言](https://blog.csdn.net/zhu_xun/article/details/16921413)

用编译型语言写的程序执行之前，需要一个专门的编译过程，通过编译系统（不仅仅只是通过编译器，编译器只是编译系统的一部分）把高级语言翻译成机器语言（预处理器->编译器->汇编器->链接器），把源高级程序编译成为机器语言文件，比如windows下的exe文件。以后就可以直接运行而不需要编译了，因为翻译只做了一次，运行时不需要翻译，所以编译型语言的程序执行效率高，但也不能一概而论，部分解释型语言的解释器通过在运行时动态优化代码，甚至能够使解释型语言的性能超过编译型语言。

解释则不同，解释型语言编写的程序不需要编译。解释型语言在运行的时候才翻译，比如VB语言，在执行的时候，专门有一个解释器能够将VB语言翻译成机器语言，每个语句都是执行的时候才翻译。这样解释型语言每执行一次就要翻译一次，效率比较低。


3. Python 的解释器种类以及相关特点？
    1.	当从Python官方网站下载并安装好Python2.7后，就直接获得了一个官方版本的解释器：Cpython，这个解释器是用C语言开发的，所以叫CPython，在命名行下运行python，就是启动CPython解释器，CPython是使用最广的Python解释器。
    2.	IPython是基于CPython之上的一个交互式解释器，也就是说，IPython只是在交互方式上有所增强，但是执行Python代码的功能和CPython是完全一样的，好比很多国产浏览器虽然外观不同，但内核其实是调用了IE。
    3.	PyPy是另一个Python解释器，它的目标是执行速度，PyPy采用JIT技术，对Python代码进行动态编译，所以可以显著提高Python代码的执行速度。
    4.	Jython是运行在Java平台上的Python解释器，可以直接把Python代码编译成Java字节码执行。
    5.	IronPython和Jython类似，只不过IronPython是运行在微软.Net平台上的Python解释器，可以直接把Python代码编译成.Net的字节码。



4. [说说你知道的Python3 和 Python2 之间的区别？](https://www.cnblogs.com/qjx-2016/p/7991956.html)

print
* py2：print语句，语句就意味着可以直接跟要打印的东西，如果后面接的是一个元组对象，直接打印。（如果希望在 Python2 中 把 print 当函数使用，那么可以导入 future 模块 中的 print_function）
* py3：print函数，函数就以为这必须要加上括号才能调用，如果接元组对象，可以接收多个位置参数，并可以打印。下面有个示例：
```
# py2
>>> print("hello", "world")
('hello', 'world')
# py3
>>> print("hello", "world")
hello world
# py2
>>> print("hello", "world")
('hello', 'world')
>>> from __future__ import print_function
>>> print("hello", "world")
hello world
```

输入函数
* py2：input_raw()
* py3：input()

在使用super()的不同
* py2：必须显示的在参数中写上基类
* py：直接无参数调用即可

1/2的结果
* py2：返回0
* py3：返回0.5，没有了int和long的区别

编码
* py2：默认编码ascii
* py3：默认编码utf-8(而且为了在py2中使用中文，在头部引入coding声明，不推荐使用)

字符串
* py2：unicode类型表示字符串序列，str类型表示字节序列
* py3:：str类型表示字符串序列，byte类型表示字节序列

True和False
* py2：True 和 False 在 Python2 中是两个全局变量，可以为其赋值或者进行别的操作，初始数值分别为1和0，虽然修改是违背了python设计的原则，但是确实可以更改
* py3：修正了这个变量，让True或False不可变

迭代器
* py2：当中许多返回列表的方法，如range,字典对象的 `dict.keys()`、`dict.values() `方法, map、filter、zip；并且迭代器必须实现next方法
* py3：将返回列表的方法改为了返回迭代器对象，内置了__next__，不用特意去实现next

nonlocal
* py2：没有办法在嵌套函数中将变量声明为一个非局部变量，只能在函数中声明全局变量
* py3：nonlocal方法实现了，示例如下：
```python
def func():
 c = 1
    def foo():
        c = 12
    foo()
    print(c)
func()    #1
```

```python
def func():
    c = 1
    def foo():
        nonlocal c
        c = 12
    foo()
    print(c)
func()   # 12
```

5. Python3 和 Python2 中 int 和 long 区别？
python3 彻底废弃了 long+int 双整数实现的方法, 统一为 int , 支持高精度整数运算

6. xrange 和 range 的区别？
    1.	xrange() 函数用法与 range() 完全相同。只是xrange()生成的时生成器而不是数组。
    2.	要生成很大的数字序列的时候，用xrange会比range性能优很多，因为不需要一上来就开辟一块很大的内存空间。




### 编码规范
7. [什么是 PEP8?](https://blog.csdn.net/ratsniper/article/details/78954852)

Python代码编码规范

8. 了解 Python 之禅么？
```python
import this
```

```
The Zen of Python, by Tim Peters

Beautiful is better than ugly.
Explicit is better than implicit.
Simple is better than complex.
Complex is better than complicated.
Flat is better than nested.
Sparse is better than dense.
Readability counts.
Special cases aren't special enough to break the rules.
Although practicality beats purity.
Errors should never pass silently.
Unless explicitly silenced.
In the face of ambiguity, refuse the temptation to guess.
There should be one-- and preferably only one --obvious way to do it.
Although that way may not be obvious at first unless you're Dutch.
Now is better than never.
Although never is often better than *right* now.
If the implementation is hard to explain, it's a bad idea.
If the implementation is easy to explain, it may be a good idea.
Namespaces are one honking great idea -- let's do more of those!
```


9. [了解 docstring 么？](https://www.datacamp.com/community/tutorials/docstrings-python#sixth-sub)

```python
class CalcError(Exception):
    '''Base Class of custom Exception'''
    pass

class NumError(CalcError):
    """Class of custom Exception derived from CalcError"""
    def __init__(self, numA, numB):
        self.numA = numA
        self.numB = numB
    def __str__(self):
        return r"本计算器只接收整数!"

if __name__ == "__main__":
    print(CalcError.__doc__)
    print(NumError.__doc__)


```

10. 了解类型注解么？

```python
def add(a: int, b: int) -> int:
    return a + b

a:int = 1
b:int = 3
print(add(a, b))
```

11. [例举你知道 Python 对象的命名规范，例如方法或者类等](https://blog.csdn.net/feikon2/article/details/79126774)

    1. 变量命名总结：
        * 单下划线开头变量：protected
        * 双下划线开头变量：private
        * 双下划线开头，双下划线结尾：系统内置变量
    2. 函数命名总结：
        * 私有方法：小写和一个前导下划线
        * 特殊方法（魔术方法）：小写和两个前导下划线，两个后置下划线
        * 函数参数：小写和下划线，缺省值等号两边无空格
    3. 类名称命名：
        * 类总是使用驼峰格式命名，即所有单词首字母大写其余字母小写。


12. Python 中的注释有几种？

```python
'''
comment
'''

"""
comment
"""

# comment

#!/usr/bin/python  
# -*- coding: utf-8 -*-

```

13. 如何优雅的给一个函数加注释？

函数注释通常在 def 语句下方，第一行表示函数用法，接下来对函数接受的参数进行解释，最后对函数的返回值进行注释，方便他人理解函数的用法。

14. 如何给变量加注释？


15. [Python 代码缩进中是否支持 Tab 键和空格混用。](https://blog.csdn.net/liuyumoye/article/details/81610311)

不支持，同时尽量使用4个空格代替Tab缩进。

16. [是否可以在一句 import 中导入多个库?](https://zhuanlan.zhihu.com/p/63143493)

```python
import lib1, lib2, lib3, lib4, lib5
```


17. [在给 Py 文件命名的时候需要注意什么?](https://softwareengineering.stackexchange.com/questions/308972/python-file-naming-convention)

使用全小写或者下划线进行命名。

18. [例举几个规范 Python 代码风格的工具](https://blog.csdn.net/zong596568821xp/article/details/84251616)

pylint, black


### 数据类型

#### 字符串

19. 列举 Python 中的基本数据类型？

20. 如何区别可变数据类型和不可变数据类型

21. 将"hello world"转换为首字母大写"Hello World"

22. 如何检测字符串中只含有数字?

23. 将字符串"ilovechina"进行反转

```python
"ilovechina"[::-1]
```

24. Python 中的字符串格式化方式你知道哪些？

25. 有一个字符串开头和末尾都有空格，比如“ adabdw ”,要求写一个函数把这个字符串的前后空格都去掉。

26. 获取字符串”123456“最后的两个字符。

27. 一个编码为 GBK 的字符串 S，要将其转成 UTF-8 编码的字符串，应如何操作？

28. (1)s="info：xiaoZhang 33 shandong"，用正则切分字符串输出['info', 'xiaoZhang', '33', 'shandong'](2) a = "你好 中国 "，去除多余空格只留一个空格。
"   xyz   ".strip()            # returns "xyz"  
"   xyz   ".lstrip()           # returns "xyz   "  
"   xyz   ".rstrip()           # returns "   xyz"  
"  x y z  ".replace(' ', '')   # returns "xyz"

29. (1)怎样将字符串转换为小写 (2)单引号、双引号、三引号的区别？




#### 列表

30.已知 AList = [1,2,3,1,2],对 AList 列表元素去重，写出具体过程。

31.如何实现 "1,2,3" 变成 ["1","2","3"]

32.给定两个 list，A 和 B，找出相同元素和不同元素

33.[[1,2],[3,4],[5,6]]一行代码展开该列表，得出[1,2,3,4,5,6]

34.合并列表[1,5,7,9]和[2,2,6,8]

35.如何打乱一个列表的元素？




字典



36.字典操作中 del 和 pop 有什么区别

37.按照字典的内的年龄排序



38.请合并下面两个字典 a = {"A":1,"B":2},b = {"C":3,"D":4}

39.如何使用生成式的方式生成一个字典，写一段功能代码。

40.如何把元组("a","b")和元组(1,2)，变为字典{"a":1,"b":2}




### 综合
41.Python 常用的数据结构的类型及其特性？

42.如何交换字典 {"A"：1,"B"：2}的键和值？

43.Python 里面如何实现 tuple 和 list 的转换？

44.我们知道对于列表可以使用切片操作进行部分元素的选择，那么如何对生成器类型的对象实现相同的功能呢？

45.请将[i for i in range(3)]改成生成器

46.a="hello"和 b="你好"编码成 bytes 类型

47.下面的代码输出结果是什么？

48.下面的代码输出的结果是什么?




### 操作类题目

49.Python 交换两个变量的值

50.在读文件操作的时候会使用 read、readline 或者 readlines，简述它们各自的作用

51.json 序列化时，可以处理的数据类型有哪些？如何定制支持 datetime 类型？

52.json 序列化时，默认遇到中文会转换成 unicode，如果想要保留中文怎么办？

53.有两个磁盘文件 A 和 B，各存放一行字母，要求把这两个文件中的信息合并(按字母顺序排列)，输出到一个新文件 C 中。

54.如果当前的日期为 20190530，要求写一个函数输出 N 天后的日期，(比如 N 为 2，则输出 20190601)。

55.写一个函数，接收整数参数 n，返回一个函数，函数的功能是把函数的参数和 n 相乘并把结果返回。

56.下面代码会存在什么问题，如何改进？



57.一行代码输出 1-100 之间的所有偶数。

58.with 语句的作用，写一段代码？

59.python 字典和 json 字符串相互转化方法

60.请写一个 Python 逻辑，计算一个文件中的大写字母数量

61. 请写一段 Python连接 Mongo 数据库，然后的查询代码。

62.说一说 Redis 的基本类型。

63. 请写一段 Python连接 Redis 数据库的代码。

64. 请写一段 Python 连接 MySQL 数据库的代码。

65.了解 Redis 的事务么？

66.了解数据库的三范式么？

67.了解分布式锁么？

68.用 Python 实现一个 Reids 的分布式锁的功能。

69.写一段 Python 使用 Mongo 数据库创建索引的代码。




### 高级特性

70.函数装饰器有什么作用？请列举说明？

71.Python 垃圾回收机制？

72.魔法函数 __call__ 怎么使用?

73.如何判断一个对象是函数还是方法？

74.@classmethod 和@staticmethod 用法和区别

75.Python 中的接口如何实现？

76.Python 中的反射了解么?

77.metaclass 作用？以及应用场景？

78.hasattr() getattr() setattr()的用法

79.请列举你知道的 Python 的魔法方法及用途。

80.如何知道一个 Python 对象的类型？

81.Python 的传参是传值还是传址？

82.Python 中的元类(metaclass)使用举例

83.简述 any()和 all()方法

84.filter 方法求出列表所有奇数并构造新列表，a = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

85.什么是猴子补丁？

86.在 Python 中是如何管理内存的？

87.当退出 Python 时是否释放所有内存分配？


### 正则表达式

88.使用正则表达式匹配出<html><h1>百度一下，你就知道</html>中的地址

a="张明 98 分"，用 re.sub，将 98 替换为 100

89.正则表达式匹配中(.*)和(.*?)匹配区别？

90.写一段匹配邮箱的正则表达式




### 其他内容

91.解释一下 python 中 pass 语句的作用？

92.简述你对 input()函数的理解

93.python 中的 is 和==

94.Python 中的作用域

95.三元运算写法和应用场景？

96.了解 enumerate 么？

97.列举 5 个 Python 中的标准模块

98.如何在函数中设置一个全局变量

99.pathlib 的用法举例

100.Python 中的异常处理，写一个简单的应用场景

101.Python 中递归的最大次数，那如何突破呢？

102.什么是面向对象的 mro

103.isinstance 作用以及应用场景？

104.什么是断言？应用场景？

105.lambda 表达式格式以及应用场景？

106.新式类和旧式类的区别

107.dir()是干什么用的？

108.一个包里有三个模块，demo1.py, demo2.py, demo3.py，但使用 `from tools import *` 导入模块时，如何保证只有 demo1、demo3 被导入了。

109.列举 5 个 Python 中的异常类型以及其含义

110.copy 和 deepcopy 的区别是什么？

111.代码中经常遇到的`*args`, `**kwargs` 含义及用法。

112.Python 中会有函数或成员变量包含单下划线前缀和结尾，和双下划线前缀结尾，区别是什么?

113.w、a+、wb 文件写入模式的区别

114.举例 sort 和 sorted 的区别

115.什么是负索引？

116.pprint 模块是干什么的？

117.解释一下 Python 中的赋值运算符

118.解释一下 Python 中的逻辑运算符

119.讲讲 Python 中的位运算符

120.在 Python 中如何使用多进制数字？

121.怎样声明多个变量并赋值？




### 算法和数据结构

122.已知：

(1) 从 AList 和 BSet 中 查找 4，最坏时间复杂度那个大？

(2) 从 AList 和 BSet 中 插入 4，最坏时间复杂度那个大？

123.用 Python 实现一个二分查找的函数

124.python 单例模式的实现方法

125.使用 Python 实现一个斐波那契数列

126.找出列表中的重复数字

127.找出列表中的单个数字

128.写一个冒泡排序

129.写一个快速排序

130.写一个拓扑排序

131.python 实现一个二进制计算

132.有一组“+”和“-”符号，要求将“+”排到左边，“-”排到右边，写出具体的实现方法。

133.单链表反转

134.交叉链表求交点

135.用队列实现栈

136.找出数据流的中位数

137.二叉搜索树中第 K 小的元素



###
网络编程

156.TCP 和 UDP 的区别？

157.简要介绍三次握手和四次挥手

158.什么是粘包？ socket 中造成粘包的原因是什么？ 哪些情况会发生粘包现象？




### 并发
159.举例说明 conccurent.future 的中线程池的用法

160.说一说多线程，多进程和协程的区别。

161.简述 GIL

162.进程之间如何通信

163.IO 多路复用的作用？

164.select、poll、epoll 模型的区别？

165.什么是并发和并行？

166.一个线程 1 让线程 2 去调用一个函数怎么实现？

167.解释什么是异步非阻塞？

168.threading.local 的作用？

### Git 面试题

169.说说你知道的 git 命令

170.git 如何查看某次提交修改的内容
