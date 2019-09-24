---
layout:     post
title:      "Design Patterns"
date:       2019-9-18
author:     Tong
catalog: true
tags:
    - Language
---

### Basics

* 一个模式有四个基本要素
    * 模式名
    * 问题
    * 解决方案
    * 效果


* 根据目的准则分类，即模式是用来完成什么工作的
    * 创建型 (creational)模式与对象的创建有关。(Facotry Method, Abstract Facotry, Builder, Prototype, Singleton)
    * 结构型 (structural)模式处理类或对象的组合。(Adapter (类), Adapter (对象), Bridge, Composite, Decorator, Facade, Flyweight, Proxy)
    * 行为型 (behavioral)模式对类或对象怎样交互和怎样分配职责进行描述。(Interpreter, Template Method, Chain of Responsibility, Command, Iterator, Mediator, Memento, Observer, State, Strategy, Visitor)

* 根据范围准则分类，即指定模式主要是用于类还是用于对象。
    * 类模式处理类和子类之间的关系，这些关系通过继承建立，是静态的，在编译时便确定下来了。(Facotry Method, Adapter, Interpreter, Template Method)
    * 对象模式处理对象间的关系，这些关系在运行时是可以变化的，更具动态性。(Abstract Facotry, Builder, Prototype, Singleton, Adapter, Bridge, Composite, Decorator, Facade, Flyweight, Proxy, Chain of Responsibility, Command, Iterator, Mediator, Memento, Observer, State, Strategy, Visitor)

* 面向对象设计的原则
    * 针对接口编程，而不是针对实现编程。
    * 优先使用对象组合，而不是类继承。

* Smalltalk MVC中的设计模式
    * 模型（Model）是应用对象。
    * 视图（View）是它在屏幕上的表示。
    * 控制器（Controller）定义用户界面对用户输入的相应方式。


### Abstract Factory (抽象工厂)

* 提供一个创建一系列相关或相互依赖对象的接口，而无须指定它们具体的类。

* 适用性：
    * 一个系统要独立于它的产品的创建，组合和表示
    * 一个系统要由多个产品系列中的一个来配置
    * 要强调一系列相关的产品对象的设计以便进行联合使用
    * 提供一个产品类库，但只想显式它们的接口而不是实现

* 参与者
    * AbstractFactory
    * ConcreteFactory
    * AbstractProduct
    * ConcreteProduct
    * Client

* `AbstractFactory`类通常由工厂方法（`Factory Method`）实现，但它们也可以用`Prototype`实现。

* 一个具体的工厂通常是一个单件（`Singleton`）。



### Adapter (适配器)

* 将一个类的接口转换成客户希望的另外一个接口。`Adapter`模式使得原本由于接口不兼容而不能一起工作的那些类可以一起工作。

### Bridge (桥接)

* 将抽象部分与它的实现部分分离，使它们都可以独立地变化。

### Builder (生成器)

* 将一个复杂对象的构建与它的表示分离，使得同样的构建过程可以创建不同的表示。

* 适用性：
    * 当创建复杂对象的算法应该独立于该对象的组成部分以及它们的装配方式时。
    * 当构造过程必须允许被构造的对象由不同的表示时。

* 参与者
    * Builder
    * ConcreteBuilder
    * Director
    * Product

* `Abstract Factory`与`Builder`相似，因为它也可以创建复杂对象。主要的区别是`Builder`模式着重于一步步构造一个复杂对象。而`Abstract Factory`着重于多个系列的产品对象（简单的或复杂的）。`Builder`在最后一步返回产品，而对于`Abstract Factory`i说，产品是立即返回的。

* `Composite`通常是由`Builder`生成的。

### Chain of Responsibility (职责链)

* 解除请求的发送者和接收者之间的耦合，使多个对象都有机会处理这个请求。将这些对象连成一条链，并沿着这条链传递该请求，直到有一个对象处理它。

### Command (命令)

* 将一个请求封装为一个对象，从而使你可用不同的请求对客户进行参数化；对请求排队或记录请求日志，以及支持可取消的操作。

### Composite (组合)

* 将对象组合成树形结构以表示“部分-整体”的层次结构。`Composite`使得客户对单个对象和组合对象的使用具有一致性。

### Decorator (装饰)

* 动态地给一个对象添加一些额外的职责。就扩展功能而言，`Decorator`模式比生成子类方式更为灵活。

### Facade (外观)

* 为子系统中的一组接口提供一个一致的界面，`Facade`模式定义了一个高层接口，这个接口使得这一子系统更加容易使用。

### Factory method (工厂方法)

* 定义一个用于创建对象的接口，让子类决定将哪一个类实例化。工厂方法使一个类的实例化延迟到其子类。

### Flyweight (享元)

* 运用共享技术有效地支持大量细粒度的对象。

### Interpreter (解释器)

* 给定一个语言，定义它的文法的一种表示，并定义一个解释器，该解释器使用该表示来解释语言中的句子。

### Iterator (迭代器)

* 提供一种方法顺序访问一个聚合对象中的各个元素，而又不需要暴露该对象的内部表示。

### Mediator (中介者)

* 用一个中介对象来封装一系列的对象交互。中介者使各对象不需要显式地相互引用，从而使其耦合松散，而且可以独立地改变它们之间的交互。

### Memento (备忘录)

* 在不破坏封装性的前提下，捕获一个对象的内部状态，并在该对象之外保存这个状态。这样以后就可将该对象恢复到保存的状态。

### Observer (观察者)

* 定义对象间的一种一对多的以来关系，以便当一个对象的状态发生改变时，所有依赖于它的对象都得到通知并自动刷新。

### Prototype (原型)

* 用原型实例指定创建对象的种类，并且通过拷贝这个原型来创建新的对象。

### Proxy (代理)

* 为其他对象提供一个代理以控制对这个对象的访问。

### Singleton (单件)

* 保证一个类仅有一个实例，并提供一个访问它的全局访问点。

### State (状态)

* 允许一个对象在其内部状态改变时改变它的行为。对象看起来似乎修改了它所属的类。

### Strategy (策略)

* 定义一系列的算法，把它们一个个封装起来，并且使它们可相互替换。本模式使得算法的变化可独立于使用它的客户。

### Template method (模板方法)

* 定义一个操作中的算法的骨架，而将一些步骤延迟到子类中。模板方法使得子类不改变一个算法的结构即可重定义该算法的某些特定步骤。

### Visitor (访问者)

* 表示一个作用于某对象结构中的各元素的操作。它使你可以在不改变各元素的类的前提下定义作用域这些元素的新操作。                     
