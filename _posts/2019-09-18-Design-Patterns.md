---
layout:     post
title:      "Design Patterns"
date:       2019-9-18
author:     Tong
catalog: true
tags:
    - Language
---

> <<Design Patterns: Elements of Reusable Object-Oriented Software>>

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



### 创建型模式

* 用一个系统创建的那些对象的类对系统进行参数化有两种方法
    * 一种是生成创建对象的类的子类，这对应于使用`Factory Method`模式。缺点：仅为了改变产品类，就可能需要创建一个新的子类。
    * 另一种更依赖于对象组合：定义一个对象负责明确产品对象的类，并将它作为该系统的参数。这是`Abstract Factory`，`Builder`和`Prototype`模式的关键特征。它们都涉及创建一个新的负责创建产品对象的“工厂对象”。`Abstract Factory`由这个工厂对象产生多个类的对象。`Builder`使用一个相对复杂的协议，由这个工厂对象逐步创建一个复杂产品。`Prototype`由该工厂对象通过拷贝原型对象来创建产品。

### 结构性模式

* `Adapter`主要是为了解决两个已有不同接口之间不匹配的问题。它不考虑这些接口是怎么实现的，也不考虑它们各自可能会如何演化。这种方式不需要对两个独立设计的类中的任一个进行重新设计，就能够使它们协同工作。该模式在类已经设计好后实施。

* `Bridge`则对抽象接口与它的（可能是多个）实现部分进行桥接。虽然这一模式允许你修改实现它的类，但是它仍然为用户提供了一个稳定的接口。该模式在设计类之前实施。

* `Facade`定义一个新的接口，而`Adapter`则复用一个原有的接口。

* `Composite`在于构造类，使多个相关的对象能够以统一的方式处理，而多个对象可以被当作一个对象来处理。它的重点不在于修饰，而在于表示。

* `Decorator`目的在于使你不需要生成子类即可给对象添加职责。这就避免了静态实现所有功能组合，从而导致子类急剧增加。

* `Proxy`像`Decorator`一样，可以构成一个对象并为用户提供一致的接口。但与`Decorator`不同的是，`Proxy`不能动态地添加或分离性质，它也不是为递归组合而设计的。它的目的是，当直接访问一个实体不方便或不符合需求时，为这个实体提供一个替代者。



### Abstract Factory (抽象工厂)

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/abstract_factory.PNG)

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

* 类适配器使用多重继承对一个接口与另一个接口进行匹配。

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/class_adapter.PNG)

* 对象匹配器依赖于对象组合。

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/object_adapter.PNG)

* 意图：将一个类的接口转换成客户希望的另外一个接口。`Adapter`模式使得原本由于接口不兼容而不能一起工作的那些类可以一起工作。

* 适用性：
    * 你想使用一个已经存在的类，而它的接口不符合你的需求。
    * 你想创建一个可以复用的类，该类可以与其他不相关的类或不可预见的类（即那些接口可能不一定兼容的类）协同工作。
    * （仅适用于对象Adapter）你想使用一些已经存在的子类，但是不可能对每一个都进行子类化以匹配它们的接口。对象适配器可以适配它的父类接口。

* 参与者
    * Target
    * Client
    * Adaptee
    * Adapter

### Bridge (桥接)

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/bridge.PNG)

* 将抽象部分与它的实现部分分离，使它们都可以独立地变化。

* 适用性
    * 你不希望在抽象和它的实现部分之间有一个固定的绑定关系。例如，这种情况可能是因为，在程序运行时实现部分应该可以被选择或者切换。
    * 类的抽象以及它的实现都应该可以通过生成子类的方法加以扩充。这时`Bridge`模式使你可以对不同的抽象接口和实现部分进行组合，并分别对他们进行扩充。
    * 对一个抽象的实现部分的修改应对客户不产生影响，即客户的代码不必重新编译。
    * （C++）你想对客户完全隐藏抽象的实现部分。在C++中，类的表示在类接口中是可见的。

* 参与者
    * Abstraction
    * RefinedAbstraction
    * Implementor
    * ConcreteImplementor




### Builder (生成器)

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/builder.PNG)

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

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/chain_of_responsibility.PNG)


* 解除请求的发送者和接收者之间的耦合，使多个对象都有机会处理这个请求。将这些对象连成一条链，并沿着这条链传递该请求，直到有一个对象处理它。

* 适用性
    * 有多个对象可以处理一个请求，哪个对象处理该请求运行时自动确定。
    * 你想在不明确指定接收者的情况下，向多个对象中的一个提交一个请求。
    * 可处理一个请求的对象集合应被动态指定。

* 参与者
    * Handler
    * ConcreteHandler
    * Client

### Command (命令)

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/command.PNG)

* 适用性
    * 抽象出待执行的动作以参数化某对象。
    * 在不同的时刻指定，排列和执行请求。
    * 支持取消操作。

* 参与者
    * Command
    * ConcreteCommand
    * Client
    * Invoker
    * Receiver

* 将一个请求封装为一个对象，从而使你可用不同的请求对客户进行参数化；对请求排队或记录请求日志，以及支持可取消的操作。

### Composite (组合)

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/composite.PNG)

* 将对象组合成树形结构以表示“部分-整体”的层次结构。`Composite`使得客户对单个对象和组合对象的使用具有一致性。

* 适用性
    * 你想表示对象的部分-整体层次结构。
    * 你希望用户忽略组合对象和单个对象的不同，用户将统一地使用组合结构中的所有对象。

* 参与者
    * Component
    * Leaf
    * Composite
    * Client

### Decorator (装饰)

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/decorator.PNG)

* 动态地给一个对象添加一些额外的职责。就扩展功能而言，`Decorator`模式比生成子类方式更为灵活。

* 适用性
    * 在不影响其他对象的情况下，以动态，透明的方式给单个对象添加职责。
    * 处理那些可以撤销的职责。
    * 当不能采用生成子类的方法进行扩充时。一种情况是，可能有大量独立的扩展，为支持每一种组合将产生大量的子类，使得子类数目呈爆炸性增长。另一种可能情况可能是，类定义被隐藏，或类定义不能用于生成子类。

* 参与者
    * Component
    * ConcreteComponent
    * Decorator
    * ConcreteDecorator

### Facade (外观)

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/facade.PNG)

* 为子系统中的一组接口提供一个一致的界面，`Facade`模式定义了一个高层接口，这个接口使得这一子系统更加容易使用。

* 适用性
    * 当你要为一个复杂子系统提供一个简单接口时。
    * 客户程序与抽象类的实现部分之间存在着很大的依赖性。
    * 当你需要构建一个层次结构的子系统时，使用`Facade`模式定义子系统中每层的入口点。如果子系统之间是相互依赖的，可以让它们仅通过`Facade`进行通信，从而简化了它们之间的依赖关系。

* 参与者
    * Facade
    * Subsystem classes


### Factory method (工厂方法)

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/factory_method.PNG)

* 定义一个用于创建对象的接口，让子类决定将哪一个类实例化。工厂方法使一个类的实例化延迟到其子类。

* 适用性
    * 当一个类不知道它所必须创建的对象的类的时候。
    * 当一个类希望由它的子类来指定它所创建的对象的时候。
    * 当类将创建对象的职责委托给多个帮助子类中的某一个，并且你希望将哪一个帮助子类是代理者这一信息局部化的时候。

* 参与者
    * Product
    * ConcreteProduct
    * Creator
    * ConcreteCreator

* 工厂方法主要有两种不同的情况
    * Creator类是一个抽象类，并且不提供它所声明的工厂方法的实现。
    * Creator是一个具体的类而且为工厂方法提供第一个缺省的实现。也有可能有一个定义了缺省实现的抽象类。

* C++中的工厂方法都是虚函数并且常常是纯虚函数。一定要注意在Creator的构造器中不要调用工厂方法---ConcreteCreator中该工厂方法还不可用。

* 惰性初始化 (lazy initialization): 构造器只是将产品初始化为0，而不是创建一个具体产品。访问者返回该产品。但首先它要检查确定该产品的存在，如果产品不存在，访问者就创建它。

```c++
class Creator
{
public:
	Product* GetProduct();
protected:
	virtual Product* CreateProduct();
private:
	Product* _product;
};

Product* Creator::GetProduct()
{
	if (_product == 0)
		_product = CreateProduct();

	return _product;
}
```
__

* 工厂方法另一个潜在的问题是它们可能仅为了创建适当的Product对象而迫使你创建Creator子类。在C++中另一个解决方法是提供一个模板子类，它使用Product类作为模板参数。

```c++
class Creator
{
public:
	virtual Product* CreateProduct() = 0;
};

template <class TheProduct>
class StandardCreator : public Creator
{
public:
	virtual Product* CreateProduct();
};

template <class TheProduct>
Product* StandardCreator<TheProduct>::CreateProduct()
{
	return new TheProduct;
}
```

* `Abstract Factory`经常用`工厂方法`来实现。

* `工厂方法`通常在`Template Method`被调用。

* `Prototype`不需要创建Creator的子类。但是它们通常要求一个针对Product类的Initialize操作。Creator使用Initialize来初始化对象，而`Factory Method`不需要这样的操作。


### Flyweight (享元)

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/flyweight.PNG)

* 运用共享技术有效地支持大量细粒度的对象。

* 当以下情况都成立时使用`Flyweight`模式：
    * 一个应用程序使用了大量的对象。
    * 完全由于使用大量的对象造成很大的存储开销。
    * 对象的大多数状态都可变为外部状态。
    * 如果删除对象的外部状态，那么可以用相对较少的共享对象取代很多组对象。
    * 应用程序不依赖于对象标识。由于`Flyweight`对象可以被共享，因此对于概念上明显有别的对象，标识测试将返回真值。

* 参与者
    * Flyweight
    * ConcreteFlyweight
    * UnsharedConcreteFlyweight
    * FlyweightFactory
    * Client

### Interpreter (解释器)

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/interpreter.PNG)

* 给定一个语言，定义它的文法的一种表示，并定义一个解释器，该解释器使用该表示来解释语言中的句子。

* 适用性：当一个语言需要解释执行，并且你可将该语言中的句子表示为一个抽象语法树时，可使用解释器模式。
    * 文法简单。
    * 效率不是问题。

* 参与者
    * AbstractExpression
    * TerminalExpression
    * NonterminalExpression
    * Context
    * Client

### Iterator (迭代器)

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/iterator.PNG)

* 提供一种方法顺序访问一个聚合对象中的各个元素，而又不需要暴露该对象的内部表示。

* 适用性
    * 访问一个聚合对象的内容而无须暴露它的内部表示。
    * 支持对聚合对象的多种遍历。
    * 为遍历不同的聚合结构提供一个统一的接口（即支持多态迭代）。

* 参与者
    * Iterator
    * ConcreteIterator
    * Aggregate
    * ConcreteAggregate


### Mediator (中介者)

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/mediator.PNG)

* 用一个中介对象来封装一系列的对象交互。中介者使各对象不需要显式地相互引用，从而使其耦合松散，而且可以独立地改变它们之间的交互。

* 适用性
    * 一组对象以定义良好但复杂的方式进行通信，产生的相互依赖关系结构混乱且难以理解。
    * 一个对象引用其他很多对象并且直接与这些对象通信，导致难以复用该对象。
    * 想定制一个分布在多个类中的行为，而又不想生成太多的子类。

* 参与者
    * Mediator
    * ConcreteMediator
    * Colleague class

### Memento (备忘录)

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/memento.PNG)

* 在不破坏封装性的前提下，捕获一个对象的内部状态，并在该对象之外保存这个状态。这样以后就可将该对象恢复到保存的状态。

* 适用性
    * 必须保存一个对象在某个时刻的（部分）状态，这样以后需要时它才能恢复到先前的状态。
    * 如果一个接口让其他对象直接得到这些状态，将会暴露对象的实现细节并破坏对象的封装性。

* 参与者
    * Memento
    * Originator
    * Caretaker

### Observer (观察者)

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/observer.PNG)

* 定义对象间的一种一对多的以来关系，以便当一个对象的状态发生改变时，所有依赖于它的对象都得到通知并自动刷新。

* 适用性
    * 一个抽象模型有两个方面，其中一个方面依赖于另一方面。将这二者封装在独立的对象中，以使它们可以独自地改变和复用。
    * 对一个对象的改变需要同时改变其他对象，而不知道具体有多少对象有待改变。
    * 一个对象必须通知其他对象，而它又不能假定其他对象是谁。换言之，你不希望这些对象是紧密耦合的。

* 参与者
    * Subject
    * Observer
    * ConcreteSubject
    * ConcreteObserver

### Prototype (原型)

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/prototype.PNG)

* 用原型实例指定创建对象的种类，并且通过拷贝这个原型来创建新的对象。

* 适用性：
    * 当一个系统应该独立于它的产品创建，构成和表示时。
    * 当要实例化的类是在运行时指定时，例如，通过动态装载。
    * 为了避免创建一个与产品类平行的工厂类层次时。
    * 当一个类的实例只能有几个不同状态组合中的一种时。建立相应数目的原型并克隆它们可能比每次用合适的状态手工实例化该类更方便一些。

* 参与者
    * Prototype
    * ConcretePrototype
    * Client



### Proxy (代理)

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/proxy.PNG)

* 为其他对象提供一个代理以控制对这个对象的访问。

* 适用性
    * 远程代理为一个对象在不同的地址空间提供局部代表。
    * 虚代理根据需要创建开销很大的对象。
    * 保护代理控制对原始对象的访问。
    * 智能指引取代了简单的指针。

* 参与者
    * Proxy
    * Subject
    * RealSubject

### Singleton (单件)

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/singleton.PNG)

* 保证一个类仅有一个实例，并提供一个访问它的全局访问点。

* 适用性：
    * 当类只能有一个实例，而且客户可以从一个众所周知的访问点访问它时。
    * 当这个唯一实例应该是通过子类化可扩展的，并且客户应该无须更改代码就能使用一个扩展的实例时。

* 参与者
    * Singleton

```c++
class Singleton
{
public:
	static Singleton* Instance();
protected:
	Singleton();
private:
	static Singleton* _instance;
};

Singleton* Singleton::_instance = 0;

Singleton* Singleton::Instance()
{
	if (_instance == 0) _instance = new Singleton;
	return _instance;
}  
```

* Instance使用惰性初始化，它的返回值直到被第一次访问时才创建和保存。

* 构造器是protected。试图直接实例化Singleton的客户将得到又给编译时的错误信息。这就保证了仅有一个实例可以被创建。


### State (状态)

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/state.PNG)

* 允许一个对象在其内部状态改变时改变它的行为。对象看起来似乎修改了它所属的类。

* 适用性
    * 一个对象的行为取决于它的状态，并且它必须在运行时根据状态改变它的行为。
    * 一个操作中含有庞大的多分支的条件语句，且这些分支依赖于该对象的状态。

* 参与者
    * Context
    * State
    * ConcreteState subclasses

### Strategy (策略)

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/strategy.PNG)

* 定义一系列的算法，把它们一个个封装起来，并且使它们可相互替换。本模式使得算法的变化可独立于使用它的客户。

* 适用性
    * 许多相关的类仅仅是行为有异。
    * 需要使用一个算法的不同变体。
    * 算法使用客户不应该直到的数据。
    * 一个类定义了多种行为，并且这些行为在这个类的操作中以多个条件语句的形式出现。

* 参与者
    * Strategy
    * ConcreteStrategy
    * Context

### Template method (模板方法)

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/template_method.PNG)

* 定义一个操作中的算法的骨架，而将一些步骤延迟到子类中。模板方法使得子类不改变一个算法的结构即可重定义该算法的某些特定步骤。

* 适用性
    * 一次性实现一个算法的不变部分，并将可变的行为留给子类来实现。
    * 各子类中的公共的行为应被提取出来并集中到一个公共父类中以避免代码重复。
    * 控制子类扩展。

* 参与者
    * AbstractClass
    * ConcreteClass

### Visitor (访问者)

![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/visitor.PNG)

* 表示一个作用于某对象结构中的各元素的操作。它使你可以在不改变各元素的类的前提下定义作用域这些元素的新操作。       

* 适用性
    * 一个对象结构包含很多类对象，它们有不同的接口，而你想对这些对象实施一些依赖于其具体类的操作。
    * 需要对一个对象结构中的对象进行很多不同并且不相关的操作，而你想避免让这些操作“污染”这些对象的类。
    * 定义对象结构的类很少改变，但经常需要在此结构上定义新的操作。

* 参与者              
    * Visitor
    * ConcreteVisitor
    * Element
    * ConcreteElement
    * ObjectStructure
