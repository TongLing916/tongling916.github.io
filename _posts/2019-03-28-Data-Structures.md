---
layout:     post
title:      "Data structures"
date:       2019-3-27
author:     Tong
catalog: true
tags:
    - Algorithm
---

### 真题

1. [树的遍历](https://runestone.academy/runestone/static/pythonds/Trees/TreeTraversals.html)
  - preorder:
    In a preorder traversal, we visit the root node first, then recursively do a preorder traversal of the left subtree, followed by a recursive preorder traversal of the right subtree.
  - inorder:
    In an inorder traversal, we recursively do an inorder traversal on the left subtree, visit the root node, and finally do a recursive inorder traversal of the right subtree.
  - postorder:
    In a postorder traversal, we recursively do a postorder traversal of the left subtree and the right subtree followed by a visit to the root node.

2. 下面哪个不是线性表? ()
<ol type="A">
  <li>循环链表</li>
  <li>队列</li>
  <li>栈</li>
  <li>关联数组</li>
  <li>空字符串数组</li>
  <li>双向链表</li>
</ol>

> Solution: D. <br>
>  A.循环链表是另一种形式的链式存贮结构。它的特点是表中最后一个结点的 指针 域指向 头结点 ，整个链表形成一个环。(1）单循环链表——在单链表中，将终端结点的指针域NULL改为指向表头结点或开始结点即可。
2）多重链的循环链表——将表中结点链在多个环上。<br>
B. 队列（Queue）是只允许在一端进行插入，而在另一端进行删除的运算受限的线性表 <br>
C. 栈（stack）在计算机科学中是限定仅在栈顶进行插入或删除操作的线性表 <br>
D. “关联数组”是一种具有特殊索引方式的数组。不仅可以通过整数来索引它，还可以使用字符串或者其他类型的值（除了NULL）来索引它。   详情查看：   http://baike.baidu.com/link?url=yYrNB5t4PrCvs-XfxfEM0ZZfALpsEi3FYopk1v0BuopUSWOr7mS0Lou8C-SzhDnSuv7BH5vKIoIblvi8GgUmGq
       关联数组和数组类似，由以名称作为键的字段和方法组成。   它包含标量数据，可用索引值来单独选择这些数据，和数组不同的是， 关联数组的索引值不是非负的整数而是任意的标量。这些标量称为Keys，可以在以后用于检索数组中的数值。
       关联数组的元素没有特定的顺序，你可以把它们想象为一组卡片。每张卡片上半部分是索引而下半部分是数值。<br>
E.链表（Linked list）是一种常见的基础数据结构，是一种线性表，是一种物理存储单元上非连续、非顺序的存储结构。双向链表也叫 双链表 ，是链表的一种，它的每个数据结点中都有两个 指针 ，分别指向直接后继和直接前驱。所以，从双向链表中的任意一个结点开始，都可以很方便地访问它的前驱结点和后继结点。一般我们都构造双向 循环链表 。 
