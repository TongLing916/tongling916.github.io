---
layout: post
title: "Appendix 1: Tensor Notation"
date:       2018-12-17
author:     Tong
catalog: true
tags:
    - MVG
---

> All contents come from <<Multiple View Geometry in Computer Vision>>.

### Abstract

In general, tensor indices will transform by either $$H$$ (subscript) or $$H^{-1}$$ (superscript) - in fact this is the characteristic of a tensor. Those indices that transform according to $$H$$ are known as _covariant_ indices and those according to $$H^{-1}$$ are known as _contravariant_ indices. The number of indices is the _valency_ of the tensor. The sum over an index, e.g. $$H_i^jl_j$$, is referred to as a _contraction_.

### A1.1 The tensor $$\epsilon_{rst}$$

1. The tensor $$\epsilon _ {rst}$$ is defined for $$r,s,t = 1,...,3$$ as follows:$$\epsilon _ {rst} = \left\{\begin{matrix}0 \quad unless\;r,\;s\;and\;t\;are\;distinct\\ +1 \quad if\; rst\; is \; an \; even \; permutation \; of \; 123\\-1 \quad if\; rst\; is \; an \; odd \; permutation \; of \; 123\end{matrix}\right.$$

### A1.2 The trifocal tensor

1. The trifocal tensor $$\mathcal{T}_i^{jk}$$ has one covariant and two contravariant indices. 
