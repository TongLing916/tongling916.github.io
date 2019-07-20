---
layout:     post
title:      "Sort Overview"
date:       2019-3-27
author:     Tong
catalog: true
tags:
    - Algorithm
---

### Summary

1. 如何比较不同排序算法？
  (1) 共比较了多少次
  (2) 共交换了多少次

2. Comparison of sorting algorithms
![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/sorting_comparison.png)

### [Bubble sort](https://runestone.academy/runestone/static/pythonds/SortSearch/TheBubbleSort.html) 冒泡排序

#### Traditional bubble sort

```python
def bubbleSort(alist):
    for passnum in range(len(alist)-1,0,-1):
        for i in range(passnum):
            if alist[i]>alist[i+1]:
                # The following three lines can be writte in one line
                # alist[i], alist[i+1] = alist[i+1], alist[i]
                temp = alist[i]
                alist[i] = alist[i+1]
                alist[i+1] = temp

alist = [54,26,93,17,77,31,44,55,20]
bubbleSort(alist)
print(alist)
```

#### Short bubble sort

```python
# Stop when recognizing the sorted list, 许多排序算法都没有这个特性
def shortBubbleSort(alist):
    exchanges = True
    passnum = len(alist)-1
    while passnum > 0 and exchanges:
       exchanges = False
       for i in range(passnum):
           if alist[i]>alist[i+1]:
               exchanges = True
               temp = alist[i]
               alist[i] = alist[i+1]
               alist[i+1] = temp
       passnum = passnum-1

alist=[20,30,40,90,50,60,70,80,100,110]
shortBubbleSort(alist)
print(alist)
```
### [Selection sort](https://runestone.academy/runestone/static/pythonds/SortSearch/TheSelectionSort.html) 选择排序

```python
def selectionSort(alist):
   for fillslot in range(len(alist)-1,0,-1):
       positionOfMax=0
       for location in range(1,fillslot+1):
           if alist[location]>alist[positionOfMax]:
               positionOfMax = location

       temp = alist[fillslot]
       alist[fillslot] = alist[positionOfMax]
       alist[positionOfMax] = temp

alist = [54,26,93,17,77,31,44,55,20]
selectionSort(alist)
print(alist)
```

### [Insertion sort](https://runestone.academy/runestone/static/pythonds/SortSearch/TheInsertionSort.html) 插入排序

```python
def insertionSort(alist):
   for index in range(1,len(alist)):

     currentvalue = alist[index]
     position = index

     while position>0 and alist[position-1]>currentvalue:
         alist[position]=alist[position-1]
         position = position-1

     alist[position]=currentvalue

alist = [54,26,93,17,77,31,44,55,20]
insertionSort(alist)
print(alist)
```

### [Shell sort](https://runestone.academy/runestone/static/pythonds/SortSearch/TheShellSort.html) 希尔排序

```python
def shellSort(alist):
    sublistcount = len(alist)//2
    while sublistcount > 0:

      for startposition in range(sublistcount):
        gapInsertionSort(alist,startposition,sublistcount)

      print("After increments of size",sublistcount,
                                   "The list is",alist)

      sublistcount = sublistcount // 2

def gapInsertionSort(alist,start,gap):
    for i in range(start+gap,len(alist),gap):

        currentvalue = alist[i]
        position = i

        while position>=gap and alist[position-gap]>currentvalue:
            alist[position]=alist[position-gap]
            position = position-gap

        alist[position]=currentvalue

alist = [54,26,93,17,77,31,44,55,20]
shellSort(alist)
print(alist)
```

### [Merge sort](https://runestone.academy/runestone/static/pythonds/SortSearch/TheMergeSort.html) 归并排序

```python
def mergeSort(alist):
    print("Splitting ",alist)
    if len(alist)>1:
        mid = len(alist)//2
        lefthalf = alist[:mid]  #Recall that the slicing operator is O(k) where k is the size of the slice. In order to guarantee that mergeSort will be O(nlogn) we will need to remove the slice operator. Again, this is possible if we simply pass the starting and ending indices along with the list when we make the recursive call. We leave this as an exercise.
        righthalf = alist[mid:] #It is important to notice that the mergeSort function requires extra space to hold the two halves as they are extracted with the slicing operations

        mergeSort(lefthalf)
        mergeSort(righthalf)

        i=0
        j=0
        k=0
        while i < len(lefthalf) and j < len(righthalf):
            if lefthalf[i] <= righthalf[j]: #A stable algorithm maintains the order of duplicate items in a list and is preferred in most cases.
                alist[k]=lefthalf[i]
                i=i+1
            else:
                alist[k]=righthalf[j]
                j=j+1
            k=k+1

        while i < len(lefthalf):
            alist[k]=lefthalf[i]
            i=i+1
            k=k+1

        while j < len(righthalf):
            alist[k]=righthalf[j]
            j=j+1
            k=k+1
    print("Merging ",alist)

alist = [54,26,93,17,77,31,44,55,20]
mergeSort(alist)
print(alist)
```


### [Quick sort](https://runestone.academy/runestone/static/pythonds/SortSearch/TheQuickSort.html) 快速排序

```python
def quickSort(alist):
   quickSortHelper(alist,0,len(alist)-1)  # How to choose pivot? Median of three

def quickSortHelper(alist,first,last):
   if first<last:

       splitpoint = partition(alist,first,last)

       quickSortHelper(alist,first,splitpoint-1)
       quickSortHelper(alist,splitpoint+1,last)


def partition(alist,first,last):
   pivotvalue = alist[first]

   leftmark = first+1
   rightmark = last

   done = False
   while not done:

       while leftmark <= rightmark and alist[leftmark] <= pivotvalue:
           leftmark = leftmark + 1

       while alist[rightmark] >= pivotvalue and rightmark >= leftmark:
           rightmark = rightmark -1

       if rightmark < leftmark:
           done = True
       else:
           temp = alist[leftmark]
           alist[leftmark] = alist[rightmark]
           alist[rightmark] = temp

   temp = alist[first]
   alist[first] = alist[rightmark]
   alist[rightmark] = temp


   return rightmark

alist = [54,26,93,17,77,31,44,55,20]
quickSort(alist)
print(alist)
```

```cpp
#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

class QuickSort
{
public:
	void sort(vector<int>& nums)
	{
		int n = nums.size();
		if (n <= 1) return;
		random_shuffle(nums.begin(), nums.end());
		sort(nums, 0, n - 1);  
	}
	void sort(vector<int>& nums, int lo, int hi)   //for small subarrays, we can consider use insertion sort
	{
		if (hi <= lo) return;
		int pivot = partition(nums, lo, hi);
		sort(nums, lo, pivot - 1);
		sort(nums, pivot + 1, hi);
	}
	int partition(vector<int>& nums, int lo, int hi)
	{
		int i = lo, j = hi + 1;
		int v = nums[lo];               //we can find the median and put it in the lo position.
		while (1)
		{
			while (nums[++i] < v)
				if (i == hi)
					break;
			while (nums[--j] > v)
				if (j == lo)
					break;
			if (i >= j)
				break;
			swap(nums[i], nums[j]);
		}
		swap(nums[lo], nums[j]);
		return j;
	}

};

//median-of-3
//small subarrays sorted using insertion sort
class QuickSortOpt
{
public:
	const int INSERTION_SORT_CUTOFF = 8;
	void sort(vector<int>& nums)
	{
		int n = nums.size();
		if (n <= 1) return;
		random_shuffle(nums.begin(), nums.end());
		sort(nums, 0, n - 1);
	}
	void sort(vector<int> & nums, int lo, int hi)   //for small subarrays, we can consider use insertion sort
	{
		if (hi <= lo) return;

		int n = hi - lo + 1;
		if (n <= INSERTION_SORT_CUTOFF)
		{
			insertionSort(nums, lo, hi);
			return;
		}

		int pivot = partition(nums, lo, hi);
		sort(nums, lo, pivot - 1);
		sort(nums, pivot + 1, hi);
	}
	int partition(vector<int> & nums, int lo, int hi)
	{
		int i = lo, j = hi + 1;
		int m = median3(nums, lo, lo + (j - i) / 2, hi);
		swap(nums[lo], nums[m]);
		int v = nums[lo];               //we can find the median and put it in the lo position.
		while (1)
		{
			while (nums[++i] < v)
				if (i == hi)
					break;
			while (nums[--j] > v)
				if (j == lo)
					break;
			if (i >= j)
				break;
			swap(nums[i], nums[j]);
		}
		swap(nums[lo], nums[j]);
		return j;
	}
	void insertionSort(vector<int>& nums, int lo, int hi)
	{
		for (int i = 1; i < nums.size(); ++i)
			for (int j = i; j > 0 && nums[j] < nums[j - 1]; --j)
				swap(nums[j - 1], nums[j]);
	}
	int median3(vector<int>& nums, int i, int j, int k)
	{
		if (nums[i] <= nums[j] && nums[j] <= nums[k])
			return j;
		else if (nums[j] <= nums[i] && nums[i] <= nums[k])
			return i;
		else
			return k;
	}
};

class Quick3Way  //Consider the equal elements
{
public:
	void sort(vector<int>& nums)
	{
		int n = nums.size();
		if (n <= 1) return;
		random_shuffle(nums.begin(), nums.end());
		sort(nums, 0, n - 1);
	}
	void sort(vector<int>& nums, int lo, int hi)
	{
		if (hi <= lo) return;
		int lt = lo, i = lo + 1, gt = hi;
		int v = nums[lo];
		while (i <= gt)
		{
			if (nums[i] < v)
				swap(nums[lt++], nums[i++]);
			else if (nums[i] > v)
				swap(nums[i], nums[gt--]);
			else
				++i;
		}
		sort(nums, lo, lt - 1);
		sort(nums, gt + 1, hi);
	}
};


int main()
{
	vector<int> test1{ 9,7,6,5,4,3,2,1 };
	vector<int> test2{ 5,7,6,5,4,3,5,5 };
	vector<int> test3{ 5,7,6,5,4,3,5,5,10,9,8,2,12,15,29,10,28,30,32 };
	QuickSort quickSort;
	QuickSortOpt quickSortOpt;
	Quick3Way quick3Way;
	quickSort.sort(test1);
	quick3Way.sort(test2);
	quickSortOpt.sort(test3);
	for (auto& num : test3)
		cout << num << " ";
	cout << endl;
}
```

### [Heap sort](https://www.geeksforgeeks.org/heap-sort/) 堆排序 [Algorithms by Princeton University, P. 323]

```python
# https://www.geeksforgeeks.org/heap-sort/
# Python program for implementation of heap Sort

# To heapify subtree rooted at index i.
# n is size of heap
def heapify(arr, n, i):
    largest = i # Initialize largest as root
    l = 2 * i + 1     # left = 2*i + 1
    r = 2 * i + 2     # right = 2*i + 2

    # See if left child of root exists and is
    # greater than root
    if l < n and arr[i] < arr[l]:
        largest = l

    # See if right child of root exists and is
    # greater than root
    if r < n and arr[largest] < arr[r]:
        largest = r

    # Change root, if needed
    if largest != i:
        arr[i],arr[largest] = arr[largest],arr[i] # swap

        # Heapify the root.
        heapify(arr, n, largest)

# The main function to sort an array of given size
def heapSort(arr):
    n = len(arr)

    # Build a maxheap.
    for i in range(n, -1, -1):
        heapify(arr, n, i)

    # One by one extract elements
    for i in range(n-1, 0, -1):
        arr[i], arr[0] = arr[0], arr[i] # swap
        heapify(arr, i, 0)

# Driver code to test above
arr = [ 12, 11, 13, 5, 6, 7]
heapSort(arr)
n = len(arr)
print ("Sorted array is")
for i in range(n):
    print ("%d" %arr[i]),
# This code is contributed by Mohit Kumra

```

```python
# The complete binary heap
class BinHeap:
    def __init__(self):
        self.heapList = [0]
        self.currentSize = 0


    def percUp(self,i):
        while i // 2 > 0:
          if self.heapList[i] < self.heapList[i // 2]:
             tmp = self.heapList[i // 2]
             self.heapList[i // 2] = self.heapList[i]
             self.heapList[i] = tmp
          i = i // 2

    def insert(self,k):
      self.heapList.append(k)
      self.currentSize = self.currentSize + 1
      self.percUp(self.currentSize)

    def percDown(self,i):
      while (i * 2) <= self.currentSize:
          mc = self.minChild(i)
          if self.heapList[i] > self.heapList[mc]:
              tmp = self.heapList[i]
              self.heapList[i] = self.heapList[mc]
              self.heapList[mc] = tmp
          i = mc

    def minChild(self,i):
      if i * 2 + 1 > self.currentSize:
          return i * 2
      else:
          if self.heapList[i*2] < self.heapList[i*2+1]:
              return i * 2
          else:
              return i * 2 + 1

    def delMin(self):
      retval = self.heapList[1]
      self.heapList[1] = self.heapList[self.currentSize]
      self.currentSize = self.currentSize - 1
      self.heapList.pop()
      self.percDown(1)
      return retval

    def buildHeap(self,alist):
      i = len(alist) // 2
      self.currentSize = len(alist)
      self.heapList = [0] + alist[:]
      while (i > 0):
          self.percDown(i)
          i = i - 1

bh = BinHeap()
bh.buildHeap([9,5,6,2,3])

print(bh.delMin())
print(bh.delMin())
print(bh.delMin())
print(bh.delMin())
print(bh.delMin())
```

### [Timsort](https://hackernoon.com/timsort-the-fastest-sorting-algorithm-youve-never-heard-of-36b28417f399)

1. https://en.wikipedia.org/wiki/Timsort
