---
layout:     post
title:      "Minimum Spanning Tree"
date:       2019-8-11
author:     Tong
catalog: true
tags:
    - Algorithm
---

### Summary

#### 前提条件

1. 图必须是连通的。如果图没连通，那么就不存在MST。

2. 边的权重可能是0或者是附属。

3. 只有边的权重不是唯一的，那么可能有不止一颗MST。


#### 试题

1. Prim和Kruskal算法不适用于有向图。

2. 通常，Kruskal算法稍微慢一些，因为对于每条边，我们都要调用`connected()`算法。

3. 与克鲁斯卡尔（Kruskal）相比，普里姆（Prim）算法更适于求 **边稠密的网** 的最小生成树。

4. 如果存在一些负的边权，那么Prim算法或Kruskal算法还能行得通。

5. 最小生成树的代价唯一。

6. 所有权值最小的边不一定会出现在所有的最小生成树中。

7. 使用普里姆（Prim）算法从不同顶点开始得到的最小生成树不一定相同。

8. 使用普里姆算法和克鲁斯卡尔（Kruskal）算法得到的最小生成树不一定会相同。

9. 假设在某个图中，所有的边的权值都一致分布在半开区间[1, 0)之间。对于Kruskal 和Prim这两个算法，你可以让哪一个运行得更快些？
以让Kruskal 算法运行的更快些。由于权值均匀分布在[1,0)区间内，可以使用桶排序排序权值。这样Kruskal算法的时间复杂度为O(|E|a(|V|))。
ps:
- 顺便说下Kruskal和Prim算法加速问题：如果权值处于[1, |V|]之间，排序可以用计数排序，这样时间Kruskal时间复杂度为O(|E|a(|V|))。
- 由于Prim算法时间主要耗费在维护优先级队列的性质上，加速的方法是使用更加复杂的数据结构，比如基数堆，斐波那契堆。

### Prim's algorithm (lazy version)

每一步，我们都添加一条新的边到已经建好的树上面。

- 从始至终，我们只有一棵树。
- 新的边必须满足两个条件：（1）该边的一个顶点必须是已经建好的树里面的顶点。（2）该边在所有满足（1）条件的边中具有最小的权重。

空间复杂度：`E`

时间复杂度：`ElogE`

#### Implementation

```java
public class LazyPrimMST {
    private static final double FLOATING_POINT_EPSILON = 1E-12;

    private double weight;       // total weight of MST
    private Queue<Edge> mst;     // edges in the MST
    private boolean[] marked;    // marked[v] = true iff v on tree
    private MinPQ<Edge> pq;      // edges with one endpoint in tree

    /**
     * Compute a minimum spanning tree (or forest) of an edge-weighted graph.
     * @param G the edge-weighted graph
     */
    public LazyPrimMST(EdgeWeightedGraph G) {
        mst = new Queue<Edge>();
        pq = new MinPQ<Edge>();
        marked = new boolean[G.V()];
        for (int v = 0; v < G.V(); v++)     // run Prim from all vertices to
            if (!marked[v]) prim(G, v);     // get a minimum spanning forest

        // check optimality conditions
        assert check(G);
    }

    // run Prim's algorithm
    private void prim(EdgeWeightedGraph G, int s) {
        scan(G, s);
        while (!pq.isEmpty()) {                        // better to stop when mst has V-1 edges
            Edge e = pq.delMin();                      // smallest edge on pq
            int v = e.either(), w = e.other(v);        // two endpoints
            assert marked[v] || marked[w];
            if (marked[v] && marked[w]) continue;      // lazy, both v and w already scanned
            mst.enqueue(e);                            // add e to MST
            weight += e.weight();
            if (!marked[v]) scan(G, v);               // v becomes part of tree
            if (!marked[w]) scan(G, w);               // w becomes part of tree
        }
    }

    // add all edges e incident to v onto pq if the other endpoint has not yet been scanned
    private void scan(EdgeWeightedGraph G, int v) {
        assert !marked[v];
        marked[v] = true;
        for (Edge e : G.adj(v))
            if (!marked[e.other(v)]) pq.insert(e);
    }

    /**
     * Returns the edges in a minimum spanning tree (or forest).
     * @return the edges in a minimum spanning tree (or forest) as
     *    an iterable of edges
     */
    public Iterable<Edge> edges() {
        return mst;
    }

    /**
     * Returns the sum of the edge weights in a minimum spanning tree (or forest).
     * @return the sum of the edge weights in a minimum spanning tree (or forest)
     */
    public double weight() {
        return weight;
    }

}
```

### Prim's algorithm (eager version)

为了提升lazy version，我们可以试着删掉`priority queue`中一些不可用的边，使得其中只保留`tree vertices`和`non-tree vertices`间的`the crossing edges`。

除此之外，其实我们还能删除更多的边。因为对于那些还没被加进树里面的节点，我们只需要对每个`non-tree vertex`保存一条边（这条边具有最小的权重并且连接个一个`tree vertex`）。每次我们想要添加一个新节点，我们只需要挑出那个具有最小权重的边的`non-tree vertex`。在把这个`non-tree vertex`变成`tree vertex`时，我们需要检查一下，是否需要更新剩余的`non-tree vertices`对应的边，因为我们多了一个新的节点。

空间复杂度：`V`

时间复杂度：`ElogV`

#### Implementation

```java
public class PrimMST {
    private static final double FLOATING_POINT_EPSILON = 1E-12;

    private Edge[] edgeTo;        // edgeTo[v] = shortest edge from tree vertex to non-tree vertex
    private double[] distTo;      // distTo[v] = weight of shortest such edge
    private boolean[] marked;     // marked[v] = true if v on tree, false otherwise
    private IndexMinPQ<Double> pq;

    /**
     * Compute a minimum spanning tree (or forest) of an edge-weighted graph.
     * @param G the edge-weighted graph
     */
    public PrimMST(EdgeWeightedGraph G) {
        edgeTo = new Edge[G.V()];
        distTo = new double[G.V()];
        marked = new boolean[G.V()];
        pq = new IndexMinPQ<Double>(G.V());
        for (int v = 0; v < G.V(); v++)
            distTo[v] = Double.POSITIVE_INFINITY;

        for (int v = 0; v < G.V(); v++)      // run from each vertex to find
            if (!marked[v]) prim(G, v);      // minimum spanning forest

        // check optimality conditions
        assert check(G);
    }

    // run Prim's algorithm in graph G, starting from vertex s
    private void prim(EdgeWeightedGraph G, int s) {
        distTo[s] = 0.0;
        pq.insert(s, distTo[s]);
        while (!pq.isEmpty()) {
            int v = pq.delMin();
            scan(G, v);
        }
    }

    // scan vertex v
    private void scan(EdgeWeightedGraph G, int v) {
        marked[v] = true;
        for (Edge e : G.adj(v)) {
            int w = e.other(v);
            if (marked[w]) continue;         // v-w is obsolete edge
            if (e.weight() < distTo[w]) {
                distTo[w] = e.weight();
                edgeTo[w] = e;
                if (pq.contains(w)) pq.decreaseKey(w, distTo[w]);
                else                pq.insert(w, distTo[w]);
            }
        }
    }

    /**
     * Returns the edges in a minimum spanning tree (or forest).
     * @return the edges in a minimum spanning tree (or forest) as
     *    an iterable of edges
     */
    public Iterable<Edge> edges() {
        Queue<Edge> mst = new Queue<Edge>();
        for (int v = 0; v < edgeTo.length; v++) {
            Edge e = edgeTo[v];
            if (e != null) {
                mst.enqueue(e);
            }
        }
        return mst;
    }

    /**
     * Returns the sum of the edge weights in a minimum spanning tree (or forest).
     * @return the sum of the edge weights in a minimum spanning tree (or forest)
     */
    public double weight() {
        double weight = 0.0;
        for (Edge e : edges())
            weight += e.weight();
        return weight;
    }


    // check optimality conditions (takes time proportional to E V lg* V)
    private boolean check(EdgeWeightedGraph G) {

        // check weight
        double totalWeight = 0.0;
        for (Edge e : edges()) {
            totalWeight += e.weight();
        }
        if (Math.abs(totalWeight - weight()) > FLOATING_POINT_EPSILON) {
            System.err.printf("Weight of edges does not equal weight(): %f vs. %f\n", totalWeight, weight());
            return false;
        }

        // check that it is acyclic
        UF uf = new UF(G.V());
        for (Edge e : edges()) {
            int v = e.either(), w = e.other(v);
            if (uf.connected(v, w)) {
                System.err.println("Not a forest");
                return false;
            }
            uf.union(v, w);
        }

        // check that it is a spanning forest
        for (Edge e : G.edges()) {
            int v = e.either(), w = e.other(v);
            if (!uf.connected(v, w)) {
                System.err.println("Not a spanning forest");
                return false;
            }
        }

        // check that it is a minimal spanning forest (cut optimality conditions)
        for (Edge e : edges()) {

            // all edges in MST except e
            uf = new UF(G.V());
            for (Edge f : edges()) {
                int x = f.either(), y = f.other(x);
                if (f != e) uf.union(x, y);
            }

            // check that e is min weight edge in crossing cut
            for (Edge f : G.edges()) {
                int x = f.either(), y = f.other(x);
                if (!uf.connected(x, y)) {
                    if (f.weight() < e.weight()) {
                        System.err.println("Edge " + f + " violates cut optimality conditions");
                        return false;
                    }
                }
            }

        }

        return true;
    }

    /**
     * Unit tests the {@code PrimMST} data type.
     *
     * @param args the command-line arguments
     */
    public static void main(String[] args) {
        In in = new In(args[0]);
        EdgeWeightedGraph G = new EdgeWeightedGraph(in);
        PrimMST mst = new PrimMST(G);
        for (Edge e : mst.edges()) {
            StdOut.println(e);
        }
        StdOut.printf("%.5f\n", mst.weight());
    }

}
```

### Kruskal's algorithm

首先，我们把所有的边从小到大排列。然后，每次我们都从中挑出一条最小权重的边（该边不能形成闭环）。在添加`V-1`条边后（`V`是顶点总数），我们就得到了一颗MST。

空间复杂度：`E`

时间复杂度：`ElogE`

#### Implementation

```java
public class KruskalMST {
    private static final double FLOATING_POINT_EPSILON = 1E-12;

    private double weight;                        // weight of MST
    private Queue<Edge> mst = new Queue<Edge>();  // edges in MST

    /**
     * Compute a minimum spanning tree (or forest) of an edge-weighted graph.
     * @param G the edge-weighted graph
     */
    public KruskalMST(EdgeWeightedGraph G) {
        // more efficient to build heap by passing array of edges
        MinPQ<Edge> pq = new MinPQ<Edge>();
        for (Edge e : G.edges()) {
            pq.insert(e);
        }

        // run greedy algorithm
        UF uf = new UF(G.V());
        while (!pq.isEmpty() && mst.size() < G.V() - 1) {
            Edge e = pq.delMin();
            int v = e.either();
            int w = e.other(v);
            if (!uf.connected(v, w)) { // v-w does not create a cycle
                uf.union(v, w);  // merge v and w components
                mst.enqueue(e);  // add edge e to mst
                weight += e.weight();
            }
        }

        // check optimality conditions
        assert check(G);
    }

    /**
     * Returns the edges in a minimum spanning tree (or forest).
     * @return the edges in a minimum spanning tree (or forest) as
     *    an iterable of edges
     */
    public Iterable<Edge> edges() {
        return mst;
    }

    /**
     * Returns the sum of the edge weights in a minimum spanning tree (or forest).
     * @return the sum of the edge weights in a minimum spanning tree (or forest)
     */
    public double weight() {
        return weight;
    }

}
```
