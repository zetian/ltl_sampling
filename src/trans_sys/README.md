## Module: H2C

1. Graph Structure

The structure is illustrated as follows:

Graph:

+ Vertex 0:
	- Edge 0 -> Neighbour Vertex
	- Edge 1 -> Neighbour Vertex

+ Vertex 1:
	- Edge 0 -> Neighbour Vertex
	- Edge 1 -> Neighbour Vertex

...

* Graph maintains a list of vertices, each vertex can be accessed from its ID
* Each vertex maintains a list of its edges
* Each edge stores the ID of the two vertices it connects

2. ltl2ba

Memory usage before adding new code:

```
==15434== HEAP SUMMARY:
==15434==     in use at exit: 21,392 bytes in 37 blocks
==15434==   total heap usage: 38 allocs, 1 frees, 21,393 bytes allocated
==15434== 
==15434== LEAK SUMMARY:
==15434==    definitely lost: 312 bytes in 5 blocks
==15434==    indirectly lost: 16 bytes in 1 blocks
==15434==      possibly lost: 20,704 bytes in 29 blocks
==15434==    still reachable: 360 bytes in 2 blocks
==15434==         suppressed: 0 bytes in 0 blocks
```