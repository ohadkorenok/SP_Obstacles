# Readme for Refael Work
This script is calculating the minimum distance from a source point to a destination point, in an obstacle surrounded area. 
We do it by a reduction to `Dijsktra` algorithm, for finding the shortest path in a weightened graph. 

#####for our use, all of the obstacles are rectangles.

## How to use the script : 
1. add all of the obstacles in the first line of the input text , in the next syntax: [((x1,y1),(x2,y2)) , ((v1,u1), (v2,u2))]
s.t x1,y1 represents the left bottom coordinate of obstacle 1, and x2,y2 represents the top right coordinate of obstacle 2. In this example we can notice that obstacle 2 coordinates is (v1,u1) and (v2,u2).
notice that this is a list of tuples that represents obstacles. (each tuple contains two tuples).

2. add the source point and the destination point in second line, by the following syntax:
[(sx1,sy1), (dx1,dy1)]. 

example: 

[((3, 4), (8, 6)), ((2, 5), (8, 15)), ((1, 3), (7, 4))] (line 1)

[(1, 1), (19, 8)] (line 2)

Thank you very much 

Ohad Koren