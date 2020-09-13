# Introduction

This repo was the working repo for a university Artificial Intelligence course assignment.  
Implemented here are two pathfinding algorithms, A* (through a quadtree & through an axis-aligned FBSP) and Rapidly-exploring Random Trees (RRT).

Below is the handed-in writeup accompanying this repo, enhanced with some embedded images.

---

# Running

Run compare.py to run both FBSP.py and RRT.py on the same problem domain. This script overrides map parameters hardcoded into FBSP and RRT, and will pass the same problem domain to both algorithms. FBSP.py and RRT.py will have their own map parameters when run individually.  

# Implementation

## Problem space

pathplanning.py was not changed from the provided code. This code generates obstacles, goals, and starting positions.

## Quadtree & FBSP

### Overview

The cell structure given from the sample code is not significantly changed, simply just abstracted into a Cell class with the addition of a parent property to support the adjacent cell finding agorithm.

Both the quadtree and BSP tree decompositions are run through the same A* pathfinding implementation.
The component changed between the quadtree and BSP tree in the A* algorithm is exclusively the algorithm to find adjacent cells in the given tree. No intermediary graph structure is created, the algorithm runs directly on the passed in tree.

The adjacent cell finding algorithm is referenced from: https://geidav.wordpress.com/2017/12/02/advanced-octrees-4-finding-neighbor-nodes/.
This same algorithm is adapted to work for the FBSP tree, albeit it is less elegant and uses some hacks and tricks to narrow down valid adjacent cells.  

Here is some justification for the float hack in checkIfChildCanTouchCell(). This function is necessary in the adjacent-cell-finding code because of some special cases with cell positions in the FBSP tree.

![](/FBSP-adjacent-deteciton-hack.png)

Diagonal cells are not considered adjacent since this seemed very complex in some cases, and even more complex for the FBSP. Supporting this for the quadtree and not the FBSP would have skewed the data, so in both cases only directly adjacent cells in the N S E W directions are considered.

The code should explain itself as it is well commented.
The entrypoint into the pathfinding code is the pathfinding() function in FBSP.py.

### Considerations

Often start or goal positions will be generated within mixed cells, which results in pathfinding failing since these cells are treated as walls.
Sometimes the path in the FBSP implementation will appear to clip through obstacles. This is because the path is drawn between cell centers. The only meaningful impact of this is that the path distance reported can sometimes be a little bit too short. These cells are always connected by other adjacent pathed-through cells.

Occasionally the FBSP adjacent cells algorithm (presumably) returns incorrect results, which can result in a solvable problem being reported as unsolvable. There isn't enough time to troubleshoot this, and it's fairly rare.

## RRT

### Overview

The algorithm has been mostly changed from the provided code. It should be commented well enough to explain how it works. Refer to the ExploreDomain() function within RRT.py.
A list of nodes in the tree is maintained and a reference to the root is kept. A random point is chosen on the map and the closest point in the tree is extended towards that random point. According to the bias value, occasionally instead the tree will be grown directly towards the goal. There is no open-space bias implemented because the cost of finding open space to generate random points over seemed too expensive.

### Considerations

Without doing RRT*, the final path could be optimized by going from the node that reached the goal, looking node by node and "raytracing" towards each next node until collision with an obstacle to eliminate redundant nodes. This would result in optimizing out the inefficient "squiggly" pathing this algorithm produces and tightening the path through empty space. This was not done in the interest of meeting the assignment deadline.

![](/RRT-path-optimization.png)

# Output

These images do not display the same maps from the data table below. They are just examples of output.

![](/output_1.png)

![](/output_2.png)

![](/output_3.png)

![](/output_4.png)

![](/output_5.png)

# Results

Graphs extracted from the Excel sheet.
![](/charts.png)

Please see the file data.xlsx for the Excel sheet.
That data is given below in raw text:
```
-------------------------------------------------------------------------------------------------------------------
                            Quadtree                   FBSP                      RRT     
obstacles | obstacle size | time    | length | cells | time  | length  | cells | time   | length | nodes | coverage
10          5               0.0020    8.4489   457     0.001   7.7242    230     0.006    9.6269   100     0.03
10          5               0.0020    5.5722   391     0.001   7.9367    167     0.002    6.3733   62      0.0188
10          5               0.0060    10.3942  520     0.002   11.0589   256     0.0519   10.7971  217     0.0628
10          5               0.0070    inf      577     0.004   inf       273     0.0818   7.8023   313     0.0892
10          5               0.0040    7.6288   487     0.002   7.4276    217     0.0728   8.5724   223     0.0704

                  Average   0.0042    8.011025 486.4   0.002   8.53685   228.6   0.0429   8.6344   183     0.05424
-------------------------------------------------------------------------------------------------------------------
20          4               0.01793   inf      649     0.01    inf       400     1.58096  14.4358  1692    0.4128
20          4               0.003     4.7118   718     0.002   5.5195    394     0.00099  4.55794  44      0.014
20          4               0.01795   inf      601     0.0099  inf       389     0.08377  4.78714  343     0.0984
20          4               0.00316   5.10142  610     0.001   4.0934    373     0.00199  4.42253  60      0.0188
20          4               0.00707   9.94106  685     0.003   10.2577   403     0.01695  10.58894 128     0.038

                  Average   0.009822  6.58476  652.6   0.00518 6.62353   391.8   0.336932 7.75847  453.4   0.1164
-------------------------------------------------------------------------------------------------------------------
30          3               0.02293   inf      700     0.0105  inf       479     0.02691  12.84235 207     0.064
30          3               0.00601   9.57785  700     0.002   9.6888    464     0.36458  12.22792 651     0.1796
30          3               0.00898   8.90214  700     0.005   9.5494    491     0.00402  8.92447  80      0.0252
30          3               0.01997   inf      745     0.009   inf       468     0.04491  10.44788 170     0.0508
30          3               0.01895   inf      802     0.008   inf       472     0.00898  4.49032  65      0.0204

                  Average   0.015368  9.239995 729.4   0.0069  9.6191    474.8   0.08988  9.786588 234.6   0.068
-------------------------------------------------------------------------------------------------------------------
40          2               0.02693   inf      715     0.0129  inf       495     0.06184  4.78875  301     0.092
40          2               0.01807   19.70357 784     0.009   19.0094   571     0.01895  13.10355 481     0.1408
40          2               0.00202   2.68161  727     0.001   2.1356    533     0.72564  10.3032  1131    0.3012
40          2               0.001     1.25     730     0.001   0.9882    544     0        0.79674  10      0.0024
40          2               0.00499   5.41392  760     0.003   5.6351    547     0.00997  5.36549  45      0.014

                  Average   0.010602  7.262275 743.2   0.00538 6.942075  538     0.16328  6.871546 393.6   0.11008
-------------------------------------------------------------------------------------------------------------------
```
From the graphs of the above data, it can be seen that as the number of obstacles increases:

- The time it takes to run all the algorithms remains relatively constant, or indeterminate in the case of RRT. Runtime is not strongly affected by the number of obstacles.
  - However, on average FBSP takes less time to run than Quadtree, and RRT takes significantly more time to run than both Quadtree and FBSP.
    - This is likely because FBSP tends to generate half as many cells as Quadtree. The algorithm is very similar between both approaches, so this is the only explanation.

- The number of cells generated by the Quadtree/FBSP algorithms increases, but RRT remains indeterminate.
  - This makes sense because the problem space is more complex with more obstacles; the trees must become more granular to describe the map geometry
  - RRT is based off randomness, so the number of nodes required to find the goal is likely to vary widely.

- Path length does not seem strongly affected.
  - This is expected since the start and goal positions are random.

- Failure rate for both Quadtree and FBSP seems to increase.
  - Failure is denoted by an "inf" in the length column.
  - This can likely be explained by the probability of a start or goal position landing in a "mixed" space increasing with more obstacles.
  - RRT almost always finds a solution when the quadtree / FBSP approach fails.
