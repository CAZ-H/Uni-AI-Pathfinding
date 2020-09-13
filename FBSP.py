__author__ = 'Jacky Baltes <jacky@cs.umanitoba.ca>' # For decomposition algorithms and rendering code
# File extended for assignment

import sys
import matplotlib.pyplot as plt
import numpy as np
import math
import copy
import time
from pathplanning import PathPlanningProblem, Rectangle

FLOAT_FUDGE = 0.005

###############################################################################################
# Decomposition
###############################################################################################

class Cell:
    def __init__(self, rect, occupancy, parent=None):
        self.rectangle = rect
        self.occupancy = occupancy
        self.children = []
        self.parent = parent

    def __repr__(self):
        return "(" + str(self.rectangle.x + self.rectangle.width/2) + ", " + str(self.rectangle.y + self.rectangle.height/2) + ")"


class CellFBSP(Cell):
    def __init__(self, rect, occupancy, parent=None, direction=None, split=None):
        super().__init__(rect, occupancy, parent)
        self.direction = direction
        self.split = split


class CellDecomposition:
    def __init__(self, domain, minimumSize):
        self.domain = domain
        self.minimumSize = minimumSize
        self.root = Cell(Rectangle(0.0, 0.0, domain.width, domain.height), 'unknown', None)
        self.maxDepth = 1

    def Draw(self, ax, node = None, depth = 1):
            if ( node == None ):
                node = self.root
            outline = plt.Rectangle((node.rectangle.x, node.rectangle.y), node.rectangle.width, node.rectangle.height, fill=False, facecolor=None, lw=0.5, zorder=math.inf)
            r = plt.Rectangle((node.rectangle.x, node.rectangle.y), node.rectangle.width, node.rectangle.height, fill=False, facecolor=None, alpha=0.5, lw=None)
            if ( node.occupancy == 'mixed' ):
                color = '#5080ff'
                if ( node.children == [] ):
                    r.set_fill(True)
                    r.set_facecolor(color)
            elif ( node.occupancy == 'free' ):
                color = '#ffff00'
                r.set_fill(True)
                r.set_facecolor(color)
                r.set_alpha(depth/self.maxDepth * r.get_alpha())
            elif ( node.occupancy == 'obstacle'):
                color = '#5050ff'
                r.set_fill(True)
                r.set_facecolor(color)
            else:
                print("Error: don't know how to draw cell of type", node.occupancy)
            ax.add_patch(r)
            ax.add_patch(outline)
            for c in node.children:
                self.Draw(ax, c, depth + 1)

    def CountCells(self, node = None ):
        if ( node is None ):
            node = self.root
        sum = 0
        if ( node.children != [] ):
            sum = 0
            for c in node.children:
                sum = sum + self.CountCells(c)
        else:
            sum = 1
        return sum


class QuadTreeDecomposition(CellDecomposition):
    def __init__(self, domain, minimumSize):
        super().__init__(domain, minimumSize)
        self.root = self.Decompose(self.root)

    def Decompose(self, node, depth = 1):
        cell = 'free'
        r = node.rectangle
        rx = r.x
        ry = r.y
        rwidth = r.width
        rheight = r.height

        if depth > self.maxDepth:
            self.maxDepth = depth

        for o in self.domain.obstacles:
            if ( o.CalculateOverlap(r) >= rwidth * rheight ):
                cell = 'obstacle'
                break
            elif ( o.CalculateOverlap(r) > 0.0 ):
                cell = 'mixed'
                break

        if ( cell == 'mixed'):
            # Decomposition order
            # 1 2
            # 3 4
            if (rwidth / 2.0 > self.minimumSize) and (rheight / 2.0 > self.minimumSize):
                childt1 = Cell(Rectangle(rx, ry, rwidth/2.0, rheight/2.0), 'unknown', node)
                qchild1 = self.Decompose(childt1, depth + 1)
                childt2 = Cell(Rectangle(rx + rwidth/2.0, ry, rwidth/2.0, rheight/2.0), 'unknown', node)
                qchild2 = self.Decompose(childt2, depth + 1)
                childt3 = Cell(Rectangle(rx, ry + rheight/2.0, rwidth/2.0, rheight/2.0), 'unknown', node)
                qchild3 = self.Decompose(childt3, depth + 1)
                childt4 = Cell(Rectangle(rx + rwidth/2.0, ry + rheight/2.0, rwidth/2.0, rheight/2.0), 'unknown', node)
                qchild4 = self.Decompose(childt4, depth + 1)
                children = [ qchild1, qchild2, qchild3, qchild4 ]
                node.children = children
            else:
                cell = 'obstacle'
                
        node.occupancy = cell
        return node


class BinarySpacePartitioning(CellDecomposition):
    def __init__(self, domain, minimumSize ):
        super().__init__(domain, minimumSize)
        self.root = self.Decompose(self.root)

    def Entropy(self, p):
        e = 0.0
        if ( ( p > 0 ) and ( p < 1.0 ) ):
            e = -p * math.log(p,2) - (1-p) * math.log(1-p,2)
        return e

    def CalcEntropy(self, rect):
        area = rect.width * rect.height
        a = 0.0
        for o in self.domain.obstacles:
            a = a + rect.CalculateOverlap(o)
        p = a / area
        return self.Entropy(p)

    def Decompose(self, node, depth = 1):
        cell = 'free'
        r = node.rectangle
        rx = r.x
        ry = r.y
        rwidth = r.width
        rheight = r.height
        area = rwidth * rheight

        if depth > self.maxDepth:
            self.maxDepth = depth

        for o in self.domain.obstacles:
            if ( o.CalculateOverlap(r) >= rwidth * rheight ):
                cell = 'obstacle'
                break
            elif ( o.CalculateOverlap(r) > 0.0 ):
                cell = 'mixed'
                break

        if ( cell == 'mixed'):
            entropy = self.CalcEntropy(r)
            hSplitTop = None
            hSplitBottom = None
            vSplitLeft = None
            vSplitRight = None

            igH = 0.0
            if ( r.height / 2.0 > self.minimumSize):
                hSplitTop = Rectangle(rx, ry + rheight/2.0, rwidth, rheight/2.0)
                entHSplitTop = self.CalcEntropy(hSplitTop)
                hSplitBottom = Rectangle(rx, ry, rwidth, rheight/2.0)
                entHSplitBottom = self.CalcEntropy( hSplitBottom )

                igH = entropy - ( r.width * r.height / 2.0 ) / area * entHSplitTop \
                      - ( r.width * r.height / 2.0 ) / area * entHSplitBottom

            igV = 0.0
            if ( r.width / 2.0 > self.minimumSize ):
                vSplitLeft = Rectangle(rx, ry, rwidth/2.0, rheight )
                entVSplitLeft = self.CalcEntropy( vSplitLeft )
                vSplitRight = Rectangle( rx + rwidth/2.0, ry, rwidth/2.0, rheight)
                entVSplitRight = self.CalcEntropy( vSplitRight)
                igV = entropy - ( r.width/2.0 * r.height ) / area * entVSplitLeft \
                      - ( r.width/2.0 * r.height ) / area * entVSplitRight

            children = []
            if ( igH > igV ):
                if ( igH > 0.0 ):
                    if ( hSplitTop is not None ) and ( hSplitBottom is not None ):
                        node.split = "horizontal"
                        # Horizontal split child order: Top, Bottom
                        childTop = CellFBSP(hSplitTop, 'unknown', node, "N")
                        childBottom = CellFBSP(hSplitBottom, 'unknown', node, "S")
                        children = [childTop, childBottom]
            else:
                if ( igV > 0.0 ):
                    if ( vSplitLeft is not None ) and ( vSplitRight is not None ):
                        node.split = "vertical"
                        # Vertical split child order: Left, Right
                        childLeft = CellFBSP(vSplitLeft, 'unknown', node, "W")
                        childRight = CellFBSP(vSplitRight, 'unknown', node, "E")
                        children = [childLeft, childRight]

            for c in children:
                self.Decompose(c, depth + 1)
            node.children = children
        node.occupancy = cell
        return node

###############################################################################################
# Quadtree processing
###############################################################################################

# Algorithm referenced from: https://geidav.wordpress.com/2017/12/02/advanced-octrees-4-finding-neighbor-nodes/
def getAdjacentOfGreaterOrEqualSizeQuadTree(cell, direction):   
    if direction == "N":
        if cell.parent is None: # Reached root?
            return None
        if cell.parent.children[2] == cell: # Is 'cell' SW child?
            return cell.parent.children[0] # NW
        if cell.parent.children[3] == cell: # Is 'cell' SE child?
            return cell.parent.children[1] # NE
     
        node = getAdjacentOfGreaterOrEqualSizeQuadTree(cell.parent, direction)
        if node is None or node.children == []:
            return node
     
        # 'cell' is guaranteed to be a north child
        return (node.children[2] # SW
                if cell.parent.children[0] == cell # Is 'cell' NW child?
                else node.children[3]) # SE

    elif direction == "S":
        if cell.parent is None: # Reached root?
            return None
        if cell.parent.children[0] == cell: # Is 'cell' NW child?
            return cell.parent.children[2] # SW
        if cell.parent.children[1] == cell: # Is 'cell' NE child?
            return cell.parent.children[3] # SE
     
        node = getAdjacentOfGreaterOrEqualSizeQuadTree(cell.parent, direction)
        if node is None or node.children == []:
            return node
     
        # 'cell' is guaranteed to be a south child
        return (node.children[0] # NW
                if cell.parent.children[2] == cell # Is 'cell' SW child?
                else node.children[1]) # NE

    elif direction == "E":       
        if cell.parent is None: # Reached root?
            return None
        if cell.parent.children[0] == cell: # Is 'cell' NW child?
            return cell.parent.children[1] # NE
        if cell.parent.children[2] == cell: # Is 'cell' SW child?
            return cell.parent.children[3] # SE
     
        node = getAdjacentOfGreaterOrEqualSizeQuadTree(cell.parent, direction)
        if node is None or node.children == []:
            return node
     
        # 'cell' is guaranteed to be a east child
        return (node.children[0] # NW
                if cell.parent.children[1] == cell # Is 'cell' NE child?
                else node.children[2]) # SW

    elif direction == "W":       
        if cell.parent is None: # Reached root?
            return None
        if cell.parent.children[1] == cell: # Is 'cell' NE child?
            return cell.parent.children[0] # NW
        if cell.parent.children[3] == cell: # Is 'cell' SE child?
            return cell.parent.children[2] # SW
     
        node = getAdjacentOfGreaterOrEqualSizeQuadTree(cell.parent, direction)
        if node is None or node.children == []:
            return node
     
        # 'cell' is guaranteed to be a west child
        return (node.children[1] # NE
                if cell.parent.children[0] == cell # Is 'cell' NW child?
                else node.children[3]) # SE


def getAdjacentOfSmallerSizeQuadTree(cell, adjacent, direction):   
    candidates = [] if adjacent is None else [adjacent]
    adjacents = []
 
    if direction == "N":
        while len(candidates) > 0:
            if candidates[0].children == []:
                adjacents.append(candidates[0])
            else:
                candidates.append(candidates[0].children[2]) # SW
                candidates.append(candidates[0].children[3]) # SE
            candidates.remove(candidates[0])

        return adjacents

    elif direction == "S":
        while len(candidates) > 0:
            if candidates[0].children == []:
                adjacents.append(candidates[0])
            else:
                candidates.append(candidates[0].children[0]) # NW
                candidates.append(candidates[0].children[1]) # NE
            candidates.remove(candidates[0])

        return adjacents

    elif direction == "E":
        while len(candidates) > 0:
            if candidates[0].children == []:
                adjacents.append(candidates[0])
            else:
                candidates.append(candidates[0].children[0]) # NW
                candidates.append(candidates[0].children[2]) # SW
            candidates.remove(candidates[0])

        return adjacents

    elif direction == "W":
        while len(candidates) > 0:
            if candidates[0].children == []:
                adjacents.append(candidates[0])
            else:
                candidates.append(candidates[0].children[1]) # NE
                candidates.append(candidates[0].children[3]) # SE
            candidates.remove(candidates[0])

        return adjacents


def getAdjacentsQuadTree(cell, direction):   
    adjacent = getAdjacentOfGreaterOrEqualSizeQuadTree(cell, direction)
    adjacents = getAdjacentOfSmallerSizeQuadTree(cell, adjacent, direction)
    return adjacents


def getAllAdjacentsQuadTree(cell):
    adjacentsN = getAdjacentsQuadTree(cell, "N")
    adjacentsS = getAdjacentsQuadTree(cell, "S")
    adjacentsE = getAdjacentsQuadTree(cell, "E")
    adjacentsW = getAdjacentsQuadTree(cell, "W")
    return adjacentsN + adjacentsS + adjacentsE + adjacentsW

###############################################################################################
# FBSP processing
###############################################################################################

def checkIfChildCanTouchCell(child, cell, axis):
    if axis == "x":
        boundNear = cell.rectangle.x
        boundFar = cell.rectangle.x + cell.rectangle.width
        xNear = child.rectangle.x + FLOAT_FUDGE # Pinch the range a little to accomodate for float accuracy. See: https://i.imgur.com/ZJYLdZH.png / FBSP Tree adjacent detection hack.png
        xFar = child.rectangle.x + child.rectangle.width - FLOAT_FUDGE
        xMid = child.rectangle.x + child.rectangle.width/2
        return (xMid > boundNear and xMid < boundFar) or (xNear > boundNear and xNear < boundFar) or (xFar > boundNear and xFar < boundFar) or (xNear < boundNear and xFar > boundFar)
    else: # y
        boundNear = cell.rectangle.y
        boundFar = cell.rectangle.y + cell.rectangle.height
        yNear = child.rectangle.y + FLOAT_FUDGE
        yFar = child.rectangle.y + child.rectangle.height - FLOAT_FUDGE
        yMid = child.rectangle.y + child.rectangle.height/2
        return (yMid > boundNear and yMid < boundFar) or (yNear > boundNear and yNear < boundFar) or (yFar > boundNear and yFar < boundFar) or (yNear < boundNear and yFar > boundFar)


def getAdjacentOfGreaterOrEqualSizeFBSP(cell, direction):
    if direction == "N":
        if cell.parent is None: # Reached root?
            return []
        if cell.direction == "S": #Is 'cell' S child?
            return [cell.parent.children[0]] # N

        nodes = getAdjacentOfGreaterOrEqualSizeFBSP(cell.parent, direction) 
        if nodes is None:
            return []

        adjacents = []
        for node in nodes:
            if node.children == []:
                if checkIfChildCanTouchCell(node, cell, "x"): # Check if child x coordinates touch 'cell' before including child
                    adjacents.append(node)

            # 'cell' is guaranteed to be a north child
            if node.split == "horizontal":
                if checkIfChildCanTouchCell(node.children[1], cell, "x"):
                    adjacents.append(node.children[1]) # S
            elif node.split == "vertical":
                if checkIfChildCanTouchCell(node.children[0], cell, "x"):
                    adjacents.append(node.children[0]) # W
                if checkIfChildCanTouchCell(node.children[1], cell, "x"):
                    adjacents.append(node.children[1]) # E

        return adjacents

    elif direction == "S":
        if cell.parent is None: # Reached root?
            return []
        if cell.direction == "N": #Is 'cell' N child?
            return [cell.parent.children[1]] # S

        nodes = getAdjacentOfGreaterOrEqualSizeFBSP(cell.parent, direction) 
        if nodes is None:
            return []

        adjacents = []
        for node in nodes:
            if node.children == []:
                if checkIfChildCanTouchCell(node, cell, "x"): # Check if child x coordinates touch 'cell' before including child
                    adjacents.append(node)

            # 'cell' is guaranteed to be a south child
            if node.split == "horizontal":
                if checkIfChildCanTouchCell(node.children[0], cell, "x"):
                    adjacents.append(node.children[0]) # N
            elif node.split == "vertical":
                if checkIfChildCanTouchCell(node.children[0], cell, "x"):
                    adjacents.append(node.children[0]) # W
                if checkIfChildCanTouchCell(node.children[1], cell, "x"):
                    adjacents.append(node.children[1]) # E

        return adjacents

    elif direction == "E":
        if cell.parent is None: # Reached root?
            return []
        if cell.direction == "W": #Is 'cell' W child?
            return [cell.parent.children[1]] # E

        nodes = getAdjacentOfGreaterOrEqualSizeFBSP(cell.parent, direction) 
        if nodes is None:
            return []

        adjacents = []
        for node in nodes:
            if node.children == []:
                if checkIfChildCanTouchCell(node, cell, "y"): # Check if child y coordinates touch 'cell' before including child
                    adjacents.append(node)

            # 'cell' is guaranteed to be an east child
            if node.split == "horizontal":
                if checkIfChildCanTouchCell(node.children[0], cell, "y"):
                    adjacents.append(node.children[0]) # N
                if checkIfChildCanTouchCell(node.children[1], cell, "y"):
                    adjacents.append(node.children[1]) # S
            elif node.split == "vertical":
                if checkIfChildCanTouchCell(node.children[0], cell, "y"):
                    adjacents.append(node.children[0]) # W

        return adjacents

    elif direction == "W":
        if cell.parent is None: # Reached root?
            return []
        if cell.direction == "E": #Is 'cell' E child?
            return [cell.parent.children[0]] # W

        nodes = getAdjacentOfGreaterOrEqualSizeFBSP(cell.parent, direction) 
        if nodes is None:
            return []

        adjacents = []
        for node in nodes:
            if node.children == []:
                if checkIfChildCanTouchCell(node, cell, "y"): # Check if child y coordinates touch 'cell' before including child
                    adjacents.append(node)

            # 'cell' is guaranteed to be an west child
            if node.split == "horizontal":
                if checkIfChildCanTouchCell(node.children[0], cell, "y"):
                    adjacents.append(node.children[0]) # N
                if checkIfChildCanTouchCell(node.children[1], cell, "y"):
                    adjacents.append(node.children[1]) # S
            elif node.split == "vertical":
                if checkIfChildCanTouchCell(node.children[1], cell, "y"):
                    adjacents.append(node.children[1]) # E

        return adjacents


def getAdjacentOfSmallerSizeFBSP(cell, adjacents, direction):
    candidates = adjacents
    allAdjacents = []

    if direction == "N":
        while len(candidates) > 0:
            if candidates[0].children == []:
                if checkIfChildCanTouchCell(candidates[0], cell, "x"): # Check if child x coordinates touch 'cell' before including child
                    allAdjacents.append(candidates[0])
            else:
                if candidates[0].split == "horizontal":
                    if checkIfChildCanTouchCell(candidates[0].children[1], cell, "x"):
                        allAdjacents.append(candidates[0].children[1]) # S
                elif candidates[0].split == "vertical":
                    if checkIfChildCanTouchCell(candidates[0].children[0], cell, "x"):
                        candidates.append(candidates[0].children[0]) # W
                    if checkIfChildCanTouchCell(candidates[0].children[1], cell, "x"):
                        candidates.append(candidates[0].children[1]) # E
            candidates.remove(candidates[0])

        return allAdjacents

    elif direction == "S":
        while len(candidates) > 0:
            if candidates[0].children == []:
                if checkIfChildCanTouchCell(candidates[0], cell, "x"): # Check if child x coordinates touch 'cell' before including child
                    allAdjacents.append(candidates[0])
            else:
                if candidates[0].split == "horizontal":
                    if checkIfChildCanTouchCell(candidates[0].children[0], cell, "x"):
                        allAdjacents.append(candidates[0].children[0]) # N
                elif candidates[0].split == "vertical":
                    if checkIfChildCanTouchCell(candidates[0].children[0], cell, "x"):
                        candidates.append(candidates[0].children[0]) # W
                    if checkIfChildCanTouchCell(candidates[0].children[1], cell, "x"):
                        candidates.append(candidates[0].children[1]) # E
            candidates.remove(candidates[0])

        return allAdjacents

    elif direction == "E":
        while len(candidates) > 0:
            if candidates[0].children == []:
                if checkIfChildCanTouchCell(candidates[0], cell, "y"): # Check if child y coordinates touch 'cell' before including child
                    allAdjacents.append(candidates[0])
            else:
                if candidates[0].split == "horizontal":
                    if checkIfChildCanTouchCell(candidates[0].children[0], cell, "y"):
                        candidates.append(candidates[0].children[0]) # N
                    if checkIfChildCanTouchCell(candidates[0].children[1], cell, "y"):
                        candidates.append(candidates[0].children[1]) # S
                elif candidates[0].split == "vertical":
                    if checkIfChildCanTouchCell(candidates[0].children[0], cell, "y"):
                        allAdjacents.append(candidates[0].children[0]) # W
            candidates.remove(candidates[0])

        return allAdjacents

    elif direction == "W":
        while len(candidates) > 0:
            if candidates[0].children == []:
                if checkIfChildCanTouchCell(candidates[0], cell, "y"): # Check if child y coordinates touch 'cell' before including child
                    allAdjacents.append(candidates[0])
            else:
                if candidates[0].split == "horizontal":
                    if checkIfChildCanTouchCell(candidates[0].children[0], cell, "y"):
                        candidates.append(candidates[0].children[0]) # N
                    if checkIfChildCanTouchCell(candidates[0].children[1], cell, "y"):
                        candidates.append(candidates[0].children[1]) # S
                elif candidates[0].split == "vertical":
                    if checkIfChildCanTouchCell(candidates[0].children[1], cell, "y"):
                        allAdjacents.append(candidates[0].children[1]) # E
            candidates.remove(candidates[0])

        return allAdjacents


def getAdjacentsFBSP(cell, direction):   
    adjacents = getAdjacentOfGreaterOrEqualSizeFBSP(cell, direction)
    allAdjacents = getAdjacentOfSmallerSizeFBSP(cell, adjacents, direction)
    return allAdjacents


def getAllAdjacentsFBSP(cell):
    adjacentsN = getAdjacentsFBSP(cell, "N")
    adjacentsS = getAdjacentsFBSP(cell, "S")
    adjacentsE = getAdjacentsFBSP(cell, "E")
    adjacentsW = getAdjacentsFBSP(cell, "W")
    return adjacentsN + adjacentsS + adjacentsE + adjacentsW

###############################################################################################
# Pathfinding - A*
###############################################################################################

def forEachLeafInTree(treeRoot, function):
    if treeRoot.children == []:
        function(treeRoot)
    else:
        for child in treeRoot.children:
            forEachLeafInTree(child, function)


def pathfinding(initial, goal, cellDecomp):
    useQuad = True
    if type(cellDecomp) == BinarySpacePartitioning:
        useQuad = False
        print("\nPathfinding using FPSP decomposition")
    else:
        print("\nPathfinding using quadtree decomposition")

    vertices = []
    distance = {}
    previous = {}

    infinity = math.inf
    startTime = time.time()

    ##############
    # Find initial and goal cell
    def checkPointCollisionWithCell(pos, cell):
        return pos[0] < cell.rectangle.x + cell.rectangle.width and pos[0] > cell.rectangle.x and pos[1] < cell.rectangle.y + cell.rectangle.height and pos[1] > cell.rectangle.y

    # Get the smallest cell containing the starting position (initialCell)
    global initialCell
    initialCell = None
    def findCellContainingInitialPos(cell):
        if checkPointCollisionWithCell(initial, cell):
            global initialCell
            initialCell = cell
    forEachLeafInTree(cellDecomp.root, findCellContainingInitialPos)

    if initialCell == None:
        print("Failed to find initial cell in map")
        return

    # Get the smallest cell containing the goal position (goalCell)
    global goalCell
    goalCell = None
    def findCellContainingGoalPos(cell):
        if checkPointCollisionWithCell(goal, cell):
            global goalCell
            goalCell = cell
    forEachLeafInTree(cellDecomp.root, findCellContainingGoalPos)

    if goalCell == None:
        print("Failed to find goal cell in map")
        return

    ##############
    # A* cost functions:
    def getDistanceBetweenCells(cellA, cellB):
        distX = (cellA.rectangle.x + cellA.rectangle.width/2) - (cellB.rectangle.x + cellB.rectangle.width/2)
        distY = (cellA.rectangle.y + cellA.rectangle.height/2) - (cellB.rectangle.y + cellB.rectangle.height/2)
        return math.sqrt(distX*distX + distY*distY)

    def getDistanceBetweenCellAndPoint(cellA, point):
        distX = (cellA.rectangle.x + cellA.rectangle.width/2) - point[0]
        distY = (cellA.rectangle.y + cellA.rectangle.height/2) - point[1]
        return math.sqrt(distX*distX + distY*distY)

    def calculateCellCost(cell):
        hCost = getDistanceBetweenCellAndPoint(cell, goal)
        gCost = infinity
        if cell in distance:
            gCost = distance[cell]
        return gCost + hCost

    ##############
    # Perform Djikstra's algorithm:
    # initial starting cell is initialCell

    # for each cell (vertex) in map
        # distance[vertex] = infinity
        # previous[vertex] = None
        # vertices.append(vertex)
    def initializeDjikstra(cell):
        if cell.occupancy != "free":
            return

        distance[cell] = infinity
        previous[cell] = None
        vertices.append(cell)
    forEachLeafInTree(cellDecomp.root, initializeDjikstra)
    distance[initialCell] = 0

    # while vertices is not empty
        # currVertex = vertex in vertices with smallest(min) distance[currVertex]
        # remove currVertex from vertices

        # for each neighbor adjVertex of currVertex
            # distToNextVertex = distance[adjVertex] + length(currVertex, adjVertex)
            # if distToNextVertex < dist[adjVertex]
                # distance[adjVertex] = distToNextVertex
                # previous[adjVertex] = currVertex
    def getCellWithMinimumCostInVertices():
        minVert = [vertices[0], calculateCellCost(vertices[0])]
        for vertex in vertices:
            vertexCellCost = calculateCellCost(vertex)
            if minVert[1] > vertexCellCost:
                minVert = [vertex, vertexCellCost]
        return minVert[0]

    foundGoal = False
    while not foundGoal and len(vertices) > 0:
        currVertex = getCellWithMinimumCostInVertices() # A* heuristic
        vertices.remove(currVertex)

        adjacentVertices = []
        if useQuad:
            adjacentVertices = getAllAdjacentsQuadTree(currVertex)
        else:
            adjacentVertices = getAllAdjacentsFBSP(currVertex)

        for adjVertex in adjacentVertices:

            # Skip cells in tree that are not empty space
            if adjVertex.occupancy != "free":
                continue

            distToNextVertex = distance[currVertex] + getDistanceBetweenCells(currVertex, adjVertex)
            if distToNextVertex < distance[adjVertex]:
                distance[adjVertex] = distToNextVertex
                previous[adjVertex] = currVertex
                if adjVertex == goalCell:
                    foundGoal = True # Exit early once we found goal, due to A* this is guaranteed shortest path

    # Pathfinding complete: follow previous[] from goal, distance[] contains path length
    timeTaken = time.time() - startTime

    ##############
    # Trace back path into array of positions
    path = []
    pathDist = math.inf
    success = False

    print("Time taken:", timeTaken, flush=True)
    if goalCell in distance:
        pathDist = distance[goalCell]
        print("Distance:", pathDist, flush=True)

        if distance[goalCell] != math.inf:
            prev = goalCell
            while prev and prev != initialCell:
                path.append([prev.rectangle.x + prev.rectangle.width/2, prev.rectangle.y + prev.rectangle.height/2])
                if prev in previous:
                    prev = previous[prev]
                else:
                    print("Cell is not accessible:", prev.rectangle.x + prev.rectangle.width/2, prev.rectangle.y + prev.rectangle.height/2)
                    break
            path.append([initialCell.rectangle.x + initialCell.rectangle.width/2, initialCell.rectangle.y + initialCell.rectangle.height/2])

            print("Path:\n", path, flush=True)
            success = True
        else:
            print("Goal cell blocked, no path possible", flush=True)
    else:
        print("Starting cell was inaccessible, no path possible", flush=True)

    return path, pathDist, timeTaken, success


###############################################################################################
# Main
###############################################################################################

def main( argv = None, showPlot = True, problemSpace = None, initial = None, goals = None):
    if ( argv == None ):
        argv = sys.argv[1:]

    width = 10.0
    height = 10.0
    obstacleCount = 15
    obstacleWidth = 5.0
    obstacleHeight = 5.0

    ##############
    # Create path planning problem
    if problemSpace == None:
        problemSpace = PathPlanningProblem( width, height, obstacleCount, obstacleWidth, obstacleHeight)
    if initial == None or goals == None:
        initial, goals = problemSpace.CreateProblemInstance()
    goal = goals[0]

    ##############
    # Quad tree decomposition
    fig = plt.figure()
    ax = fig.add_subplot(1,2,1, aspect='equal')
    ax.set_xlim(0.0, width)
    ax.set_ylim(0.0, height)

    # Draw obstacles
    for o in problemSpace.obstacles:
        ax.add_patch(copy.copy(o.patch) )

    # Do the space partitioning
    qtd = QuadTreeDecomposition(problemSpace, 0.2)
    qtd.Draw(ax)

    # Draw initial position
    ax.add_patch(plt.Circle((initial[0], initial[1]), 0.05, color="#ff0000"))

    # Draw goal
    ax.add_patch(plt.Circle((goal[0], goal[1]), 0.05, color="#00ff00"))

    # Do pathfinding
    pathDist = math.inf
    timeTaken = 0
    path, pathDist, timeTaken, success = pathfinding(initial, goal, qtd)

    if success:
        prevPoint = goal
        for point in path:
            plt.plot([point[0], prevPoint[0]], [point[1], prevPoint[1]], '#00aaff', lw=2)
            ax.add_patch(plt.Circle((point[0], point[1]), 0.05, color="#00aaff"))
            prevPoint = point
        plt.plot([initial[0], prevPoint[0]], [initial[1], prevPoint[1]], '#00aaff', lw=2)

    # Title plot
    n = qtd.CountCells()
    ax.set_title('Quadtree Decomposition\n{0} cells\nTime taken: {1:.5f}\nDistance: {2:.5f}'.format(n, timeTaken, pathDist))

    ##############
    # Binary space partiioning
    ax = fig.add_subplot(1,2,2, aspect='equal')
    ax.set_xlim(0.0, width)
    ax.set_ylim(0.0, height)

    # Draw obstacles
    for o in problemSpace.obstacles:
        ax.add_patch(copy.copy(o.patch))

    # Do the space partitioning
    bsp = BinarySpacePartitioning(problemSpace, 0.2)
    bsp.Draw(ax)

    # Draw initial position
    ax.add_patch(plt.Circle((initial[0], initial[1]), 0.05, color="#ff0000"))

    # Draw goal
    ax.add_patch(plt.Circle((goal[0], goal[1]), 0.05, color="#00ff00"))

    # Do pathfinding
    path, pathDist, timeTaken, success = pathfinding(initial, goal, bsp)

    if success:
        prevPoint = goal
        for point in path:
            plt.plot([point[0], prevPoint[0]], [point[1], prevPoint[1]], '#00aaff', lw=2)
            ax.add_patch(plt.Circle((point[0], point[1]), 0.05, color="#00aaff"))
            prevPoint = point
        plt.plot([initial[0], prevPoint[0]], [initial[1], prevPoint[1]], '#00aaff', lw=2)

    # Title plot
    n = bsp.CountCells()
    ax.set_title('BSP Decomposition\n{0} cells\nTime taken: {1:.4f}\nDistance: {2:.4f}'.format(n, timeTaken, pathDist))

    ##############
    # Show plot
    if showPlot:
        plt.show()

###############################################################################################

if ( __name__ == '__main__' ):
    main()


