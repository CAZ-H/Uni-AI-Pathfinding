__author__ = 'Jacky Baltes <jacky@cs.umanitoba.ca>' # For rendering code
# File extended for assignment

import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import numpy as np
import math
import random
import time
from pathplanning import PathPlanningProblem, Rectangle

###############################################################################################
# RRT Exploration
###############################################################################################

class Node:
    def __init__(self, x, y, parent=None):
        self.children = []
        self.pos = [x, y]
        self.parent = parent

    def __repr__(self):
        return "(" + x + ", " + y + ") < (" + self.parent.x + ", " + self.parent.y + ")"


def getDistanceBetweenPoints(pointA, pointB):
        distX = pointA[0] - pointB[0]
        distY = pointA[1] - pointB[1]
        return math.sqrt(distX*distX + distY*distY)


def ExploreDomain(domain, initial, goal, maxSteps, stepDistance=0.1, biasToGoal=0, goalRadius=0.25):
    treeNodes = []
    treeRoot = Node(initial[0], initial[1])
    treeNodes.append(treeRoot)

    nodeNearGoal = None

    for i in range(maxSteps):
        # Get a target point in space to expand towards
        targetPoint = goal
        if random.uniform(0,1) < 1-biasToGoal: # If we're going in a random direction this step
            targetPoint = [random.uniform(0, domain.width), random.uniform(0, domain.height)]

        # Expand node nearest to point towards point
        nearestToRandomPoint = treeRoot
        distNearestToRandomPoint = getDistanceBetweenPoints(nearestToRandomPoint.pos, targetPoint)
        for node in treeNodes:
            dist = getDistanceBetweenPoints(node.pos, targetPoint)
            if (dist < distNearestToRandomPoint):
                nearestToRandomPoint = node
                distNearestToRandomPoint = dist

        # Get a vector towards the target point
        dirVector = [targetPoint[0]-nearestToRandomPoint.pos[0], targetPoint[1]-nearestToRandomPoint.pos[1]] # Create direction vector
        dirVector[0] /= distNearestToRandomPoint # Normalize vector
        dirVector[1] /= distNearestToRandomPoint 

        # Move the point nearest to target point towards target point, create new node
        newPoint = [nearestToRandomPoint.pos[0]+dirVector[0]*stepDistance, nearestToRandomPoint.pos[1]+dirVector[1]*stepDistance]
        if ( newPoint[0] >= 0.0 ) and ( newPoint[0] < domain.width ) and ( newPoint[1] >= 0.0 ) and ( newPoint[1] < domain.height ): # within map
            newPointRect = Rectangle(newPoint[0], newPoint[1], 0.1, 0.1)
            if ( not domain.CheckOverlap(newPointRect) ): # Not inside of an obstacle
                newNode = Node(newPoint[0], newPoint[1], nearestToRandomPoint)
                treeNodes.append(newNode) # Create new node parented to its nearest node
                # If node near goal then stop searching
                if getDistanceBetweenPoints(newNode.pos, goal) <= goalRadius:
                    nodeNearGoal = newNode
                    break

    return treeNodes, nodeNearGoal

###############################################################################################
# Main
###############################################################################################

def main( argv = None, showPlot = True, problemSpace = None, initial = None, goals = None ):
    if ( argv == None ):
        argv = sys.argv[1:]

    width = 10.0
    height = 10.0
    obstacleCount = 15
    obstacleWidth = 5.0
    obstacleHeight = 5.0

    goalRadius = 0.15
    biasToGoal = 0.5
    stepDistance = 0.15
    maxSteps = 50000

    ##############
    # Create path planning problem
    if problemSpace == None:
        problemSpace = PathPlanningProblem( width, height, obstacleCount, obstacleWidth, obstacleHeight)
    if initial == None or goals == None:
        initial, goals = problemSpace.CreateProblemInstance()
    goal = goals[0]

    fig = plt.figure()
    ax = fig.add_subplot(1,2,1, aspect='equal')
    ax.set_xlim(0.0, width)
    ax.set_ylim(0.0, height)

    ##############
    # RRT
    startTime = time.time()
    path, nodeNearGoal = ExploreDomain( problemSpace, initial, goal, maxSteps, stepDistance, biasToGoal, goalRadius)
    timeTaken = time.time() - startTime

    ax.set_title('Vacuuming Domain')

    # Draw the path
    pathX = []
    pathY = []
    for node in path:
        if node.parent:
            plt.plot([node.pos[0], node.parent.pos[0]], [node.pos[1], node.parent.pos[1]], '#7777aa', lw=2, zorder=10000)
        ax.add_patch(plt.Circle((node.pos[0], node.pos[1]), 0.025, color="#0000aa", zorder=10002))
        pathX.append(node.pos[0])
        pathY.append(node.pos[1])

    pathDistance = 0
    walkNode = nodeNearGoal
    while walkNode and walkNode.parent:
        plt.plot([walkNode.pos[0], walkNode.parent.pos[0]], [walkNode.pos[1], walkNode.parent.pos[1]], '#00aaff', lw=2, zorder=10001)
        ax.add_patch(plt.Circle((walkNode.pos[0], walkNode.pos[1]), 0.025, color="#99ccff", zorder=10003))
        pathDistance += getDistanceBetweenPoints(walkNode.pos, walkNode.parent.pos)
        walkNode = walkNode.parent
    if nodeNearGoal:
        plt.plot([nodeNearGoal.pos[0], goal[0]], [nodeNearGoal.pos[1], goal[1]], '#00aaff', lw=2)
        pathDistance += getDistanceBetweenPoints(nodeNearGoal.pos, goal)

    ##############
    # Draw map
    # Draw obstacles
    for o in problemSpace.obstacles:
        ax.add_patch(o.patch)

    # Draw initial position
    ax.add_patch(plt.Circle((initial[0], initial[1]), 0.05, color="#ff0000", zorder=math.inf))

    # Draw goal
    ax.add_patch(plt.Circle((goal[0], goal[1]), goalRadius, color="#ffff00"))
    ax.add_patch(plt.Circle((goal[0], goal[1]), 0.05, color="#00ff00", zorder=math.inf))

    # Output extra data
    ax.set_title('Time taken: {0:.5f}\nDistance: {1:.5f}\nNodes: {2}'.format(timeTaken, pathDistance, len(path)))

    ##############
    # Draw heatmap
    ax = fig.add_subplot(1,2,2)

#    x,y,z = problemSpace.CalculateCoverage(path, 0.5)
#    X,Y = np.meshgrid(x,y)
#    Z = z
#    ax.plot_surface(X,Y,Z, rstride=1, cstride=1, cmap=cm.coolwarm)

    heatmap, x, y = np.histogram2d(pathX, pathY, bins = 50, range=[[0.0, problemSpace.width], [0.0, problemSpace.height]])
    coverage = float( np.count_nonzero(heatmap) ) / float( len(heatmap) * len(heatmap[0]))
    extent = [ x[0], x[-1], y[0], y[-1]]
    ax.set_title('Random Walk\nCoverage {0}'.format(coverage))
    plt.imshow(np.rot90(heatmap))
    plt.colorbar()

    ##############
    # Show plot
    if showPlot:
        plt.show()

###############################################################################################

if ( __name__ == '__main__' ):
    main()

