import matplotlib.pyplot as plt
from pathplanning import PathPlanningProblem
from RRT import main as rrt
from FBSP import main as fbsp

width = 10.0
height = 10.0
obstacleCount = 10
obstacleWidth = 5.0
obstacleHeight = 5.0

problemSpace = PathPlanningProblem( width, height, obstacleCount, obstacleWidth, obstacleHeight)
initial, goals = problemSpace.CreateProblemInstance()

fbsp(showPlot=False, problemSpace=problemSpace, initial=initial, goals=goals)
rrt(showPlot=False, problemSpace=problemSpace, initial=initial, goals=goals)
plt.show()