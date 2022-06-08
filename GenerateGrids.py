#imports required
from cmath import inf
from fileinput import filename
import matplotlib.pyplot as plt
from matplotlib.pyplot import grid

import numpy as np

import random

# define the dimensions of the grid
grid_len = 100 # col
grid_height = 50 # row

# number of grids to be generated
number_grids = 50

#checks cells surrounding the vertex: 1 for blocked, 0 for unblocked
def computeBlockedSquares(grid, curr):
    squares = []
    for i in range(2):
        for j in range(2):
            square = curr[0] - i, curr[1] - j
            if square[0] < 0 or square[1] < 0 or square[0] >= grid_height or square[1] >= grid_len:
                squares.insert(0, 1)
            else:
                squares.insert(0, grid[square[0]][square[1]])
    return squares

#returns 1 if valid move, 0 if not a valid move
def isValid(i, j, blockedSquares):
    if i == -1:
        if j==-1:
            if blockedSquares[0] == 1:
                return 0
        elif j==0:
            if blockedSquares[0] == 1 and blockedSquares[1] == 1:
                return 0
        elif j==1:
            if blockedSquares[1] == 1:
                return 0
    elif i==0:
        if j==-1:
            if blockedSquares[0] == 1 and blockedSquares[2] == 1:
                return 0
        elif j==1:
            if blockedSquares[1] == 1 and blockedSquares[3] == 1:
                return 0
    elif i==1:
        if j==-1:
            if blockedSquares[2] == 1:
                return 0
        elif j==0:
            if blockedSquares[2] == 1 and blockedSquares[3] == 1:
                return 0
        elif j==1:
            if blockedSquares[3] == 1:
                return 0
    return 1

#return 1 is there is a path, 0 if no path
def BFS(grid, start, goal):
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(-1,-1),(1,-1),(-1,1),(1,1)]
    visited = set()
    queue = []
    queue.append(start)
    while queue:
        curr = queue.pop(0)
        visited.add(curr)
        if curr == goal:
            return 1
        else:
            blockedSquares = computeBlockedSquares(grid, curr)
            for i, j in neighbors:
                neighbor = curr[0] + i, curr[1] + j
                if neighbor not in visited:
                    if isValid(i, j, blockedSquares) == 1:
                        queue.append(neighbor)
                        visited.add(neighbor)        
    return 0

for num in range(1, number_grids + 1):
    # Generate a 2D array that will be randomly populated by 0s and 1s
    grid = np.zeros((grid_height,grid_len))

    # Block 10% of the squares
    grid_values_list = [1]*30 + [0]*70
    for ir, row in enumerate(grid):
        for ic, col in enumerate(row):
            grid[ir][ic] = random.choice(grid_values_list)

    # Create random start and goal vertices
    start = (random.randint(0, grid_height),random.randint(0, grid_len))
    goal = (random.randint(0, grid_height),random.randint(0, grid_len))

    # if valid path exists, save grid as text file
    if BFS(grid, start, goal) == 1:
        current_file_name = "grid" + str(num) + ".txt"
        with open(current_file_name, 'w') as f:
            f.write(' '.join(map(str, start)) + '\n')
            f.write(' '.join(map(str, goal)) + '\n')
            f.write(str(grid_len) + " " + str(grid_height) + '\n')
            for ir, row in enumerate(grid):
                for ic, col in enumerate(row):
                    f.write(str(ir) + " " + str(ic))
                    f.write(" " + str(int(grid[ir][ic])) + "\n")
