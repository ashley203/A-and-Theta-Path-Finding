#imports required
from cmath import inf

import matplotlib.pyplot as plt
from matplotlib.pyplot import grid

import numpy as np

import math

import time

#import heapq
from MinHeap import MinHeap

# read text file (if file not found, allows you to retry) 
while True:
    try:
        grid_file_name = input("Please enter the name of the grid you would like to use (ex: 'grid1.txt')\n")
        with open(grid_file_name) as f:
            
            lines = f.readlines()
            line_num = 0

            for line in lines:
                line_num += 1

                # get start vertex
                if line_num == 1:
                    start_string = ",".join(line.split(" "))
                    start = tuple(map(int, start_string.split(',')))

                # get goal vertex
                elif line_num == 2:
                    goal_string = ",".join(line.split(" "))
                    goal = tuple(map(int, goal_string.split(',')))
                
                # get grid dimensions and intialize grid
                elif line_num == 3:
                    grid_len = int(line.split(" ")[0]) #col
                    grid_height = int(line.split(" ")[1]) #row
                    grid = np.zeros((grid_height,grid_len))

                else:
                    # set blocked squares
                    x_coord = int(line.split(" ")[0])
                    y_coord = int(line.split(" ")[1])
                    grid[x_coord][y_coord] = int(line.split(" ")[2].strip())
        break
    except FileNotFoundError:
        print("File not found.")
        continue

#Calculates heuristic value 
def heuristic(x,y):
    first_term = np.sqrt(2)*min(abs(x-goal[0]),abs(y-goal[1]))
    second_term = max(abs(x-goal[0]),abs(y-goal[1]))
    third_term = min(abs(x-goal[0]),abs(y-goal[1]))

    return first_term+second_term-third_term

#Calculates the cost from one node to another
def cost(self, other):
    x_diff = other[0] - self[0]
    y_diff = other[1] - self[1]
    return math.sqrt(pow(x_diff, 2) + pow(y_diff, 2))

#A* Algorithm
def a_star():
    start_time = time.time()
    #Initalizing open list, closed list, path
    fringe = MinHeap()
    closed_list = set()
    parents = {}

    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

    #Initalizing g score, and f score
    global g_score_a_star 
    if 'g_score_a_star' not in globals():
        g_score_a_star = {start:0}
    global f_score_a_star 
    
    f_score_a_star = {start:heuristic(start[0], start[1])}

    #Pushing the start node into the fringe
    fringe.insert((f_score_a_star[start], g_score_a_star[start], start))
 
    #For as long as the fringe is populated
    while fringe:

        #Min f score node
        current_node = fringe.pop()
        
        #If the current node is the goal
        if current_node == goal:
            path = []

            #Building the path
            print("A* Time: ", time.time()-start_time)
            while current_node in parents:
                path.append(current_node)
                current_node = parents[current_node]
            
            return path

        #Add current node to the list of explored nodes
        closed_list.add(current_node)

        blockedSquares=computeBlockedSquares(current_node)

        #Adding neighbors
        for i, j in neighbors:
            #Getting new node
            neighbor = current_node[0] + i, current_node[1] + j

            #Calculating the new g score
            temp_g_score = g_score_a_star[current_node] + cost(current_node, neighbor)

            if isValid(i, j, blockedSquares) == 0: 
                continue
 
            #Checking closed list
            if neighbor in closed_list and temp_g_score >= g_score_a_star.get(neighbor, 0):
                continue
 
            #Update vertex
            if  temp_g_score < g_score_a_star.get(neighbor, 0) or not fringe.containsNode(neighbor):
                parents[neighbor] = current_node
                g_score_a_star[neighbor] = temp_g_score
                f_score_a_star [neighbor] = temp_g_score + heuristic(neighbor[0], neighbor[1])
                
                fringe.insert((f_score_a_star[neighbor], g_score_a_star[neighbor], neighbor))

    return False

def line_of_sight(self, other):
    self_x = self[0]
    self_y = self[1]
    other_x = other[0]
    other_y = other[1]

    f = 0

    dy = other_y - self_y
    dx = other_x - self_x

    if dy < 0:
        dy *= -1
        sign_y = -1
    else:
        sign_y = 1
    if dx < 0:
        dx *= -1
        sign_x = -1
    else:
        sign_x = 1
    
    if dx >= dy:
        while(self_x != other_x):
            f += dy

            if f >= dx:
                try:
                    if grid[int(self_x + ((sign_x - 1)/2)), int(self_y + ((sign_y - 1)/2))] == 1:
                        return False
                    self_y += sign_y
                    f -= dx
                except IndexError:
                    return False
            try:
                if f != 0 and grid[int(self_x + ((sign_x - 1)/2)), int(self_y + ((sign_y - 1)/2))] == 1:
                    return False
            except IndexError:
                return False
            try:
                if dy == 0 and grid[int(self_x + ((sign_x - 1)/2)), int(self_y + ((sign_y - 1)/2))] == 1 and grid[int(self_x + ((sign_x - 1)/2)), self_y - 1] == 1:
                    return False
                self_x += sign_x
            except IndexError:
                return False
    else:
        while(self_y != other_y):
            f += dx

            if f >= dy:
                try:
                    if grid[int(self_x + ((sign_x - 1)/2)), int(self_y + ((sign_y - 1)/2))] == 1:
                        return False
                    self_x += sign_x
                    f -= dy
                except IndexError:
                    return False
            try:
                if f != 0 and grid[int(self_x + ((sign_x - 1)/2)), int(self_y + ((sign_y - 1)/2))] == 1:
                    return False
            except IndexError:
                return False
            try:
                if dx == 0 and grid[self_x, int(self_y + ((sign_y - 1)/2))] and grid[self_x-1, int(self_y + ((sign_y - 1)/2))]:
                    return False
                self_y += sign_y
            except IndexError:
                return False
            
    return True  

def theta_star():
    start_time = time.time()
    num_comps_los = 0

    #Initalizing open list, closed list, path
    fringe = MinHeap()
    closed_list = set()
    parents = {start:start}

    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

    #Initalizing g score, and f score
    global g_score
    if 'g_score' not in globals():
        g_score = {start:0}
    global f_score
    if 'f_score' not in globals():
        f_score = {start:cost(start, goal)}

    #Pushing the start node into the fringe
    fringe.insert((f_score[start], g_score[start], start))
 
    #For as long as the fringe is populated
    while fringe:

        #Min f score node
        current_node = fringe.pop()

        blockedSquares=computeBlockedSquares(current_node)
        valid_points = []

        num_comps_los += 1
        if not line_of_sight(parents[current_node], current_node):
            parent_candidates = []
            for i, j in neighbors:
                if isValid(i, j, blockedSquares) == 0:
                    continue
                valid_points.append(neighbor)
                neighbor = current_node[0] + i, current_node[1] + j
                parent_candidates.append(neighbor)
            intersection = set.intersection(closed_list, set(parent_candidates))

            new_parent = (0,0)
            parent_cost = float('inf')
            for point in intersection:
                temp_score = g_score[point]+cost(point, current_node)
                if(temp_score<parent_cost):
                    parent_cost = temp_score
                    new_parent = point

            parents[current_node] = new_parent
            g_score[current_node] = parent_cost

        #If the current node is the goal
        if current_node == goal:
            path = []

            #Building the path
            print("line of sight checks: ", num_comps_los)
            print("Theta* Time: ", time.time()-start_time)
            while current_node in parents and current_node != start:
                path.append(current_node)
                current_node = parents[current_node]

            return path

        #Add current node to the list of explored nodes
        closed_list.add(current_node)

        #Adding neighbors
        for i, j in neighbors:
            #Getting new node
            neighbor = current_node[0] + i, current_node[1] + j

            #Calculating the new g score
            temp_g_score = g_score[current_node] + cost(current_node, neighbor)

            if isValid(i, j, blockedSquares) == 0: 
                continue
            
            #Checking closed list
            if neighbor in closed_list and temp_g_score >= g_score.get(neighbor, 0):
                continue
 
            #Update vertex
            temp_g_score = g_score[parents[current_node]] + cost(parents[current_node],neighbor)
            if  temp_g_score < g_score.get(neighbor, 0) or not fringe.containsNode(neighbor):
                parents[neighbor] = parents[current_node]
                g_score[neighbor] = temp_g_score
                f_score[neighbor] = temp_g_score + cost(neighbor, goal)
                
                fringe.insert((f_score[neighbor], g_score[neighbor], neighbor))
 
    return False

def on_click_a_star(event):
    x = math.floor(event.xdata)
    y = math.floor(event.ydata)

    coord = (y,x)
    blocked = grid[y][x]

    print("Coordinate: ", coord, " Blocked: ", blocked)
    if coord in g_score_a_star:
        print("G Score: ", g_score_a_star[coord])
        print("F Score: ", f_score_a_star[coord])
        print("H Score: ", heuristic(coord[0],coord[1]),"\n")
    else:
        print("No score (F,G,or,H) was calculated for this square\n")

def on_click_theta_star(event):
    x = math.floor(event.xdata)
    y = math.floor(event.ydata)

    coord = (y,x)
    blocked = grid[y][x]

    print("Coordinate: ", coord, " Blocked: ", blocked)
    if coord in g_score:
        print("G Score: ", g_score[coord])
        print("F Score: ", f_score[coord])
        print("H Score: ", cost(coord,goal),"\n")
    else:
        print("No score (F,G,or,H) was calculated for this square\n")
    
def run_a_star():
    if a_star():
        path = a_star()
        path = path + [start]
        path = path[::-1]
        
        path_length = 0
        for i in range(len(path)-1):
            path_length += cost(path[i],path[i+1])
        print("Path length: ", path_length)

        print(path)

        x_coords = []
        y_coords = []

        for i in (range(0,len(path))):
            x = path[i][0]
            y = path[i][1]

            x_coords.append(x)
            y_coords.append(y)

        fig,ax = plt.subplots(figsize=(30,30))
        ax.imshow(grid, cmap=plt.cm.BuGn, extent=(0, grid_len, grid_height, 0))
        ax.scatter(start[1],start[0], marker = "*", color = "purple", s = 200)
        ax.scatter(goal[1],goal[0], marker = "*", color = "green", s = 200)
        ax.plot(y_coords,x_coords, color = "red")

        plt.grid()

        plt.xticks(np.arange(0, grid_len,1),[])
        plt.yticks(np.arange(0, grid_height,1),[])

        fig.canvas.mpl_connect('button_press_event', on_click_a_star)

        plt.show()
    else:
        print("No Path Found")
        fig,ax = plt.subplots(figsize=(30,30))
        ax.imshow(grid, cmap=plt.cm.BuGn, extent=(0, grid_len, grid_height, 0))
        ax.scatter(start[1],start[0], marker = "*", color = "purple", s = 100)
        ax.scatter(goal[1],goal[0], marker = "*", color = "green", s = 100)

        plt.grid()

        plt.xticks(np.arange(0, grid_len,1),[])
        plt.yticks(np.arange(0, grid_height,1),[])
        plt.show()

def run_theta_star():
    if theta_star():
        path = theta_star()
        path = path + [start]
        path = path[::-1]

        path_length = 0
        for i in range(len(path)-1):
            path_length += cost(path[i],path[i+1])
        print("Path length: ", path_length)

        print(path)

        x_coords = []
        y_coords = []

        for i in (range(0,len(path))):
            x = path[i][0]
            y = path[i][1]

            x_coords.append(x)
            y_coords.append(y)


        fig, ax = plt.subplots(figsize=(30,30))
        ax.imshow(grid, cmap=plt.cm.BuGn, extent=(0, grid_len, grid_height, 0))
        ax.scatter(start[1],start[0], marker = "*", color = "purple", s = 200)
        ax.scatter(goal[1],goal[0], marker = "*", color = "green", s = 200)
        ax.plot(y_coords,x_coords, color = "red")

        plt.grid()

        plt.xticks(np.arange(0, grid_len,1))
        plt.yticks(np.arange(0, grid_height,1))

        fig.canvas.mpl_connect('button_press_event', on_click_theta_star)

        plt.show()
    else:
        print("No Path Found")

#return 1 is there is a path, 0 if no path
def BFS():
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
            blockedSquares = computeBlockedSquares(curr)
            for i, j in neighbors:
                neighbor = curr[0] + i, curr[1] + j
                if neighbor not in visited:
                    if isValid(i, j, blockedSquares) == 1:
                        queue.append(neighbor)
                        visited.add(neighbor)        
    return 0

#checks if out of bounds before finding if cell is blocked, 1 if blocked, 0 if unbl
def gridValue(x, y):
    if x >= grid_height or y >= grid_len:
        return 1
    else:
        return grid[x][y]

#1 for blocked, 0 for unblocked
def computeBlockedSquares(curr):
    squares = []
    for i in range(2):
        for j in range(2):
            square = curr[0] - i, curr[1] - j
            if(square[0] < 0 or square[1] < 0) or square[0] >= grid_height or square[1] >= grid_len:
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

val = input("Which algorithm would you like to run? A*, Theta*, or Both (A* will run first, followed by Theta*) ")

if val == "A*" or val == "a*":
    run_a_star()
elif val == "Theta*" or val == "theta*":
    run_theta_star()
elif val == "Both" or val == "both":
    run_a_star()
    run_theta_star()
else:
    print("Please run the program again with a proper algorithm choice!")
