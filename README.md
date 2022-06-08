# AI-hw1

Authors: Ashley Mathai (asm268), Manasvi Medam (mm2883), Caitlin Moy (cam693)

This project contains two files: FindPath.py and GenerateGrids.py

## To run the program: FindPath.py
1. Put your appropriately-formatted (according to the assignment instructions) text file into the project directory
1. Run the program by first navigating to the project directory and typing 'python3 FindPath.py' in the terminal
1. The front-end will then prompt the user to enter the name of the grid text file they would like to run, and then prompt the user to specify which path-finding algorithm to run
1. A grid with a shortest path will pop up in a new window. If there is no path possible between the start and goal nodes, the console will print "No path found!"
1. Each cell can be clicked on- doing so will display information about the cell's coordinates, whether it is blocked (1) or unblocked (0), and, if during the path-finding algorithm execution the cell is considered, the cell's f, g, and h-values.

## GenerateGrids.py
This file was used for testing. It creates 50 100x50 grids within the project directory with 10% blocked squares, and randomized start and goal nodes. Each grid text file name is as follows: grid1.txt, grid2.txt, ... grid50.txt.
