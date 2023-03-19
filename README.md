# ENPM661_Proj3_AStar
UMD ENPM661 Project 3 A-Star

# Student Information
Authors: Brendan Neal and Adam Lobo

# Project Information
Goal: Solve a maze using the A* Algorithm

File Name: a_start_adam_brendan.py

Recommended IDE: Visual Studio Code

Python Version: 3

# Github Repository Links

Brendan: https://github.com/brendanneal12/ENPM661_Proj3_AStar

Adam: https://github.com/AdazInventionz/ENPM661-Project-3

# Libraries Used
numpy, from matplotlib: pyplot, opencv, math, timeit, from queue: PriorityQueue

# How to Run Code

1. Hit Run
2. Prompted by the terminal, enter the initial state X, Y, and Theta with spaces separating them. Theta should be a multiple of 30 (0,30,60,270,etc.)
3. If your initial state is in an obstacle or outside of the workspace, you will be prompted to restart.
4. Prompted by the terminal, enter the goal state X, Y, and Theta with spaces separating them. Theta should be a multiple of 30 (0,30,60,270,etc.)
5. If your goal state is in an obstacle or outside of the workspace, you will be prompted to restart.
6. Enter the Robot Radius
7. Enter the Step Size.
8. Observe the obstacle space setup. Obstacles are white, the desired clearance (5 pixels) is in light gray, and the additional robot radius clearance is in dark grey.
9. Close the window to begin the A* search.
10. While the search is running, the current node state (popped from the open list) is printed to the terminal.
11. Once the goal is reached, the total cost and the time it took to search will be printed to the terminal. The final map will be displayed to your screen with the searched nodes in green, and the optimal path in magenta.
12. Close the window to start the visualization.
13. Once the visualization is complete, close the window to end the program.

# IMPORTANT NOTES PLEASE READ
1. For cases far away the start point, the search can take upwards of 10-15 minutes. We're not generating repeat nodes, but we have noticed that getting through the two rectangles is a bit of a challenge. Please be patient.
2. The visualization is also quite slow. Please be patient.
3. For smaller step sizes, the search appears to be  large blob, but if you zoom in you can see the individual moves quite well.
4. Please click the following link in order to view an example output video that is produced by our code. Initial State is 5 5, Goal State is 300 225. I cannot commit the video directly to GitHub since the file size is way too large: https://drive.google.com/drive/u/0/folders/11FW4Fq-76BoGIyPZIVHYzmYJJwSpE-TE


