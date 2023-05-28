'''
> Purpose :
Module to perform pathplanning from source to destination using provided methods.
                                                                         [DFS,DFS_Shortest,Dijisktra,Astar]

> Usage :
You can perform pathplanning by
1) Importing the class (bot_pathplanner)
2) Creating its object
3) Accessing the object's function of (find_path_nd_display). 
E.g ( self.bot_pathplanner.find_path_nd_display(self.bot_mapper.Graph.graph, start, end, maze,method="a_star") )


> Inputs:
1) Graph extracted in mapping stage
2) Source & Destination
3) Maze Image
4) Method to Use [DFS,DFS_Shortest,Dijisktra,Astar]

> Outputs:
1) self.path_to_goal      => Computed Path from Source to destination [List of Cordinates]
2) self.img_shortest_path => Found path Overlayed (In Color) on Image

Author :
Haider Abbasi

Date :
6/04/22
'''
import cv2
import numpy as np
from numpy import sqrt

from PP_Algorithms.DFS import DFS
from PP_Algorithms.Dijkstra import Dijkstra
from PP_Algorithms.A_Star import A_Star

class Bot_PathPlanner():

    def __init__(self):
        self.DFS = DFS()
        self.Dijkstra = Dijkstra()
        self.A_Star = A_Star()
        
        self.path_to_goal = []
        self.img_shortest_path = []
        self.choosen_route = []
        

    @staticmethod
    def cords_to_pts(cords):
      return [cord[::-1] for cord in cords]

    def draw_path_on_maze(self,maze,shortest_path_pts,method):
        
        maze_bgr = cv2.cvtColor(maze, cv2.COLOR_GRAY2BGR)
        self.choosen_route = np.zeros_like(maze_bgr)

        rang = list(range(0,254,25))
        
        depth = maze.shape[0]
        for i in range(len(shortest_path_pts)-1):
            per_depth = (shortest_path_pts[i][1])/depth

            # Blue : []   [0 1 2 3 251 255 251 3 2 1 0] 0-depthperc-0
            # Green :[]     depthperc
            # Red :  [] 100-depthperc
            color = ( 
                      int(255 * (abs(per_depth+(-1*(per_depth>0.5)))*2) ),
                      int(255 * per_depth),
                      int(255 * (1-per_depth))
                    )
            cv2.line(maze_bgr,shortest_path_pts[i] , shortest_path_pts[i+1], color)
            cv2.line(self.choosen_route,shortest_path_pts[i] , shortest_path_pts[i+1], color,3)

        img_str = "maze (Found Path) [" +method +"]"

        cv2.imshow(img_str, maze_bgr)
        cv2.waitKey(1)
        
        if method == "Dijkstra":
            self.Dijkstra.shortest_path_overlayed = maze_bgr
        elif method == "A_Star":
            self.A_Star.shortest_path_overlayed = maze_bgr
            
        self.img_shortest_path = maze_bgr.copy()
    
    def find_path_nd_display(self,graph,start,end,maze,method = "DFS"):

        Path_str = "Path"
        
        if method == "DFS":
            paths = self.DFS.get_paths(graph, start, end)
            path_to_display = paths[0]

        elif method == "DFS_Shortest":
            paths_N_Cost = self.DFS.get_paths_cost(graph, start, end)
            paths = paths_N_Cost[0]
            costs = paths_N_Cost[1]

            min_cost = min(costs)

            path_to_display = paths[costs.index(min_cost)]     

        elif method == "Dijkstra":
            if not self.Dijkstra.shortestpath_found:
                print("Finding Shortest Routes")
                self.Dijkstra.find_best_routes(graph, start, end)
            
            path_to_display = self.Dijkstra.shortest_path
            Path_str = "Shortest "+ Path_str

        elif (method == "A_Star"):
            
            if not self.A_Star.shortestpath_found:
                print("Finding Shortest ROutes")
                self.A_Star.find_best_routes(graph, start, end)
            
            path_to_display = self.A_Star.shortest_path
            Path_str = "\nShortest "+ Path_str

        pathpts_to_display = self.cords_to_pts(path_to_display)
        self.path_to_goal = pathpts_to_display

        self.draw_path_on_maze(maze, pathpts_to_display, method)

 

