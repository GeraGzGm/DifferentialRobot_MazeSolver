import numpy as np
import cv2

from functools import lru_cache


class DFS():

    # A not so simple problem, 
    #    Lets try a recursive approach
    
    def get_paths(self,graph,start,end,path = []):
        
        # Update the path to where ever you have been to
        path = path + [start]

        # 2) Define the simplest case
        if (start == end):
            return [path]

        # Handle boundary case [start not part of graph]
        if start not in graph.keys():
            return []
        # List to store all possible paths from start to end
        paths = []

        # 1) Breakdown the complex into simpler subproblems
        for node in graph[start].keys():
            #  Recursively call the problem with simpler case
            # 3) Once encountered base cond >> Roll back answer to solver subproblem
            # Checking if not already traversed and not a "case" key
            if ( (node not in path) and (node!="case") ):
                new_paths = self.get_paths(graph,node,end,path)
                for p in new_paths:
                    paths.append(p)

        return paths

    
    # Retrieve all possible paths and their respective costs to reaching the goal node
    @staticmethod
    def get_paths_cost(graph,start,end,path=[],cost=0,trav_cost=0):

        # Update the path and the cost to reaching that path
        path = path + [start]
        cost = cost + trav_cost

        # 2) Define the simplest case
        if start == end:
            return [path],[cost]
        # Handle boundary case [start not part of graph]
        if start not in graph.keys():
            return [],0

        # List to store all possible paths from point A to B
        paths = []
        # List to store costs of each possible path to goal
        costs = []

        # Retrieve all connections for that one damn node you are looking it
        for node in graph[start].keys():

            # Checking if not already traversed and not a "case" key
            if ( (node not in path) and (node!="case") ):

                new_paths,new_costs = DFS.get_paths_cost(graph,node, end,path,cost,graph[start][node]['cost'])

                for p in new_paths:
                    paths.append(p)
                for c in new_costs:
                    costs.append(c)
        
        return paths,costs