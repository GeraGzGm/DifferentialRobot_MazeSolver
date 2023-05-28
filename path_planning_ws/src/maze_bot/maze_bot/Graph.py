import numpy
from collections import deque

class Graph():
    def __init__(self) -> None:
        self.graph = {}

        self.start = 0
        self.end = 0

    def Add_Vertex(self, vertex, neighbor = None, case = None, cost = None):

        if vertex in self.graph.keys():
            self.graph[vertex][neighbor] = {}
            self.graph[vertex][neighbor]["case"] = case
            self.graph[vertex][neighbor]["cost"] = cost
        else:
            self.graph[vertex] = {}
            self.graph[vertex]["case"] = case
    
    def Display_Graph(self):

        for key, value in self.graph.items():
            print("key: {} , value: {}".format(key, value))
