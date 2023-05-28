import cv2
import numpy as np
from skimage.morphology import skeletonize , thin

from .Graph import Graph

class Mapping():
    def __init__(self) -> None:
        
        self.Mapping_Done = False
        self.Crop_pixels = 5
        self.Draw = True

        self.Graph = Graph()
        #Left. Up-Left, Up, Up-Right
        self.Directions = [False, False, False, False]

        self.Maze_Connect = []
        self.maze = []

    def Reset_Connection(self):
        self.Directions = [False, False, False, False]

    def Display_GraphConnections(self, _Node, N_Node, case = '', color = (0,0,255), Display = False):

        if Display:
            self.Maze_Connect = cv2.line(self.Maze_Connect,(_Node[1],_Node[0]),(N_Node[1],N_Node[0]), color, 1)
            cv2.imshow("Nodes Conected", self.Maze_Connect)
            cv2.waitKey(1)

    def Neihbor_Connection(self, Maze, r, c, case, Lstep = 1, Upstep = 0, totalNodes = 0):

        _Node = (r,c)

        if Maze[r-Upstep][c-Lstep] > 0:
            nei_Node = (r-Upstep, c-Lstep)

            if nei_Node in self.Graph.graph.keys():

                neighbors_case = self.Graph.graph[nei_Node]["case"]
                cost = max(abs(Lstep), abs(Upstep))
                totalNodes += 1

                self.Graph.Add_Vertex(_Node, nei_Node, neighbors_case, cost)
                self.Graph.Add_Vertex(nei_Node, _Node, case, cost)

                if not self.Directions[0]: #Left
                    self.Display_GraphConnections(_Node, nei_Node, "LEFT", (0,0,255))
                    self.Directions[0] = True
                    Lstep = 1
                    Upstep = 1
                    self.Neihbor_Connection(Maze, r, c, case, Lstep, Upstep, totalNodes)
                
                if not self.Directions[1]: #Up-Left
                    self.Display_GraphConnections(_Node, nei_Node, "UPLEFT", (0,120,255))
                    self.Directions[1] = True
                    Lstep = 0
                    Upstep = 1
                    self.Neihbor_Connection(Maze, r, c, case, Lstep, Upstep, totalNodes)
                
                if not self.Directions[2]: #Up
                    self.Display_GraphConnections(_Node, nei_Node, "UP", (255,0,255))
                    self.Directions[2] = True
                    Lstep = -1
                    Upstep = 1
                    self.Neihbor_Connection(Maze, r, c, case, Lstep, Upstep, totalNodes)

                if not self.Directions[3]: #Up-Right
                    self.Display_GraphConnections(_Node, nei_Node, "UPRIGHT", (0,0,255))
                    self.Directions[3] = True
                    self.Neihbor_Connection(Maze, r, c, case, Lstep, Upstep, totalNodes)
            
            if not self.Directions[3]:
                if not self.Directions[0]:
                    Lstep += 1
                elif not self.Directions[1]:
                    Lstep += 1
                    Upstep += 1
                elif not self.Directions[2]:
                    Upstep += 1
                elif not self.Directions[3]:
                    Lstep -= 1
                    Upstep += 1

                self.Neihbor_Connection(Maze, r, c, case, Lstep, Upstep, totalNodes)
            
        else:
            if not self.Directions[0]:
                self.Directions[0] = True
                Lstep = 1
                Upstep = 1

                self.Neihbor_Connection(Maze, r, c, case, Lstep, Upstep, totalNodes)
            
            elif not self.Directions[1]:
                self.Directions[1] = True
                Lstep = 0
                Upstep = 1

                self.Neihbor_Connection(Maze, r, c, case, Lstep, Upstep, totalNodes)

            elif not self.Directions[2]:
                self.Directions[2] = True
                Lstep = -1
                Upstep = 1

                self.Neihbor_Connection(Maze, r, c, case, Lstep, Upstep, totalNodes)

            elif not self.Directions[3]:
                self.Directions[3] = True
                Lstep = 0
                Upstep = 0

                return 

    @staticmethod
    def GetNeighbors(Maze: np.array, row: int, col: int) -> np.array:
        """Obtain the cells next to the cell

        Args:
            Maze (np.array)
            row (int)
            col (int)

        Returns:
            np.array: [upLeft,up,upRight,right,downRight,down,leftDown,left, n_Paths]
        """
        
        Row, Col = Maze.shape

        maze = cv2.threshold(Maze, 1, 1, cv2.THRESH_BINARY)[1]

        rows = maze.shape[0]
        cols = maze.shape[1]

        nei = np.array( [False,False,False,False,False,False,False,False] )

        if(Maze[row-1][col-1] > 0):
            nei[0] = True
        
        if(Maze[row-1][col] > 0):
            nei[1] = True

        if(Maze[row-1][col+1] > 0):
            nei[2] = True

        if(Maze[row][col+1] > 0):
            nei[3] = True

        if(Maze[row+1][col+1] > 0):
            nei[4] = True
        
        if(Maze[row+1][col] > 0):
            nei[5] = True
        
        if(Maze[row+1][col-1] > 0):
            nei[6] = True
        
        if(Maze[row][col-1] > 0):
            nei[7] = True
        
        paths = np.sum(nei == True)

        return np.hstack((nei, np.array(paths)))
    
    def InterestPointsExtraction(self, Maze: np.array):

        self.Graph.graph.clear()
        self.Maze_Connect = cv2.cvtColor(Maze, cv2.COLOR_GRAY2BGR)
        #cv2.namedWindow("Nodes Conected", cv2.WINDOW_FREERATIO)

        Turns = 0
        Junc_3 = 0
        Junc_4 = 0

        #Obtain only the main path
        Path = np.argwhere(Maze == 255)

        Maze_color = cv2.cvtColor(Maze, cv2.COLOR_GRAY2BGR)
        row, col = Maze.shape

        for r,c in Path:
            #self.Maze_Connect = cv2.cvtColor(Maze, cv2.COLOR_GRAY2BGR)
            nei = self.GetNeighbors(Maze, r, c)

            if((r == 0) or (r > row - 1) or (c == 0) or (c == col - 1)):
                #Check wheter it is a start/end point or not
                if( r == 0 ):
                    #Start
                    Maze_color[r][c] = (0,255,00)
                    self.Graph.Add_Vertex((r,c), None, "_Start_", None)
                    self.Graph.start = (r,c)
                else:
                    #End
                    Maze_color[r][c] = (0,0,255)
                    self.Graph.Add_Vertex((r,c), None, "_End_", None)
                    self.Graph.end = (r,c)

                    self.Reset_Connection()
                    self.Neihbor_Connection(Maze, r,c, "_End_")

            else:
                #Dead-End 
                if(nei[-1] == 1):
                    Maze_color[r][c] = (255,32,255) 
                    cv2.circle(Maze_color, (c,r), 2 , (255,32,255), 2)

                    self.Graph.Add_Vertex((r,c), None, "_DeadEnd_", None)
                    self.Reset_Connection()
                    self.Neihbor_Connection(Maze, r,c, "_DeadEnd_")
                
                #Corner
                elif(nei[-1] == 2):
                    if not ((nei[1]>0 and nei[5]>0) or (nei[3]>0 and nei[7]>0)):
                        Maze_color[r][c] = (130,200,255) 
                        #cv2.circle(Maze_color, (c,r), 5 , (130,200,255), 1)

                        self.Graph.Add_Vertex((r,c), None, "_Turn_", None)
                        self.Reset_Connection()
                        self.Neihbor_Connection(Maze, r,c, "_Turn_")
                        Turns += 1

                #3-Junction
                elif(nei[-1] == 3):
                    Maze_color[r][c] = (130,200,0) 
                    cv2.circle(Maze_color, (c,r), 5 ,  (130,200,0) , 1)

                    self.Graph.Add_Vertex((r,c), None, "_3-Junct_", None)
                    self.Reset_Connection()
                    self.Neihbor_Connection(Maze, r,c, "_3-Junct_")
                    Junc_3 += 1
                
                #4-Junction
                elif(nei[-1] > 3):
                    Maze_color[r][c] = (255,0,200) 
                    cv2.circle(Maze_color, (c,r), 5 ,  (130,200,0) , 1)

                    self.Graph.Add_Vertex((r,c), None, "_4-Junct_", None)
                    self.Reset_Connection()
                    self.Neihbor_Connection(Maze, r,c, "_4-Junct_")
                    Junc_4 += 1

        
        cv2.imshow("IP" , Maze_color)
        cv2.waitKey(1)
                  
    def Graphify(self, Maze: np.array):

        if not self.Mapping_Done:
            #Create a thin path like the given maze
            Thinned_maze = cv2.ximgproc.thinning(Maze, None, thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)

            #Do a clean-up process for the given maze
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
            Thinned_maze = cv2.morphologyEx(Thinned_maze, cv2.MORPH_DILATE, kernel, iterations=1)
            _,Bw_Maze = cv2.threshold(Thinned_maze, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
            Bw_Maze = skeletonize(Bw_Maze, method='lee').astype(np.uint8)

            #Crop maze to remove noise from first columns and rows
            maze_shape = Maze.shape
            Bw_Maze = Bw_Maze[self.Crop_pixels:maze_shape[0]-self.Crop_pixels, self.Crop_pixels:maze_shape[1]-self.Crop_pixels]
            
            #Draw obtained maze into the Input Maze parameter
            Extracted_Maze = Maze[self.Crop_pixels:maze_shape[0]-self.Crop_pixels, self.Crop_pixels:maze_shape[1]-self.Crop_pixels]
            Extracted_Maze = cv2.cvtColor(Extracted_Maze, cv2.COLOR_RGB2BGR )
            Extracted_Maze[Bw_Maze > 0] = (255, 0, 255)

            self.InterestPointsExtraction(Bw_Maze)
            
            #cv2.imshow("Thinned Maze", Thinned_maze)
            
            #cv2.imshow("Bw_Maze", Bw_Maze)

            self.maze = Bw_Maze
            cv2.imshow("Extracted_Maze", Extracted_Maze)
            cv2.waitKey(1)

