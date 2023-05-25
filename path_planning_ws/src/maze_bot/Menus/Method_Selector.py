#Import the required Libraries
from tkinter import *
from tkinter import ttk


class Method_Selector():
    def __init__(self) -> None:
        self.Method = ""

        #Create an instance of Tkinter frame
        self.win = Tk()
        #Set the geometry of the Tkinter frame
        self.win.geometry("200x150")

        self.count = False

    def __DFS(self):
        self.Method = "DFS"
        self.win.destroy()

    def __DFS_Shortest(self):
        self.Method = "DFS_Shortest"
        self.win.destroy()

    def __Dijkstra(self):
        self.Method = "Dijkstra"
        self.win.destroy()

    def __A_Star(self):
        self.Method = "A_Star"
        self.win.destroy()
        

    def Selector(self):
        
        label = ttk.Label(self.win, text = "Choose Path Planning Method \n")
        label.pack() 

        button_dfs = ttk.Button(self.win, text = "DFS", command = self.__DFS).pack()

        button_dfs_sh = ttk.Button(self.win, text = "DFS_Shortest", command = self.__DFS_Shortest)
        button_dfs_sh.pack()
        
        button_dfs_sh = ttk.Button(self.win, text = "Dijkstra", command = self.__Dijkstra)
        button_dfs_sh.pack()
        
        button_dfs_sh = ttk.Button(self.win, text = "A_Star", command = self.__A_Star)
        button_dfs_sh.pack()

        self.win.update()
        self.count = True
    
    def run(self):
        if not self.count:
            self.Selector()

        self.win.quit()
        self.win.update()
