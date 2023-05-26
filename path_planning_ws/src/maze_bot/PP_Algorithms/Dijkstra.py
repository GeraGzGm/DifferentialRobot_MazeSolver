from .Heap import Heap

class Dijkstra():

    def __init__(self):
        
        # State variable 
        self.shortestpath_found = False
        # Once found save the shortest path
        self.shortest_path = []

        self.shortest_path_overlayed = []

        # instance variable assigned obj of heap class for implementing required priority queue
        self.minHeap = Heap()
        
        # Creating dictionaries to manage the world
        self.idxs2vrtxs = {}
        self.vrtxs2idxs = {}
        # Counter added to track total nodes visited to 
        #               reach goal node
        self.dijiktra_nodes_visited = 0

    def ret_shortestroute(self,parent,start,end,route):
        
        # Keep updating the shortest route from end to start by visiting closest vertices starting fron end
        route.append(self.idxs2vrtxs[end])
        
        # Once we have reached the start (maze_entry) => Stop! We found the shortest route
        if (end==start):
            return
        
        # Visit closest vertex to each node
        end = parent[end]
        # Recursively call function with new end point until we reach start
        self.ret_shortestroute(parent, start, end, route)

    def find_best_routes(self,graph,start,end):

        # Teaking the first item of the list created by list comprehension
        # Which is while looping over the key value pair of graph. Return the pairs_idx that match the start key
        start_idx = [idx for idx, key in enumerate(graph.items()) if key[0]==start][0]
        print("Index of search key : {}".format(start_idx))

        # Distanc list storing dist of each node
        dist = []       
        # Storing found shortest subpaths [format ==> (parent_idx = closest_child_idx)]
        parent = []

        # Set size of minHeap to be the total no of keys in the graph.
        self.minHeap.size = len(graph.keys())

        for idx,v in enumerate(graph.keys()):

            # Initialize dist for all vertices to inf
            dist.append(1e7)
            # Creating BinaryHeap by adding one node([vrtx2idx(v),dist]) at a time to minHeap Array
            # So instead of vertex which is a tuple representing an Ip we pass in an index 
            self.minHeap.array.append(self.minHeap.new_minHeap_node(idx, dist[idx]))
            self.minHeap.posOfVertices.append(idx)

            # Initializing parent_nodes_list with -1 for all indices
            parent.append(-1)
            # Updating dictionaries of vertices and their positions
            self.vrtxs2idxs[v] = idx
            self.idxs2vrtxs[idx] = v

        # Set dist of start_idx to 0 while evertything else remains Inf
        dist[start_idx] = 0
        # Decrease Key as new found dist of start_vertex is 0
        self.minHeap.decreaseKey(start_idx, dist[start_idx])

        # Loop as long as Priority Queue has nodes.
        while(self.minHeap.size!=0):
            
            # Counter added for keeping track of nodes visited to reach goal node
            self.dijiktra_nodes_visited += 1

            # Retrieve the node with the min dist (Highest priority)
            curr_top = self.minHeap.extractmin()
            u_idx = curr_top[0]
            u = self.idxs2vrtxs[u_idx]

            # check all neighbors of vertex u and update their distance if found shorter
            for v in graph[u]:
                # Ignore Case node
                if v!= "case":

                    print("Vertex adjacent to {} is {}".format(u,v))
                    v_idx = self.vrtxs2idxs[v]

                    #if we have not found shortest distance to v + new found dist < known dist ==> Update dist for v
                    if ( self.minHeap.isInMinHeap(v_idx) and (dist[u_idx]!=1e7) and
                       (    (graph[u][v]["cost"] + dist[u_idx]) < dist[v_idx] )    ):

                       dist[v_idx] = graph[u][v]["cost"] + dist[u_idx]
                       self.minHeap.decreaseKey(v_idx, dist[v_idx])
                       parent[v_idx] = u_idx
            
            # End Condition: When our End goal has already been visited. 
            #                This means shortest part to end goal has already been found 
            #     Do   --->              Break Loop
            if (u == end):
                break
        
        shortest_path = []
        self.ret_shortestroute(parent, start_idx,self.vrtxs2idxs[end],shortest_path)
        
        # Return route (reversed) to start from the beginned
        self.shortest_path = shortest_path[::-1]
        self.shortestpath_found = True
