from .Dijkstra import Dijkstra

from math import sqrt

class A_Star(Dijkstra):

    def __init__(self):

        super().__init__()
        # Counter added to track total nodes visited to 
        #               reach goal node
        self.astar_nodes_visited = 0

    # Heuristic function ( One of the components required to compute total cost of any node ) 
    @staticmethod
    def euc_d(a,b):
        return sqrt( pow( (a[0]-b[0]),2 ) + pow( (a[1]-b[1]),2 ) )


    # Function Ovverrriding
    def find_best_routes(self,graph,start,end):

        # Teaking the first item of the list created by list comprehension
        # Which is while looping over the key value pair of graph. Return the pairs_idx that match the start key
        start_idx = [idx for idx, key in enumerate(graph.items()) if key[0]==start][0]
        print("Index of search key : {}".format(start_idx))

        # Cost of reaching that node from start
        cost2node = []
        # Distanc list storing dist of each node
        dist = []       
        # Storing found shortest subpaths [format ==> (parent_idx = closest_child_idx)]
        parent = []

        # Set size of minHeap to be the total no of keys in the graph.
        self.minHeap.size = len(graph.keys())

        for idx,v in enumerate(graph.keys()):

            # Initializing the cost 2 node with Infinity
            cost2node.append(1e7)

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

        # We set the cost of reaching the start node to 0
        cost2node[start_idx] = 0
        # Total cost(Start Node) = Cost2Node(Start) + Heuristic Cost(Start,End)
        dist[start_idx] = cost2node[start_idx] + self.euc_d(start, end)
        # Decrease Key as new found dist of start_vertex is 0
        self.minHeap.decreaseKey(start_idx, dist[start_idx])

        # Loop as long as Priority Queue has nodes.
        while(self.minHeap.size!=0):
            # Counter added for keeping track of nodes visited to reach goal node
            self.astar_nodes_visited += 1

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

                    #if we have not found shortest distance to v + new found cost2Node < known cost2node ==> Update Cost2node for neighbor node
                    if ( self.minHeap.isInMinHeap(v_idx) and (dist[u_idx]!=1e7) and
                       (    (graph[u][v]["cost"] + cost2node[u_idx]) < cost2node[v_idx] )    ):

                       cost2node[v_idx] = graph[u][v]["cost"] + cost2node[u_idx]
                       dist[v_idx] = cost2node[v_idx] + self.euc_d(v, end)
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