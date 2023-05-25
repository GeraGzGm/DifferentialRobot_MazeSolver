class Heap():
    def __init__(self):
        # Priority queue will be stored in an array (list of list containing vertex and their resp distance)
        self.array = []
        # Counter to track nodes left in priority queue
        self.size = 0
        # Curr_pos of each vertex is stored
        self.posOfVertices = []

    # create a minheap node => List(vertex,distance)
    def new_minHeap_node(self,v,dist):
        return([v,dist])

    # Swap node a (List_A) with node b (List_b)
    def swap_nodes(self,a,b):
        temp = self.array[a]
        self.array[a] = self.array[b]
        self.array[b] = temp

    # Convert any part of complete tree in minHeap in O(nlogn) time
    def minHeapify(self,node_idx):
        smallest = node_idx
        left = (node_idx*2)+1
        right = (node_idx*2)+2

        if ((left<self.size) and (self.array[left][1]<self.array[smallest][1])):
            smallest = left
        if ((right<self.size) and (self.array[right][1]<self.array[smallest][1])):
            smallest = right

        # If node_idx is not the smallest
        if(smallest != node_idx):
            # Update the positions to keep smallest on top
            self.posOfVertices[self.array[node_idx][0]] = smallest
            self.posOfVertices[self.array[smallest][0]] = node_idx
            # Swap node_idx with smallest
            self.swap_nodes(node_idx, smallest)
            # Recursively call minHeapify until all subnodes part of minheap or no more subnodes left
            self.minHeapify(smallest)

    # extract top (min value) node from the min-heap => then minheapify to keep heap property
    def extractmin(self):

        # Handling boudary condtion
        if self.size == 0:
            return

        # root node (list[root_vertex,root_value]) extracted
        root = self.array[0]


        # Move Last node to top
        lastNode = self.array[self.size-1]
        self.array[0] = lastNode

        # Update the postion of vertices
        self.posOfVertices[root[0]] = self.size-1
        self.posOfVertices[lastNode[0]] = 0

        # Decrease the size of minheap by 1
        self.size-=1

        # Perform Minheapify from root
        self.minHeapify(0)
        # Return extracted root node to user
        return root

    # Update distance for a node to a new found shorter distance
    def decreaseKey(self,vertx,dist):
        
        # retreviing the idx of vertex we want to decrease value of
        idxofvertex = self.posOfVertices[vertx]

        self.array[idxofvertex][1] = dist

        # Travel up while complete heap is not heapified
        # While idx is valid and (Updated_key_dist < Parent_key_dist)
        while((idxofvertex>0) and (self.array[idxofvertex][1]<self.array[(idxofvertex-1)//2][1])):
            # Update position of parent and curr_node
            self.posOfVertices[self.array[idxofvertex][0]] = (idxofvertex-1)//2 
            self.posOfVertices[self.array[(idxofvertex-1)//2][0]] = idxofvertex

            # Swap curr_node with parent
            self.swap_nodes(idxofvertex, (idxofvertex-1)//2)

            # Navigate to parent and start process again
            idxofvertex = (idxofvertex-1)//2

    # A utility function to check if a given
    # vertex 'v' is in min heap or not
    def isInMinHeap(self, v):
 
        if self.posOfVertices[v] < self.size:
            return True
        return False