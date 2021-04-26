"""
Here we will implement the Graph/GeoMap(graph class in this file) using 
adjacency-list because we are implementing this graph datastructure
fro geo locations and finding shortest route between two locations/nodes 

-> for GeoLocation based graphs - Adjacency List (sparse graph)
-> for Social Graphs - Adjacency Matrix (dense graph)

"""

from collections import defaultdict
from sys import maxsize as INF
# import time

# define state/status for a node for dijkstra/A*
TEMP = 1 # un-relaxed state
PERMANENT = 2 # relaxed state

class Node:
    def __init__(self,name:int,x:float, y:float):
        self.name = name # name of node -> must be unique
        self.path_length = INF # initialize with INFINITY
        self.parent = None
        self.status = TEMP
        # define geo location: x,y coordinate
        self.x = x
        self.y = y


class GeoMap:
    def __init__(self):
        # initialize an empty graph
        self.nodes = list()
        self.edges = defaultdict(list)
        #edges will be like {'a':['b','f','h'], 'b':['c','d'],...}
        self.path_lengths = dict()

    
    def addNode(self,name:str, x:float, y:float):
        new_node = Node(name,x,y)
        self.nodes.append(new_node)


    def getNodes(self):
        return self.nodes


    # define method to get node_index and node_object from name
    def getNodeFromName(self,name):
        for i,n in enumerate(self.nodes):
            if n.name == name:
                return i,n 

    
    def addEdge(self, from_node_name:str,to_node_name:str, path_length:float):
        # we will store index using indexes of node in list
        # from_node_index,from_node = self.getNodeFromName(from_node_name)
        # to_node_index,to_node = self.getNodeFromName(to_node_name)

        self.edges[from_node_name].append(to_node_name)
        self.path_lengths[(from_node_name,to_node_name)] = path_length

    
    def getPathLength(self,from_node:str, to_node:str):
        return self.path_lengths[(from_node,to_node)]


    def getEdges(self):
        return [(start_node,[to_node for to_node in to_nodes]) for start_node,to_nodes in self.edges.items()]
    

    def _getManhattenDistanceBetweenNodes(self, node_a, node_b):
        # we will use manhatten distance between two nodes as our heuristic
        x1, y1 = node_a.x, node_a.y
        x2, y2 = node_b.x, node_b.y
        return abs(x2-x1) + abs(y2-y1)

    
    def _getTemNodeWithMinPathLength(self):
        minimum = INF
        index, ret_node = None, None
        for i, node in enumerate(self.nodes):
            if node.status == TEMP and node.path_length < minimum:
                index, ret_node = i, node
                minimum = node.path_length

        return index, ret_node
    

    def dijkstra(self,source_node_name):
        source_index, source_node = self.getNodeFromName(source_node_name)

        self.nodes[source_index].path_length = 0

        while True:
            min_node_index, min_node = self._getTemNodeWithMinPathLength()
            
            if min_node is None:
                return 

            # relaxe min node selected
            self.nodes[min_node_index].status = PERMANENT

            # update all adjacent nodes
            for v, node in enumerate(self.nodes):
                # check if edge from min_node to current node exist and if it is TEMP
                if node.name in self.edges[min_node.name] and node.status == TEMP:
                    # there is an edge from min-node to curr node:
                    if self.getPathLength(min_node.name, node.name) + min_node.path_length < node.path_length:
                        node.path_length = min_node.path_length + self.getPathLength(min_node.name, node.name)
                        node.parent = min_node


    def aStarSearch(self,source_node_name:str, destination_name:str):
        """
        A* Search is nothing but dijkstra with a modification: here we use heuristic value
        by calculating manhatten distance b/w adjacent nodes and destination nodes.
        which helps avoiding looking for nodes which are in opposite direction.
        In this way it reaches the destination node earlier without looking the opposite nodes
        as it would be costlier to look into those nodes than looking ones which are in right direction.
        """
        
        source_index, source_node = self.getNodeFromName(source_node_name)
        destination_index, destination_node = self.getNodeFromName(destination_name)

        self.nodes[source_index].path_length = 0

        while True:
            min_node_index, min_node = self._getTemNodeWithMinPathLength()
            
            if min_node is None:
                return 

            # relaxe min node selected
            self.nodes[min_node_index].status = PERMANENT

            # update all adjacent nodes
            for v, node in enumerate(self.nodes):
                # check if edge from min_node to current node exist and if it is TEMP
                if node.name in self.edges[min_node.name] and node.status == TEMP:
                    # there is an edge from min-node to curr node:
                    heuristic = self.getPathLength(min_node.name, node.name) + min_node.path_length \
                                + self._getManhattenDistanceBetweenNodes(node, destination_node)

                    if self.getPathLength(min_node.name, node.name) + min_node.path_length < node.path_length:
                        node.path_length = min_node.path_length + self.getPathLength(min_node.name, node.name)
                        node.parent = min_node




    def getRoute(self, source:str, destination:str):
        # self.dijkstra(source)
        self.aStarSearch(source,destination)
        path = []
        shortest_dist = 0

        source_index, source_node = self.getNodeFromName(source)
        dest_index, dest_node = self.getNodeFromName(destination)

        if self.nodes[dest_index].path_length == INF:
            print("not reachable")
            exit(-1)
        else:

            while source_node != dest_node:
                path.append(dest_node.name)
                
                via = dest_node.parent
                shortest_dist += self.getPathLength(via.name,dest_node.name)
                
                dest_node = via

            path.append(source)

            path = reversed(path)
            path = " -> ".join(path)

            return path, shortest_dist


# dg = GeoMap()
# dg.addNode("zero",0,0)
# dg.addNode("one",0,0)
# dg.addNode("two",0,0)
# dg.addNode("three",0,0)
# dg.addNode("four",0,0)
# dg.addNode("five",0,0)
# dg.addNode("six",0,0)
# dg.addNode("seven",0,0)
# dg.addNode("eight",0,0)

# dg.addEdge("zero","three",2)
# dg.addEdge("zero","one",5)
# dg.addEdge("zero","four",8)
# dg.addEdge("one","four",2)
# dg.addEdge("two","one",3)
# dg.addEdge("two","five",4)
# dg.addEdge("three","four",7)
# dg.addEdge("three","six",8)
# dg.addEdge("four","five",9)
# dg.addEdge("four","seven",4)
# dg.addEdge("five","one",6)
# dg.addEdge("six","seven",9)
# dg.addEdge("seven","three",5)
# dg.addEdge("seven","five",3)
# dg.addEdge("seven","eight",5)
# dg.addEdge("eight","five",3)

# source = "zero"
# destination = "seven"
# path, dist = dg.getRoute(source,destination)
# print(f"path from source({source}) to destination({destination}): {path} \n'distance = {dist} units'")

