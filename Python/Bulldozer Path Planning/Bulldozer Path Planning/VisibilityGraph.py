import numpy as np
from matplotlib import pyplot as plt

from Map import Map
from BasicGeometry import BasicGeometry
from Graph import Graph

class VisibilityGraph(Graph):
    #A graph structure where connection represents an edge a vehicle can travel along

    
    def __init__(self, map):
        #improvements possible here: http://cs.smith.edu/~istreinu/Teaching/Courses/274/Spring98/Projects/Philip/fp/algVisibility.htm
        
        #initialise visibility graph
        self._graph = dict()
        self._nodes = map.nodes
        for r in range(len(self._nodes)):
            self._graph[r] = dict()
        self._nodes_block_which_edges = dict()
        for p in range(len(self._nodes)):
            self._nodes_block_which_edges[p] = []
        close_nodes = []
        #Create a list of boundary and obstacle edges
        edges = []
        for obs in map.obstacles:
            for pt_index in range(len(obs)-1):
                edges.append([obs[pt_index], obs[pt_index+1]])
        for bd_index in range(len(map.boundary)-1):
            edges.append([map.boundary[bd_index], map.boundary[bd_index+1]])
        for i in range(len(self._nodes)-1):
            for j in range(i+1,len(self._nodes)):
                traverse_line = [self._nodes[i],self._nodes[j]]
                intersect = False
                
                #check every edge against the connection between two nodes
                k = 0
                while (k < len(edges)) and (intersect == False):
                    #check if any line of an obstacle intersects the line
                    #between two nodes
                    curr_edge = edges[k]
                    intersect = BasicGeometry.doLinesIntersect(traverse_line, curr_edge)

                    #Check to see if any edges are too close
                    if (intersect == False):
                        #Find the minimum distance between two lines
                        curr_dist = [BasicGeometry.point2LineDist(traverse_line, curr_edge[0])]
                        curr_dist.append(BasicGeometry.point2LineDist(traverse_line, curr_edge[1]))
                        curr_dist.append(BasicGeometry.point2LineDist(curr_edge, traverse_line[0]))
                        curr_dist.append(BasicGeometry.point2LineDist(curr_edge, traverse_line[1]))
                        if (map.disk_radius - min(curr_dist) > np.finfo(np.float32).eps):
                            intersect = True
                    k += 1

                #since all obstalces are clear now check to see which nodes are too close to the line
                if (intersect == False):
                    for s in range(len(self._nodes)):
                        #if doesn't equal to the start or destination of the traverse line
                        if ((s != j) and (s != i)):
                            curr_dist = BasicGeometry.point2LineDist(traverse_line, self._nodes[s])
                            #determine if curr dist is less than tolerance
                            if ((2*map.disk_radius-curr_dist) > np.finfo(np.float32).eps):
                                close_nodes.append(s)


                
                if (intersect == False):
                    self._graph[i][j] = True
                    self._graph[j][i] = True
                    if (len(close_nodes) > 0):
                        for close_node in close_nodes:
                            self._nodes_block_which_edges[close_node].append([i,j])



    def addPushPoints(self, new_nodes, map):
        #add structures for the new nodes
        for r in range(len(self._nodes), len(new_nodes) + len(self._nodes)):
            self._graph[r] = dict()
        close_nodes = []
        #Create a list of boundary and obstacle edges
        edges = []
        for obs in map.obstacles:
            for pt_index in range(len(obs)-1):
                edges.append([obs[pt_index], obs[pt_index+1]])
        for bd_index in range(len(map.boundary)-1):
            edges.append([map.boundary[bd_index], map.boundary[bd_index+1]])
        for i in range(len(new_nodes)):
            for j in range(len(self._nodes)): #add +i  if you want connections between different push and dest points
                intersect = False
                #find the traverse line between the push point and the node or other
                #push point
                traverse_line = [new_nodes[i]]
                start = 0
                if (j < len(self._nodes)):
                    traverse_line.append(self._nodes[j])
                    dest = j
                else:
                    traverse_line.append(new_nodes[j-len(self._nodes)])
                    dest = 0
                
                #check that push points from the same node cannot connect to each other
                if (((j < len(self._nodes)) or (new_nodes[i][2] != new_nodes[j-len(self._nodes)][2])) and (new_nodes[i][2] != j)):
                    #check every obstacle against the connection between two nodes
                    k = 0
                    while (k < len(edges)) and (intersect == False):
                        #check if any line of an obstacle intersects the traverse line
                        #between two nodes
                        curr_edge = edges[k]
                        intersect = BasicGeometry.doLinesIntersect(traverse_line, curr_edge)

                        #Check to see if any edges are too close
                        if (intersect == False):
                            #Find the minimum distance between two lines
                            curr_dist = [BasicGeometry.point2LineDist(traverse_line, curr_edge[0])]
                            curr_dist.append(BasicGeometry.point2LineDist(traverse_line, curr_edge[1]))
                            curr_dist.append(BasicGeometry.point2LineDist(curr_edge, traverse_line[0]))
                            curr_dist.append(BasicGeometry.point2LineDist(curr_edge, traverse_line[1]))
                            if (map.disk_radius - min(curr_dist) > np.finfo(np.float32).eps):
                                intersect = True
                        
                        k += 1

                    #since all obstalces are clear now check to see which nodes are too close to the line
                    if (intersect == False):
                        for s in range(len(self._nodes)):
                            #if doesn't equal to the start or destination of the traverse line
                            if ((s != j) and (s != i)):
                                curr_dist = BasicGeometry.point2LineDist(traverse_line, self._nodes[s])
                                #determine if curr dist is less than tolerance
                                if ((2*map.disk_radius-curr_dist) > np.finfo(np.float32).eps):
                                    close_nodes.append(s)
                    
                else:
                    intersect = True
                
                if (intersect == False):
                    self._graph[i+len(self._nodes)][j] = True
                    self._graph[j][i+len(self._nodes)] = True
                    if (len(close_nodes) > 0):
                        for close_node in close_nodes:
                            self._nodes_block_which_edges[close_node].append([i+len(self._nodes),j])

    @property
    def nodes_block_which_edges(self):
        return self._nodes_block_which_edges