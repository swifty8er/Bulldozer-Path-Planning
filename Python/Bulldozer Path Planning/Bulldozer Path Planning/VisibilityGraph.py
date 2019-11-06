import Map
import numpy as np
import math
from matplotlib import pyplot as plt


class VisibilityGraph:
    """description of class"""

    @staticmethod
    def __doLinesIntersect(line_1, line_2):
        x1 = line_1[0][0]
        x2 = line_1[1][0]
        x3 = line_2[0][0]
        x4 = line_2[1][0]
        y1 = line_1[0][1]
        y2 = line_1[1][1]
        y3 = line_2[0][1]
        y4 = line_2[1][1]
        diff_x = x1 - x3
        diff_y = y1 - y3
        denominator = (x4 - x3)*(y1 - y2) - (x1 - x2)*(y4 - y3)
        if (denominator != 0):
            t1 = ((y3 - y4)*diff_x + (x4 - x3)*diff_y)/denominator
            t2 = ((y1 - y2)*diff_x + (x2 - x1)*diff_y)/denominator
            if ((t1 <= 1) and (t1 >= 0) and (t2 <= 1) and (t2 >= 0)):
                intersect = True
            else:
                intersect = False
        else:
            intersect = False

        return intersect

    @staticmethod
    #find the perpendicuar distance between a point and a line if within the
    #bounds of the line (line is not extended) otherwise find the distance to the closest point
    def __point2LineDist(line, point):
        x1 = line[0][0]
        x2 = line[1][0]
        y1 = line[0][1]
        y2 = line[1][1]
        if (abs(y1-y2) < np.finfo(np.float32).eps):
            inbounds = ((point[0] <= max([x1,x2])) and (point[0] >= min([x1,x2])))
        else:
            perp_gradient = -(x1-x2)/(y1-y2)
            perp_intercept_1 = y1 - perp_gradient*x1
            perp_intercept_2 = y2 - perp_gradient*x2
            perp_lines_y = [perp_gradient*point[0] + perp_intercept_1]
            perp_lines_y.append(perp_gradient*point[0] + perp_intercept_2)
            inbounds = ((point[1] <= max(perp_lines_y)) and (point[1] >= min(perp_lines_y)))

        if (inbounds):
            a = np.array([x1,y1,0]) - np.array([x2,y2,0])
            b = np.array([point[0],point[1],0]) - np.array([x2,y2,0])
            dist = np.linalg.norm(np.cross(a,b)) / np.linalg.norm(a)
        else:
            distances = [math.sqrt((point[0]-x1)**2+(point[1]-y1)**2)]
            distances.append(math.sqrt((point[0]-x2)**2+(point[1]-y2)**2))
            dist = min(distances)

        return dist

    def __init__(self, map):
        #improvements possible here: http://cs.smith.edu/~istreinu/Teaching/Courses/274/Spring98/Projects/Philip/fp/algVisibility.htm
        
        #initialise visibility graph
        self._vis_graph = dict()
        for r in range(len(map.nodes)):
            self._vis_graph[r] = dict()
        self._nodes_block_which_vertices = dict()
        for p in range(len(map.nodes)):
            self._nodes_block_which_vertices[p] = []
        close_nodes = []
        #Create a list of boundary and obstacle vertices
        vertices = []
        for obs in map.obstacles:
            for pt_index in range(len(obs)-1):
                vertices.append([obs[pt_index], obs[pt_index+1]])
        for bd_index in range(len(map.boundary)-1):
            vertices.append([map.boundary[bd_index], map.boundary[bd_index+1]])
        for i in range(len(map.nodes)-1):
            for j in range(i+1,len(map.nodes)):
                traverse_line = [map.nodes[i],map.nodes[j]]
                intersect = False
                
                #check every vertex against the connection between two nodes
                k = 0
                while (k < len(vertices)) and (intersect == False):
                    #check if any line of an obstacle intersects the line
                    #between two nodes
                    curr_vertex = vertices[k]
                    intersect = self.__doLinesIntersect(traverse_line, curr_vertex)

                    #Check to see if any vertexes too close
                    if (intersect == False):
                        #Find the minimum distance between two lines
                        curr_dist = [self.__point2LineDist(traverse_line, curr_vertex[0])]
                        curr_dist.append(self.__point2LineDist(traverse_line, curr_vertex[1]))
                        curr_dist.append(self.__point2LineDist(curr_vertex, traverse_line[0]))
                        curr_dist.append(self.__point2LineDist(curr_vertex, traverse_line[1]))
                        if (map.disk_radius - min(curr_dist) > np.finfo(np.float32).eps):
                            intersect = True
                    k += 1

                #since all obstalces are clear now check to see which nodes are too close to the line
                if (intersect == False):
                    for s in range(len(map.nodes)):
                        #if doesn't equal to the start or destination of the traverse line
                        if ((s != j) and (s != i)):
                            curr_dist = self.__point2LineDist(traverse_line, map.nodes[s])
                            #determine if curr dist is less than tolerance
                            if ((2*map.disk_radius-curr_dist) > np.finfo(np.float32).eps):
                                close_nodes.append(s)


                
                if (intersect == False):
                    self._vis_graph[i][j] = True
                    self._vis_graph[j][i] = True
                    if (len(close_nodes) > 0):
                        for close_node in close_nodes:
                            self._nodes_block_which_vertices[close_node].append([i,j])

    def plotGraph(self, nodes, line_size, ax):
        for i in range(len(nodes)):
            for j in range(len(nodes)):
                if (j in self._vis_graph[i].keys())  == True:
                    ax.plot([nodes[i][0],nodes[j][0]],[nodes[i][1],nodes[j][1]], color='blue', linewidth=line_size)
        plt.draw()
        plt.pause(0.001)
        plt.show(block=False)