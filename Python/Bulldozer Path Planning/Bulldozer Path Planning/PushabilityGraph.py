import queue
import numpy as np
import math
from matplotlib import pyplot as plt

from Map import Map
from VisibilityGraph import VisibilityGraph
from BasicGeometry import BasicGeometry
from Graph import Graph

class PushabilityGraph(Graph):
    #A graph structure where each connection represents an edge a vehicle can push a disk across
    
    @staticmethod
    def __pushPoint(start, dest, map, start_id, dest_id):
        pushable = True
        push_point = []
        #Create a list of boundary and obstacle edges
        edges = []
        for obs in map.obstacles:
            for pt_index in range(len(obs)-1):
                edges.append([obs[pt_index], obs[pt_index+1]])
        for bd_index in range(len(map.boundary)-1):
            edges.append([map.boundary[bd_index], map.boundary[bd_index+1]])
        #initialise vehicle circle
        angle = math.atan2(start[1] - dest[1], start[0] - dest[0])
        centre = [start[0] + (map.disk_radius+map.vehicle_radius)*math.cos(angle)]
        centre.append(start[1] + (map.disk_radius+map.vehicle_radius)*math.sin(angle))
        i = 0
        while ((i < len(edges)) and (pushable == True)):
            curr_edge = edges[i]
            pushable = not BasicGeometry.doesCircleIntersectLine(centre, map.vehicle_radius, curr_edge)
            i += 1
        if (pushable == True):
            push_point = [start[0] + (map.disk_radius+map.vehicle_radius)*math.cos(angle)]
            push_point.append(start[1] + (map.disk_radius+map.vehicle_radius)*math.sin(angle))
            push_point.append(start_id)
            push_point.append(dest[0] + (map.disk_radius+map.vehicle_radius)*math.cos(angle))
            push_point.append(dest[1] + (map.disk_radius+map.vehicle_radius)*math.sin(angle))
            push_point.append(dest_id)

        return push_point




    def __init__(self, map, vg):
        self._graph = dict()
        self._nodes = map.nodes
        self._push_points = []
        self._dest_points = []
        all_push_dest_points = []
        for r in range(len(self._nodes)):
            self._graph[r] = dict()
        added = np.zeros(len(self._nodes))
        q = queue.Queue()
        for goal in map.goal_pos:
            q.put(goal)
        while (q.empty() == False):
            curr_node = q.get()
            for i in vg.nodes_with_connections():
                #checks if node isn't the same and that they can see each other
                if ((curr_node in vg.connecting_nodes(i)) and (curr_node != i)):
                    #finds and adds the push point if it exists
                    push_dest_point = self.__pushPoint(self._nodes[i], self._nodes[curr_node], map, i, curr_node)
                    if (len(push_dest_point) > 0):
                        self._graph[i][curr_node] = vg.edge_weight(i, curr_node)
                        #add to queue if not in there already
                        if (added[i] == 0):
                            q.put(i)
                            added[i] = 1
                        if (BasicGeometry.isPointInArray(all_push_dest_points, push_dest_point, np.finfo(np.float32).eps) < 0):
                            all_push_dest_points.append(push_dest_point)
        all_push_dest_points = sorted(all_push_dest_points,key=lambda l:l[5])
        all_push_dest_points = sorted(all_push_dest_points,key=lambda l:l[2])
        for p_d_point in all_push_dest_points:
            self._push_points.append([p_d_point[0], p_d_point[1], p_d_point[2]])
            self._dest_points.append([p_d_point[3], p_d_point[4], p_d_point[5]])
