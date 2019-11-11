from matplotlib import path

from Map import Map
from VisibilityGraph import VisibilityGraph
from PushabilityGraph import PushabilityGraph
from BasicGeometry import BasicGeometry


class MapState():
    """description of class"""
    @staticmethod
    def _findNodes(map):
        #set up the nodes in the map
        nodes = []
        for i in range(map._max_y-map._min_y):
            for j in range(map._max_x-map._min_x):
                new_node = [map._min_x+0.5 + j*map._grid_size, map._min_y+0.5 + i*map._grid_size]
                #remove any outbounds points
                boundary = path.Path(map._boundary)
                if (boundary.contains_point(new_node) == True):
                    k = 0
                    inside_obs = False
                    if len(map._obstacles) > 0:
                        while k < len(map._obstacles) and inside_obs == False:
                            obstacle = path.Path(map._obstacles[k])
                            inside_obs = obstacle.contains_point(new_node)
                            k += 1
                        if inside_obs == False:
                            nodes.append(new_node)
                    else:
                        nodes.append(new_node)

        #find the indexes of the nodes that hold a vehicle, disk or goal
        for i in range(len(map._initial_vehicle_pos_xy)):
            node_index = BasicGeometry.isPointInArray(nodes, map._initial_vehicle_pos_xy[i], map._grid_size*0.05)
            if node_index < 0:
                nodes.append(map._initial_vehicle_pos_xy[i])
        for i in range(len(map._initial_disk_pos_xy)):
            node_index = BasicGeometry.isPointInArray(nodes, map._initial_disk_pos_xy[i], map._grid_size*0.05)
            if node_index < 0:
                nodes.append(map._initial_disk_pos_xy[i])
            node_index = BasicGeometry.isPointInArray(nodes, map._goal_pos_xy[i], map._grid_size*0.05)
            if node_index < 0:
                nodes.append(map._goal_pos_xy[i])
        return nodes


    def __init__(self, map):
        self._vehicle_pos = []
        self._disk_pos = []
        self._goal_pos = []
        self._map = map
        self._nodes = self._findNodes(map)

        #find the indexes of the nodes that hold a vehicle, disk or goal
        for i in range(len(self._map._initial_vehicle_pos_xy)):
            node_index = BasicGeometry.isPointInArray(self._nodes, self._map._initial_vehicle_pos_xy[i], self._map._grid_size*0.05)
            if node_index >= 0:
                 self._vehicle_pos.append(node_index)
            else:
                self._nodes.append(self._map._initial_vehicle_pos_xy[i])
                self._vehicle_pos.append(len(self._nodes))
        for i in range(len(self._map._initial_disk_pos_xy)):
            node_index = BasicGeometry.isPointInArray(self._nodes, self._map._initial_disk_pos_xy[i], self._map._grid_size*0.05)
            if node_index >= 0:
                self._disk_pos.append(node_index)
            else:
                self._nodes.append(self._map._initial_disk_pos_xy[i])
                self._disk_pos.append(len(self._nodes))
            node_index = BasicGeometry.isPointInArray(self._nodes, self._map._goal_pos_xy[i], self._map._grid_size*0.05)
            if node_index >= 0:
                self._goal_pos.append(node_index)
            else:
                self._nodes.append(self._map._goal_pos_xy[i])
                self._goal_pos.append(len(self._nodes))

        self._vg = VisibilityGraph(self._nodes, self._map)
        self._pg = PushabilityGraph(self._map, self._vg)
        all_points = self._pg.push_points
        for point in self._pg.dest_points:
            all_points.append(point)
        self._vg.addPushPoints(all_points, self._map)
        
        self._num_of_nodes = len(self._pg.nodes)
        self._num_of_points = len(self._pg.push_points) + len(self._pg.dest_points)
        self._total_num_nodes = self._num_of_nodes + self._num_of_points

        #Create a copy function for the graph classes
        #self._master_vg = self._vg.deepCopy()
        #self._master_pg = self._pg.deepCopy()



    #function from updating the visibility graph and the pushability graph
    def updateGraphs(self):
        #Find the correct VG, PG to use by blocking paths that are inaccesible
        #due to disk placement
        for disk_pos in self._disk_pos:
            for edge in self._vg.edgesBlockedByNode(disk_pos):
                node1 = edge[0]
                node2 = edge[1]
                self._vg.removeEdge(node1, node2)
                if ((node1 < self._num_of_nodes) and (node2 < self._num_of_nodes)):
                    self._pg.removeEdge(node1, node2)

            #Now all connections related to that disk node
            self._vg.removeAllNodesEdges(disk_pos)
        

    #A function to find reachable points from the current map state
    def findPossibleDecisions():
        print("coll")

    @property
    def vg(self):
        return self._vg

    @property
    def pg(self):
        return self._pg

    @property
    def map(self):
        return self._map

    @property
    def num_of_nodes(self):
        return self._num_of_nodes

    @property
    def num_of_points(self):
        return self._num_of_points

    @property
    def total_num_nodes(self):
        return self._total_num_nodes