import queue
import numpy as np
from matplotlib import path
import matplotlib
import matplotlib.pyplot as plt
import imageio
import math

from Map import Map
from VisibilityGraph import VisibilityGraph
from PushabilityGraph import PushabilityGraph
from BasicGeometry import BasicGeometry
from PQNode import PQNode


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
        for i in range(len(map._initial_disk_poses_xy)):
            node_index = BasicGeometry.isPointInArray(nodes, map._initial_disk_poses_xy[i], map._grid_size*0.05)
            if node_index < 0:
                nodes.append(map._initial_disk_poses_xy[i])
            node_index = BasicGeometry.isPointInArray(nodes, map._goal_poses_xy[i], map._grid_size*0.05)
            if node_index < 0:
                nodes.append(map._goal_poses_xy[i])
        return nodes


    def __init__(self, map):
        self._vehicle_pos = []
        self._disk_poses = []
        self._goal_poses = []
        self._map = map
        self._nodes = self._findNodes(map)

        #find the indexes of the nodes that hold a vehicle, disk or goal
        for i in range(len(self._map._initial_vehicle_pos_xy)):
            node_index = BasicGeometry.isPointInArray(self._nodes, self._map._initial_vehicle_pos_xy[i], self._map._grid_size*0.05)
            if node_index >= 0:
                 self._vehicle_pos.append(node_index)
        for i in range(len(self._map._initial_disk_poses_xy)):
            node_index = BasicGeometry.isPointInArray(self._nodes, self._map._initial_disk_poses_xy[i], self._map._grid_size*0.05)
            if node_index >= 0:
                self._disk_poses.append(node_index)
            node_index = BasicGeometry.isPointInArray(self._nodes, self._map._goal_poses_xy[i], self._map._grid_size*0.05)
            if node_index >= 0:
                self._goal_poses.append(node_index)


        self._vg = VisibilityGraph(self._nodes, self._map)
        self._pg = PushabilityGraph(self._map, self._vg)
        all_points = self._pg.push_points.copy()
        for point in self._pg.dest_points:
            all_points.append(point)
        self._vg.addPushPoints(all_points, self._map)
        
        self._num_of_nodes = len(self._pg.nodes)
        self._num_of_points = len(self._pg.push_points) + len(self._pg.dest_points)
        self._total_num_nodes = self._num_of_nodes + self._num_of_points

        self._vg_removed_edges = []
        self._pg_removed_edges = dict()



    #function from updating the visibility graph and the pushability graph
    def updateState(self, node):
        self._vehicle_pos = [node.vehicle_pos]
        self._disk_poses = node.disk_poses.copy()

        self._pg_removed_edges = {disk_pos: [] for disk_pos in self._disk_poses}
        #Find the correct VG, PG to use by blocking paths that are inaccesible
        #due to disk placement
        for disk_pos in self._disk_poses:
            for edge in self._vg.edgesBlockedByNode(disk_pos):
                node1 = edge[0]
                node2 = edge[1]
                removed_edges = self._vg.removeEdge(node1, node2)
                for removed_edge in removed_edges:
                    self._vg_removed_edges.append(removed_edge)
                if ((node1 < self._num_of_nodes) and (node2 < self._num_of_nodes)):
                    removed_edges = self._pg.removeEdge(node1, node2)
                    for removed_edge in removed_edges:
                        self._pg_removed_edges[disk_pos].append(removed_edge)

            #Now all connections related to that disk node
            removed_edges = self._vg.removeAllNodesEdges(disk_pos)
            for removed_edge in removed_edges:
                self._vg_removed_edges.append(removed_edge)

    #function from updating the visibility graph and the pushability graph
    def resetGraphs(self):

        for vg_removed_edge in self._vg_removed_edges:
            node1 = vg_removed_edge[0]
            node2 = vg_removed_edge[1]
            self._vg.addEdge(node1, node2, True)

        for disk_pos in self._pg_removed_edges.keys():
            for pg_removed_edge in self._pg_removed_edges[disk_pos]:
                node1 =pg_removed_edge[0]
                node2 = pg_removed_edge[1]
                self._pg.addEdge(node1, node2, True)

        self._vg_removed_edges = []
        self._pg_removed_edges = dict()

    def isFinishState(self):
        i = 0
        finished = True
        sorted_disk_poses = sorted(self._disk_poses)
        sorted_goal_poses = sorted(self._goal_poses)
        while ((i < len(self._disk_poses)) and (finished == True)):
            if (sorted_disk_poses[i] != sorted_goal_poses[i]):
                    finished = False
            i += 1

        return finished

    def getCurrentState(self):
        min_cf= [math.inf]*len(self._disk_poses)
        for i in range(len(self._disk_poses)):
            for j in range(len(self._goal_poses)):
                disk_pos = self._nodes[self._disk_poses[i]]
                goal_pos = self._nodes[self._goal_poses[j]]
                curr_cf = BasicGeometry.ptDist(disk_pos, goal_pos)
                if (curr_cf < min_cf[i]):
                    min_cf[i] = curr_cf
        cost = sum(min_cf)
        node = PQNode(self._vehicle_pos[0], -1, -1, self._disk_poses, -1, 0, [], [], cost)

        return node


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

    @property
    def nodes(self):
        return self._nodes

    def getNodeOrPoint(self, index):
        coordinate = [math.inf, math.inf]
        if index >= 0 and index < self._num_of_nodes:
            coordinate = self._nodes[index]
        elif index >= self._num_of_nodes and index < self._num_of_nodes + len(self._pg.push_points):
            coordinate = self._pg.push_points[index-self._num_of_nodes]
        elif index >= self._num_of_nodes + len(self._pg.push_points) and index < self._total_num_nodes:
            coordinate = self._pg.dest_points[index-(self._num_of_nodes + len(self._pg.push_points))]

        return coordinate

    def __findPushPoints(self, disk_from, disks_to):
        valid_push_points = []
        if (len(disks_to) > 0):
            k = 0
            while (k < len(self._pg._push_points)):
                if(self._pg._push_points[k][2] == disk_from):
                    kk = 0
                    while ((kk < len(disks_to)) and (len(valid_push_points) < len(disks_to))):
                        if (self._pg._dest_points[k][2] == disks_to[kk]):
                            valid_push_points.append(k)
                        kk += 1
                k += 1

        return valid_push_points


    def __possibleDiskDest(self, doi, filter_disks, remove_blockage_disks):
        doi_nodes = list(self._pg.connectingNodes(doi))
        for filter_disk in filter_disks:
            if filter_disk in doi_nodes:
                ind = doi_nodes.index(filter_disk)
                del doi_nodes[ind] #filter out that disk
        for remove_blockage_disk in remove_blockage_disks:
            for edge in self._pg_removed_edges[remove_blockage_disk]:
                node1 = edge[0]
                node2 = edge[1]
                if ((node1 < self._num_of_nodes) and (node2 == doi)):
                    if not (node1 in doi_nodes):
                        doi_nodes.append(node1)
                elif((node1 == doi) and (node2 < self._num_of_nodes)):
                    if not (node2 in doi_nodes):
                        doi_nodes.append(node2)

        return doi_nodes


    def __findDiskThatNeedToMove(self, filter_disks, disk_of_interest, nodes_to):
        #check to see if there are any pushable connections
        #from the disks of interest node
        disk_req_move = [list() for _ in range(len(nodes_to))]
        for filter_disk in filter_disks:
            if (filter_disk in nodes_to):
                found_index = nodes_to.index(filter_disk)
                disk_req_move[found_index].append(filter_disk)
            u = 0
            while (u < len(self._vg.edgesBlockedByNode(filter_disk))):
                ii = self._vg.edgesBlockedByNode(filter_disk)[u][0]
                jj = self._vg.edgesBlockedByNode(filter_disk)[u][1]
                if ((ii == disk_of_interest) and (jj < self._num_of_nodes)):
                    if (jj in nodes_to):
                        found_index = nodes_to.index(jj)
                        disk_req_move[found_index].append(filter_disk)
                elif ((jj == disk_of_interest) and (ii < self._num_of_nodes)):
                    if (ii in nodes_to):
                        found_index = nodes_to.index(ii)
                        disk_req_move[found_index].append(filter_disk)
                u += 1

        return disk_req_move

    def _getStateFromDecision(self, push_decision, decision_path, vehicle_path, disks_path):
        vehicle_dest_pt = push_decision + self.num_of_nodes + int(self.num_of_points/2)
        vehicle_pos = self._pg.push_points[push_decision][2]
        disk_num = self._disk_poses.index(self._pg.push_points[push_decision][2])
        new_disk_poses = self._disk_poses.copy()
        new_disk_poses[disk_num] = self._pg.dest_points[push_decision][2]
        #dest_pos = self._nodes[new_disk_pose[disk_num]]
        #start_pos = self._nodes[self._disk_poses[disk_num]]
        #travelled = BasicGeometry.ptDist(dest_pos, start_pos) #+curr_node.travelled
        min_cf= [math.inf]*len(new_disk_poses)
        for i in range(len(new_disk_poses)):
            for j in range(len(self._goal_poses)):
                disk_pos = self._nodes[new_disk_poses[i]]
                goal_pos = self._nodes[self._goal_poses[j]]
                curr_cf = BasicGeometry.ptDist(disk_pos, goal_pos)
                if (curr_cf < min_cf[i]):
                    min_cf[i] = curr_cf
        cost = sum(min_cf)
        node = PQNode(vehicle_pos, vehicle_dest_pt, decision_path, new_disk_poses, disk_num, 0, vehicle_path, disks_path, cost)

        return node


    def addVehiclePositionsToStack(self):
        stack = queue.LifoQueue()
        for r in range(len(self._vehicle_pos)):
            curr_node = {
                "pos" : self._vehicle_pos[r],
                "path" : [self._vehicle_pos[r]]
                }
            stack.put(curr_node)
       
        return stack

    def getVisitedNodesForDFS(self):
        visited_nodes = [False]*self._total_num_nodes
        visited_nodes[self._vehicle_pos[0]] = True
        for disk_pos in self._disk_poses:
            visited_nodes[disk_pos] = True
        return visited_nodes

    def getIndexRangesForDFS(self):
        #find range necessary indices to check for possible push points
        #This works because the push points are list in order of disk poses
        index_ranges = [[-1,-1] for _ in range(len(self._disk_poses))]
        for p in range(len(self._disk_poses)):
            k = 0
            finished = False
            while k < len(self._pg._push_points) and finished == False:
                if(self._pg._push_points[k][2] == self._disk_poses[p]):
                    if (index_ranges[p][0] == -1):
                        #first index in the push point list that has a push point relevant to the current disk pos
                        index_ranges[p][0] = k
                        index_ranges[p][1] = k
                    else:
                        #last index in the push point list that has a push point relevant to the current disk pos
                        index_ranges[p][1] = k

                elif index_ranges[p][0] != -1:
                    finished = True
                k += 1

        #filter out any disk poses that don't have any push points
        index_ranges = [index_range for index_range in index_ranges if index_range[0] != -1]
        return index_ranges


    def getMaxNumPointsDFS(self,index_ranges):
        max_num_points = 0
        if (len(index_ranges) > 0):
            for index_range in index_ranges:
                max_num_points += index_range[1] - index_range[0] + 1
        return max_num_points

    def addUnvisitedNodesToStackDFS(self,stack,curr_node,new_positions,visited_nodes):
        #add unvisited nodes to the stack
        t = 0
        while t < len(new_positions) and new_positions[t] < self._num_of_nodes:
            new_pos = new_positions[t]
            if(visited_nodes[new_pos] == False):
                new_node = {
                    "pos" : new_pos,
                    "path" : curr_node["path"].copy()
                }
                new_node["path"].append(new_pos)
                stack.put(new_node)
                visited_nodes[new_pos] = True
            t += 1

    def pushIntoDeadEnd(self,disk_pushed_from,disk_pushed_to):
        #check if being pushed into a dead-end
        #get a list of the disks that aren't being moved
        other_disks = [disk_pos for disk_pos in self._disk_poses if disk_pos != disk_pushed_from]
        #check to see if there are any pushable connections
        #from the destination node
        #pushable connections have been found
        #find push points
        ori_disk_pushed_from = disk_pushed_from
        disk_pushed_from = disk_pushed_to
        disks_pushed_to = self.__possibleDiskDest(disk_pushed_from, other_disks, [ori_disk_pushed_from])
        valid_push_points = self.__findPushPoints(disk_pushed_from, disks_pushed_to)
        #find if these push points are reachable with
        #new disk placement
        u = 0
        #search through all push points to find one that is valid
        while u < len(valid_push_points):
            valid_push_point = valid_push_points[u]
            v = 0
            disk_is_blocking = False
            #check whether there are any disks that are
            #too close and blocking the push points
            while ((v < len(other_disks)) and (disk_is_blocking == False)):
                other_disk = other_disks[v]
                curr_dist = BasicGeometry.ptDist(self._nodes[other_disk], self._pg.push_points[valid_push_point])
                if ((2*self._map._disk_radius-curr_dist) > np.finfo(np.float32).eps):
                    disk_is_blocking = True
                            
                v += 1
                        
            if (disk_is_blocking == False):
                return False #not a dead end
                        
            u += 1
        return True #is a dead end

    def getDisksThatAreRequiredToMove(self,disk_pushed_from,disk_pushed_to):
        #find the push points that were neglected
        #from the CellPG
        nodes_pushed_to = self.__possibleDiskDest(disk_pushed_from, [], other_disks) #every single node connected to a certain node
        valid_push_points = self.__findPushPoints(disk_pushed_from, nodes_pushed_to)
        #list of disks that are required to move to make a certain push point valid
        disk_req_move = self.__findDiskThatNeedToMove(other_disks, disk_pushed_from, nodes_pushed_to)
        #find if these push points are reachable with
        #new disk placement
        for u in range(len(valid_push_points)):
            for other_disk in other_disks:
                curr_dist = BasicGeometry.ptDist(self._nodes[other_disk], self._pg.push_points[valid_push_points[u]])
                if ((2*self._map._disk_radius-curr_dist) > np.finfo(np.float32).eps):
                    if (not (other_disk in disk_req_move[u])):
                        disk_req_move[u].append(other_disk)
        return disk_req_move
                                    
                                
                            

    def addAllAccessiblePushPointsDFS(self, decisions, index_ranges, visited_nodes, max_num_points):
        #add all accessible push points
        for index_range in index_ranges:
            for pp_index in range(index_range[0], index_range[1]+1):
                disk_pushed_from = self._pg.push_points[pp_index][2]
                disk_pushed_to = self._pg.dest_points[pp_index][2]
                if ((self._vg.validEdge(curr_node["pos"], pp_index + self._num_of_nodes) == True) and
                        (visited_nodes[pp_index + self._num_of_nodes] == False) and
                        (not (disk_pushed_to in self._disk_poses)) and
                        (self._pg.validEdge(disk_pushed_from,disk_pushed_to) == True)):
                    if (not (disk_pushed_to in self._goal_poses)):

                        if not self.pushIntoDeadEnd(disk_pushed_from,disk_pushed_to):
                            point_path = curr_node["path"].copy()
                            point_path.append(pp_index+self._num_of_nodes)
                            decisions.append(self._getStateFromDecision(pp_index, point_path, vehicle_path, disks_path))
                            visited_nodes[pp_index+self._num_of_nodes] = True
                    
                        else:
                            disk_req_move = self.getDisksThatAreRequiredToMove(disk_pushed_from,disk_pushed_to)

                            
                        
                            #order all push points to be investigated
                            #by the number of disks that they require
                            #to be moved from least to most
                            sorted(disk_req_move,key=lambda disks:len(disks))
                            
                            #Now take the first possible push point and
                            #check that the disks that are in the way
                            #can be moved
                            i = 0
                            valid = False
                            disk_req_move_conn = dict()
                            filter_disks = other_disks.copy()
                            filter_disks.append(disk_pushed_from)
                            for other_disk in other_disks:
                                disk_req_move_conn[other_disk] = self.__possibleDiskDest(other_disk, filter_disks, other_disks)
                            #search through all possible push points to
                            #see if any are valid
                            while ((i < len(disk_req_move)) and (valid == False)):
                                u = 0
                                valid_pp = True
                                #search through all the disks that need
                                #moving to see if any have valid push
                                #decisions
                                while ((u < len(disk_req_move[i])) and (valid_pp == True)):
                                    if len(disk_req_move_conn[disk_req_move[i][u]]) > 0:
                                        v = 0
                                        valid_disk_move = False
                                        #find the push points for the
                                        #disk that needs moving
                                        pps = self.__findPushPoints(disk_req_move[i][u], disk_req_move_conn[disk_req_move[i][u]])
                                        #check that at least on push
                                        #point isn't too close to the
                                        #new disk location
                                        while ((v < len(pps)) and (valid_disk_move == False)):
                                            curr_pp = self._pg.push_points[pps[v]]
                                            curr_dist = BasicGeometry.ptDist(self._nodes[disk_pushed_from], curr_pp)
                                            if ((2*self._map._disk_radius-curr_dist) <= np.finfo(np.float32).eps):
                                                valid_disk_move = True
                                        
                                            v += 1
                                    
                                        if (valid_disk_move == False):
                                            valid_pp = False
                                    
                                    else:
                                        valid_pp = False
                                
                                    u += 1
                            
                                if (valid_pp == True):
                                    valid = True
                            
                                i += 1
                        
                            if (valid == True):
                                point_path = curr_node["path"].copy()
                                point_path.append(pp_index+self._num_of_nodes)
                                decisions.append(self._getStateFromDecision(pp_index, point_path, vehicle_path, disks_path))
                                visited_nodes[pp_index+self._num_of_nodes] = True
                        
                    
                    else:
                        point_path = curr_node["path"].copy()
                        point_path.append(pp_index+self._num_of_nodes)
                        decisions.append(self._getStateFromDecision(pp_index, point_path, vehicle_path, disks_path))
                        visited_nodes[pp_index+self._num_of_nodes] = True

    def findReachablePushPoints(self, vehicle_path, disks_path):
        decisions = []
        stack = self.addVehiclePositionsToStack()
        visited_nodes = self.getVisitedNodesForDFS()
        index_ranges = self.getIndexRangesForDFS()
        max_num_points = self.getMaxNumPointsDFS(index_ranges)

        #find all possible push points by a depth first search
        while ((stack.empty() == False) and (len(decisions) < max_num_points)):
            curr_node = stack.get()
            new_positions = sorted(list(self._vg.connectingNodes(curr_node["pos"])))

            self.addUnvisitedNodesToStackDFS(stack,curr_node,new_positions,visited_nodes)
            
            self.addAllAccessiblePushPointsDFS(decisions,index_ranges,visited_nodes,max_num_points)
        

        return decisions

    def plotSolution(self, vehicle_path, disks_path):

        solution_images = []
        all_nodes = self._pg.nodes + self._pg.push_points + self._pg.dest_points
        curr_disk_index = [0]*len(disks_path)
        curr_disk_pos = []
        push_point_rng = [self.num_of_nodes, self.num_of_nodes + int(self.num_of_points/2) - 1]
        dest_points_rng = [self.num_of_nodes + int(self.num_of_points/2), self.total_num_nodes - 1]
        for curr_disk_path in disks_path:
            curr_disk_pos.append(all_nodes[curr_disk_path[0]])
        for i in range(1,len(vehicle_path)):
            fig, ax = plt.subplots(1, 1)
            #get and plot vehicle position
            curr_pos = all_nodes[vehicle_path[i]]
            #find transition in vehicle path when vehicle pushes a disk
            if ((vehicle_path[i-1] <= push_point_rng[1]) and (vehicle_path[i-1] >= push_point_rng[0]) and 
                (vehicle_path[i] <= dest_points_rng[1]) and (vehicle_path[i] >= dest_points_rng[0])):
                #find closest disk to the vehicle
                closest_disk = BasicGeometry.findClosestPoint(all_nodes[vehicle_path[i-1]], curr_disk_pos)
                #store the next move to be made by the disks if valid
                curr_disk_path = disks_path[closest_disk]
                if (curr_disk_index[closest_disk] + 1 <len(curr_disk_path)):
                    curr_disk_index[closest_disk] = curr_disk_index[closest_disk] + 1
                    curr_disk_pos[closest_disk] = all_nodes[curr_disk_path[curr_disk_index[closest_disk]]]
            
            #plot new game state
            ax = self.map.plotMap(ax, False, curr_pos, curr_disk_pos)
            
            # Used to return the plot as an image array
            fig.canvas.draw()       # draw the canvas, cache the renderer
            image = np.frombuffer(fig.canvas.tostring_rgb(), dtype='uint8')
            image  = image.reshape((1920,640,3))
            solution_images.append(image)
            plt.close(fig)
        
        plt.close("all")
        return solution_images