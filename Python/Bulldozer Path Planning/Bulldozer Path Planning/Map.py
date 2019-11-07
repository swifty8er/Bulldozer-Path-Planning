import numpy as np
from matplotlib import path
from matplotlib import pyplot as plt
import math

from BasicGeometry import BasicGeometry

class Map:
    #A list of attributes that specify the size, shape and starting positions of a Continous Sokoban problem

    
    #private method
    def __findNodes(self):
        #set up the nodes in the map
        nodes = []
        for i in range(self._max_y-self._min_y):
            for j in range(self._max_x-self._min_x):
                new_node = [self._min_x+0.5 + j*self._grid_size, self._min_y+0.5 + i*self._grid_size]
                #remove any outbounds points
                boundary = path.Path(self._boundary)
                if (boundary.contains_point(new_node) == True):
                    k = 0
                    inside_obs = False
                    if len(self._obstacles) > 0:
                        while k < len(self._obstacles) and inside_obs == False:
                            obstacle = path.Path(self._obstacles[k])
                            inside_obs = obstacle.contains_point(new_node)
                            k += 1
                        if inside_obs == False:
                            nodes.append(new_node)
                    else:
                        nodes.append(new_node)
        return nodes

    def __init__(self, num, minx, miny, maxx, maxy, gridsize, bound, obs, diskradi, vehradi, goalpos, vehpos, diskpos):
        self._number = num
        self._min_x = minx
        self._max_x = maxx
        self._min_y = miny
        self._max_y = maxy
        self._grid_size = gridsize
        self._boundary = bound
        self._obstacles = obs
        self._disk_radius = diskradi
        self._vehicle_radius = vehradi
        self._goal_pos_xy = goalpos
        self._initial_vehicle_pos_xy = vehpos
        self._initial_disk_pos_xy = diskpos
        self._goal_pos = []
        self._initial_vehicle_pos = []
        self._initial_disk_pos = []
        self._nodes = self.__findNodes()
        #for i in range(len(self._nodes)):
        #    print(self._nodes[i])
        

        #find the indexes of the nodes that hold a vehicle, disk or goal
        for i in range(len(self._initial_vehicle_pos_xy)):
            node_index = BasicGeometry.isPointInArray(self._nodes, self._initial_vehicle_pos_xy[i], self._grid_size*0.05)
            if node_index >= 0:
                 self._initial_vehicle_pos.append(node_index)
            else:
                self._nodes.append(self._initial_vehicle_pos_xy[i])
                self._initial_vehicle_pos.append(len(self._nodes))
        for i in range(len(self._initial_disk_pos_xy)):
            node_index = BasicGeometry.isPointInArray(self._nodes, self._initial_disk_pos_xy[i], self._grid_size*0.05)
            if node_index >= 0:
                self._initial_disk_pos.append(node_index)
            else:
                self._nodes.append(self._initial_disk_pos_xy[i])
                self._initial_disk_pos.append(len(self._nodes))
            node_index = BasicGeometry.isPointInArray(self._nodes, self._goal_pos_xy[i], self._grid_size*0.05)
            if node_index >= 0:
                self._goal_pos.append(node_index)
            else:
                self._nodes.append(self._goal_pos_xy[i])
                self._goal_pos.append(len(self._nodes))

        #print("Map", num)
        #print("Vehicle Position: ", self._initial_vehicle_pos)
        #print("Goal Positions")
        #for j in range(len(self._goal_pos)):
        #    print(self._goal_pos[j])
        #print("Disk Positions")
        #for j in range(len(self._initial_disk_pos)):
        #    print(self._initial_disk_pos[j])

    @property
    def number(self):
        return self._number

    @property
    def min_x(self):
        return self._min_x

    @property
    def max_x(self):
        return self._max_x

    @property
    def min_y(self):
        return self._min_y

    @property
    def max_y(self):
        return self._max_y

    @property
    def grid_size(self):
        return self._grid_size

    @property
    def boundary(self):
        return self._boundary

    @property
    def obstacles(self):
        return self._obstacles

    @property
    def disk_radius(self):
        return self._disk_radius

    @property
    def vehicle_radius(self):
        return self._vehicle_radius

    @property
    def goal_pos_xy(self):
        return self._goal_pos_xy

    @property
    def initial_vehicle_pos_xy(self):
        return self._initial_vehicle_pos_xy

    @property
    def initial_disk_pos_xy(self):
        return self._initial_disk_pos_xy

    @property
    def goal_pos(self):
        return self._goal_pos

    @property
    def initial_vehicle_pos(self):
        return self._initial_vehicle_pos

    @property
    def initial_disk_pos(self):
        return self._initial_disk_pos

    @property
    def nodes(self):
        return self._nodes

    def plotMap(self, line_width, ax):
        ax.axis([self._min_x, self._max_x,  self._min_y, self._max_y])
        x_axis = []
        y_axis = []
        for point in self._boundary:
            x_axis.append(point[0])
            y_axis.append(point[1])
        ax.plot(x_axis, y_axis, 'k-')
        
        for curr_obs in self._obstacles:
            x_axis = []
            y_axis = []
            for point in curr_obs:
                x_axis.append(point[0])
                y_axis.append(point[1])
            ax.plot(x_axis, y_axis, 'k-')
        for goal in self._goal_pos_xy:
            goal_circle = BasicGeometry.circlePoints(goal, self._disk_radius*1.1, 25)
            ax.plot(goal_circle[0],goal_circle[1],color='green', linewidth=line_width)
        for pos in self._initial_vehicle_pos_xy:
            pos_circle = BasicGeometry.circlePoints(pos, self._disk_radius, 25)
            ax.plot(pos_circle[0],pos_circle[1],color='red', linewidth=line_width)
        for disk_pos in self._initial_disk_pos_xy:
            disk_circle = BasicGeometry.circlePoints(disk_pos, self._disk_radius, 25)
            ax.plot(disk_circle[0],disk_circle[1],color='blue', linewidth=line_width)
        plt.draw()
        plt.pause(0.001)
        plt.show(block=False)
        