import numpy as np
from matplotlib import pyplot as plt
from matplotlib.offsetbox import TextArea, DrawingArea, OffsetImage, AnnotationBbox
import matplotlib.image as mpimg
import math
from scipy import ndimage

from BasicGeometry import BasicGeometry
from Vehicle import Vehicle

bulldozer_pic = mpimg.imread('Bulldozer_pic.png')

class Map:
    #A list of attributes that specify the size, shape and starting positions of a Continous Sokoban problem

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
        self._goal_poses_xy = goalpos
        self._initial_vehicle_pos_xy = vehpos
        self._initial_disk_poses_xy = diskpos
        self._pushingHeatmap = self.createPushingHeatmap()

    @property 
    def pushing_heatmap(self):
        return self._pushingHeatmap

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
        return self._goal_poses_xy

    @property
    def initial_vehicle_pos_xy(self):
        return self._initial_vehicle_pos_xy

    @property
    def initial_disk_pos_xy(self):
        return self._initial_disk_poses_xy


    def createPushingHeatmap(self):
        print("Creating pushing heatmap...")
        heatmap = {}
        x = self._min_x
        while x <= self._max_x:
            y = self._min_y
            while y <= self._max_y:
                point = (round(x,1),round(y,1))
                heatmap[point] = []
                for angle in range(0,362,4):
                    second_point = (x+self._disk_radius*3*math.cos(math.radians(angle)),y+self._disk_radius*3*math.sin(math.radians(angle)))
                    line = [point,second_point]
                    if not self.lineIntersectsObstacle(line):
                        heatmap[point].append((angle+180)%360)
                y += 0.1

            x += 0.1
        print("Done!")
        return heatmap


    def lineIntersectsObstacle(self,line):
        edges = self.getMapEdgesAndObstacles()
        for edge in edges:
            if BasicGeometry.doLinesIntersect(line,edge):
                return True
        return False

    def getExtremeState(self):
        return Vehicle(self._max_x,self._max_y,180.0)


    def getCentreState(self):
        return Vehicle((self._max_x-self._min_x)/2.0,(self._max_y-self._min_y)/2.0,0)

    def getMapEdgesAndObstacles(self):
        return self._boundary + self._obstacles


    def plotMapBoundaryObstacles(self,ax,line_width=2):
        ax.axis([self._min_x, self._max_x,  self._min_y, self._max_y])
        x_axis = []
        y_axis = []
        for line in self._boundary:
            p1 = line[0]
            p2 = line[1]
            (x1,y1) = p1
            (x2,y2) = p2
            x_axis.append(x1)
            x_axis.append(x2)
            y_axis.append(y1)
            y_axis.append(y2)
        ax.plot(x_axis, y_axis, 'k-')
        
        for curr_obs in self._obstacles:
            x_axis = []
            y_axis = []
            for point in curr_obs:
                x_axis.append(point[0])
                y_axis.append(point[1])
            ax.plot(x_axis, y_axis, 'k-')
        for goal in self._goal_poses_xy:
            goal_circle = BasicGeometry.circlePoints(goal, self._disk_radius*1.1, 25)
            ax.plot(goal_circle[0],goal_circle[1],color='green', linewidth=line_width)

    def plotStartingMap(self,ax, line_width = 2):
        self.plotMapBoundaryObstacles(ax)   
        for disk_pos in self._initial_disk_poses_xy:
                disk_circle = BasicGeometry.circlePoints(disk_pos, self._disk_radius, 25)
                ax.plot(disk_circle[0],disk_circle[1],color='blue', linewidth=line_width)
       

    def displayMap(self,ax,vehicle_pos,curr_disk_positions, line_width=2):
        ax.axis([self._min_x, self._max_x,  self._min_y, self._max_y])
        x_axis = []
        y_axis = []
        for line in self._boundary:
            p1 = line[0]
            p2 = line[1]
            (x1,y1) = p1
            (x2,y2) = p2
            x_axis.append(x1)
            x_axis.append(x2)
            y_axis.append(y1)
            y_axis.append(y2)
        ax.plot(x_axis, y_axis, 'k-')
        
        for curr_obs in self._obstacles:
            x_axis = []
            y_axis = []
            for point in curr_obs:
                x_axis.append(point[0])
                y_axis.append(point[1])
            ax.plot(x_axis, y_axis, 'k-')

        for goal in self._goal_poses_xy:
            goal_circle = BasicGeometry.circlePoints(goal, self._disk_radius*1.1, 25)
            ax.plot(goal_circle[0],goal_circle[1],color='green', linewidth=line_width)
       
        rotated_pic = ndimage.rotate(bulldozer_pic, (vehicle_pos.theta+180)%360)
        imagebox = OffsetImage(rotated_pic,zoom=0.9)

        ab = AnnotationBbox(imagebox, (vehicle_pos.x, vehicle_pos.y))

        ax.add_artist(ab)

        #robot_pose = [vehicle_pos.x,vehicle_pos.y]
        #pos_circle = BasicGeometry.circlePoints(robot_pose, self._disk_radius, 25)
        #ax.plot(pos_circle[0],pos_circle[1],color='red', linewidth=line_width)
        #r = 0.5
        #dx = r*math.cos(math.radians(vehicle_pos.theta))
        #dy = r*math.sin(math.radians(vehicle_pos.theta))
        #ax.arrow(vehicle_pos.x,vehicle_pos.y,dx,dy,width=0.05,color='red')
        for disk_pos in curr_disk_positions:
            disk_circle = BasicGeometry.circlePoints(disk_pos, self._disk_radius, 25)
            ax.plot(disk_circle[0],disk_circle[1],color='blue', linewidth=line_width)
        return ax


    def plotMap(self, ax, show_plot, vehicle_pos = [], disk_poses = [], line_width = 2):
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
        for goal in self._goal_poses_xy:
            goal_circle = BasicGeometry.circlePoints(goal, self._disk_radius*1.1, 25)
            ax.plot(goal_circle[0],goal_circle[1],color='green', linewidth=line_width)
        if len(vehicle_pos) > 0:
            pos_circle = BasicGeometry.circlePoints(vehicle_pos, self._disk_radius, 25)
            ax.plot(pos_circle[0],pos_circle[1],color='red', linewidth=line_width)
        else:
            for pos in self._initial_vehicle_pos_xy:
                pos_circle = BasicGeometry.circlePoints(pos, self._disk_radius, 25)
                ax.plot(pos_circle[0],pos_circle[1],color='red', linewidth=line_width)
        if len(disk_poses) > 0:
            for disk_pos in disk_poses:
                disk_circle = BasicGeometry.circlePoints(disk_pos, self._disk_radius, 25)
                ax.plot(disk_circle[0],disk_circle[1],color='blue', linewidth=line_width)
        else:

            for disk_pos in self._initial_disk_poses_xy:
                disk_circle = BasicGeometry.circlePoints(disk_pos, self._disk_radius, 25)
                ax.plot(disk_circle[0],disk_circle[1],color='blue', linewidth=line_width)

        if show_plot == True:
            plt.draw()
            plt.pause(0.001)
            plt.show(block=False)
        #ax.set_ylim(0, max(y_axis))
        return ax