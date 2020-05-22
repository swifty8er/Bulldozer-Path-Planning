from matplotlib import pyplot as plt
from Vehicle import Vehicle
from BasicGeometry import BasicGeometry
import bezier
from Maps import Maps
import math
import random
import numpy as np
myMap = Maps()

fig1, ax1 = plt.subplots(1, 1)

def drawVehicle(v,axis,map):
    vehicle_pos = (v.x,v.y)
    pos_circle = BasicGeometry.circlePoints(vehicle_pos, map.disk_radius, 25)
    axis.plot(pos_circle[0],pos_circle[1],color='red', linewidth=2)
    r = 0.5
    dx = r*math.cos(math.radians(v.theta))
    dy = r*math.sin(math.radians(v.theta))
    axis.arrow(v.x,v.y,dx,dy,width=0.05)
    plt.draw()
    plt.pause(1)
    plt.show(block=False)


mapNums = [1]
#mapNums = list(range(88,93))+list(range(94,97))
#mapNums = list(range(1,4))
#for mm in range(num,num+10):
#for mm in range(num,num+1):
for mm in mapNums:
    map = myMap.test_maps[mm-1]
    print("Test Map", map.number)
    map.plotStartingMap(ax1)
    plt.draw()
    plt.pause(1)
    plt.show(block=False)
    v1 = Vehicle(random.uniform(map.min_x,map.max_x),random.uniform(map.min_y,map.max_y),random.uniform(0,360))
    v2 = Vehicle(random.uniform(map.min_x,map.max_x),random.uniform(map.min_y,map.max_y),random.uniform(0,360))
    drawVehicle(v1,ax1,map)
    drawVehicle(v2,ax1,map)
    x_points_start = [v1.x,v1.x+math.cos(math.radians(v1.theta))]
    x_points_end = [v2.x-math.cos(math.radians(v2.theta)),v2.x]
    y_points_start = [v1.y,v1.y+math.sin(math.radians(v1.theta))]
    y_points_end = [v2.y-math.sin(math.radians(v2.theta)),v2.y]
    for x in range(10):
        x_points_middle = []
        y_points_middle = []
        for i in range(5):
            x_points_middle.append(random.uniform(map.min_x,map.max_x))
            y_points_middle.append(random.uniform(map.min_y,map.max_y))
        x_points = x_points_start + x_points_middle + x_points_end
        y_points = y_points_start + y_points_middle + y_points_end
        nodes = np.asfortranarray([x_points,y_points])
        curve = bezier.Curve(nodes,degree=8)
        curve.plot(100,'red',ax=ax1)
        print(curve.nodes)
        plt.draw()
        plt.pause(1)
        plt.show(block=False)

