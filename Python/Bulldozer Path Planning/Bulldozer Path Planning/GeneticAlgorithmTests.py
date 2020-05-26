from matplotlib import pyplot as plt
from Vehicle import Vehicle
from BasicGeometry import BasicGeometry
from GeneticAlgorithm import GeneticAlgorithm
import bezier
from Maps import Maps
import math
import random
import numpy as np
import sympy as sym
from scipy import special as sp

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




mapNums = [4]
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
    curr_disk_pos = [[1,3.5],[2.75,2.25]]
    #v1 = Vehicle(random.uniform(map.min_x,map.max_x),random.uniform(map.min_y,map.max_y),random.uniform(0,360))
    #v2 = Vehicle(random.uniform(map.min_x,map.max_x),random.uniform(map.min_y,map.max_y),random.uniform(0,360))
    v2 = Vehicle(1.85,2.25,360)
    v1 = Vehicle(3.21,1.38,332.53)
    GA = GeneticAlgorithm(map,v1,v2,30,0.95,0.01,10000,0,curr_disk_pos,ax1)
    for c in GA.population:
        c.plot(100,'red',ax=ax1)