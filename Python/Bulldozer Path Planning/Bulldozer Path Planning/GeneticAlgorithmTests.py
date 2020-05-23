from matplotlib import pyplot as plt
from Vehicle import Vehicle
from BasicGeometry import BasicGeometry
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

map = myMap.test_maps[0]
v2 = Vehicle(3,3,60)
v1 = Vehicle(1,1,90)
x_points_start = [v1.x,v1.x+math.cos(math.radians(v1.theta))]
x_points_end = [v2.x-math.cos(math.radians(v2.theta)),v2.x]
y_points_start = [v1.y,v1.y+math.sin(math.radians(v1.theta))]
y_points_end = [v2.y-math.sin(math.radians(v2.theta)),v2.y]
for x in range(1):
    x_points_middle = []
    y_points_middle = []
    for i in range(5):
        x_points_middle.append(random.uniform(map.min_x,map.max_x))
        y_points_middle.append(random.uniform(map.min_y,map.max_y))
    x_points = x_points_start + x_points_middle + x_points_end
    y_points = y_points_start + y_points_middle + y_points_end
    nodes = np.asfortranarray([x_points,y_points])
    curve = bezier.Curve(nodes,degree=8)
    BasicGeometry.getSecondDerivativeOfBezierCurve(curve,0.5)


#s = sym.symbols('s')
#mapNums = [1]
##mapNums = list(range(88,93))+list(range(94,97))
##mapNums = list(range(1,4))
##for mm in range(num,num+10):
##for mm in range(num,num+1):
#for mm in mapNums:
#    map = myMap.test_maps[mm-1]
#    print("Test Map", map.number)
#    map.plotStartingMap(ax1)
#    plt.draw()
#    plt.pause(1)
#    plt.show(block=False)
#    #v1 = Vehicle(random.uniform(map.min_x,map.max_x),random.uniform(map.min_y,map.max_y),random.uniform(0,360))
#    #v2 = Vehicle(random.uniform(map.min_x,map.max_x),random.uniform(map.min_y,map.max_y),random.uniform(0,360))
#    v2 = Vehicle(3,3,60)
#    v1 = Vehicle(1,1,90)
#    x_points_start = [v1.x,v1.x+math.cos(math.radians(v1.theta))]
#    x_points_end = [v2.x-math.cos(math.radians(v2.theta)),v2.x]
#    y_points_start = [v1.y,v1.y+math.sin(math.radians(v1.theta))]
#    y_points_end = [v2.y-math.sin(math.radians(v2.theta)),v2.y]
#    for x in range(100000):
#        x_points_middle = []
#        y_points_middle = []
#        for i in range(5):
#            x_points_middle.append(random.uniform(map.min_x,map.max_x))
#            y_points_middle.append(random.uniform(map.min_y,map.max_y))
#        x_points = x_points_start + x_points_middle + x_points_end
#        y_points = y_points_start + y_points_middle + y_points_end
#        nodes = np.asfortranarray([x_points,y_points])
#        curve = bezier.Curve(nodes,degree=8)
        
#        t = 0.0
#        acceptable = True
#        while t <= 1.0:
#            first_derivative_point_array = curve.evaluate_hodograph(t)
#            first_derivative_point = [i[0] for i in first_derivative_point_array]
#            symbolic = curve.to_symbolic()
#            d1 = sym.diff(symbolic,s)
#            d2 = sym.diff(d1,s)
#            matrix = d2.evalf(subs={s:t})
#            ddx = matrix.row(0)[0]
#            ddy = matrix.row(1)[0]
#            (dx,dy) = first_derivative_point
#            numerator = dx * ddy - ddx * dy
#            denominator = pow(dx*dx + dy*dy, 1.5)
#            kappa = abs(numerator / denominator)
#            roC = 1.0/kappa
#            if roC < 0.4:
#                print("Bad Curve")
#                acceptable = False
#                break
#            t += 0.001

#        if acceptable:
#            print("Acceptable curve found")
#            plt.cla()
#            map.plotStartingMap(ax1)
#            drawVehicle(v1,ax1,map)
#            drawVehicle(v2,ax1,map)
#            curve.plot(100,'red',ax=ax1)
#            plt.draw()
#            plt.pause(10)
#            plt.show(block=False)

