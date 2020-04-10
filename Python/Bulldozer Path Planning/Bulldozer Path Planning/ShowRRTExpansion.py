import turtle
import math
from RRT import RRT
from RRT import Status
from Vehicle import Vehicle
from Maps import Maps
from BasicGeometry import BasicGeometry
SCALING = 100.0
OFFSET = 300.0
NUM_NODES = 1000
MyMaps = Maps()
map = MyMaps.test_maps[0]
StartVehiclePos = Vehicle(1.5,2.5,270)
ControlsList = [
    (0.4,45,"FL"),
    (0.4826,37.3,"FL"),
    (0.593,30.37,"FL"),
    (0.7493,24.02,"FL"),
    (0.991,18.16,"FL"),
    (1.405,12.8,"FL"),
    (2.223,8.097,"FL"),
    (4.199,4.286,"FL"),
    ((0.4*math.pi)/4.0,0,"F"),
    (0.4,45,"FR"),
    (0.4826,37.3,"FR"),
    (0.593,30.37,"FR"),
    (0.7493,24.02,"FR"),
    (0.991,18.16,"FR"),
    (1.405,12.8,"FR"),
    (2.223,8.097,"FR"),
    (4.199,4.286,"FR"),
    (0.4,45,"RL"),
    (0.4826,37.3,"RL"),
    (0.593,30.37,"RL"),
    (0.7493,24.02,"RL"),
    (0.991,18.16,"RL"),
    (1.405,12.8,"RL"),
    (2.223,8.097,"RL"),
    (4.199,4.286,"RL"),
    ((0.4*math.pi)/4.0,0,"R"),
    (0.4,45,"RR"),
    (0.4826,37.3,"RR"),
    (0.593,30.37,"RR"),
    (0.7493,24.02,"RR"),
    (0.991,18.16,"RR"),
    (1.405,12.8,"RR"),
    (2.223,8.097,"RR"),
    (4.199,4.286,"RR")]


InverseControlMappings = [17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]


t = turtle.Pen()
t.speed("slowest")
t.up()
t.goto(300,300)
t.down()
t.speed("fastest")
edges = map.getMapEdgesAndObstacles()
for edge in edges:
    t.up()
    t.goto(edge[0][0]*SCALING-OFFSET,edge[0][1]*SCALING-OFFSET)
    t.down()
    t.goto(edge[1][0]*SCALING-OFFSET,edge[1][1]*SCALING-OFFSET)

#t.speed("slowest")


t.speed("fastest")
#turtle.tracer(False,5)
MyRRT = RRT(map,StartVehiclePos,ControlsList,InverseControlMappings)
i = 0
while i < NUM_NODES:
    print("i = ",i)
    x_rand = MyRRT.generateRandomState()
    status = MyRRT.extend(x_rand,t)
    if (status == Status.ADVANCED or status == Status.REACHED):
        i+=1



turtle.done()