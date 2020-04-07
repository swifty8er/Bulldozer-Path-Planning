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
StartVehiclePos = Vehicle(1,1,90)
ControlsList = [
    (0.6,45,"FL"),
    (0.739,36.5,"FL"),
    (0.928,29.09,"FL"),
    (1.204,22.42,"FL"),
    (1.634,16.44,"FL"),
    (2.42,11.15,"FL"),
    (4.027,6.706,"FL"),
    (8.157,3.31,"FL"),
    ((0.6*math.pi)/4.0,0,"F"),
    (0.6,45,"FR"),
    (0.739,36.5,"FR"),
    (0.928,29.09,"FR"),
    (1.204,22.42,"FR"),
    (1.634,16.44,"FR"),
    (2.42,11.15,"FR"),
    (4.027,6.706,"FR"),
    (8.157,3.31,"FR"),
    (0.6,45,"RL"),
    (0.739,36.5,"RL"),
    (0.928,29.09,"RL"),
    (1.204,22.42,"RL"),
    (1.634,16.44,"RL"),
    (2.42,11.15,"RL"),
    (4.027,6.706,"RL"),
    (8.157,3.31,"RL"),
    ((0.6*math.pi)/4.0,0,"R"),
    (0.6,45,"RR"),
    (0.739,36.5,"RR"),
    (0.928,29.09,"RR"),
    (1.204,22.42,"RR"),
    (1.634,16.44,"RR"),
    (2.42,11.15,"RR"),
    (4.027,6.706,"RR"),
    (8.157,3.31,"RR")]


InverseControlMappings = [17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]


t = turtle.Pen()
t.speed("slowest")
t.up()
t.goto(300,300)
t.down()
t.speed("fastest")
edges = map.getMapEdges()
for edge in edges:
    t.up()
    t.goto(edge[0][0]*SCALING-OFFSET,edge[0][1]*SCALING-OFFSET)
    t.down()
    t.goto(edge[1][0]*SCALING-OFFSET,edge[1][1]*SCALING-OFFSET)

#t.speed("slowest")


#t.speed(3)
turtle.tracer(False,5)
MyRRT = RRT(map,StartVehiclePos,ControlsList,InverseControlMappings)
i = 0
while i < NUM_NODES:
    print("i = ",i)
    x_rand = MyRRT.generateRandomState()
    status = MyRRT.extend(x_rand,t)
    if (status == Status.ADVANCED or status == Status.REACHED):
        i+=1



turtle.done()