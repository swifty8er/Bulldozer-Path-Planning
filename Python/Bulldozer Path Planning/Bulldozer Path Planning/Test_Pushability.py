
import turtle
import math
from RRT import RRT
from RRT import Status
from Push_RRT import Push_RRT
from Vehicle import Vehicle
from Maps import Maps
from BasicGeometry import BasicGeometry
SCALING = 100.0
OFFSET = 300.0
NUM_NODES = 2000

#(p1,p2) = BasicGeometry.twoCirclesIntersectionPoints(0.45,(2.5,1.5),0.4826,(1.79,1.915))
#print(p1,p2)
#exit(1)
MyMaps = Maps()
map = MyMaps.test_maps[0]
My_Push = Push_RRT(map)
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

starting_disk_pos = map.initial_disk_pos_xy
for pos in starting_disk_pos:
    t.up()
    t.goto(pos[0]*SCALING-OFFSET,pos[1]*SCALING-OFFSET)
    t.forward(map.disk_radius*SCALING)
    t.left(90)
    t.down()
    t.color("blue")
    t.circle(map.disk_radius*SCALING)
    push_points = My_Push.getPushPoints(pos)
    for veh in push_points:
        t.color("black")
        t.up()
        t.goto(veh.x*SCALING-OFFSET,veh.y*SCALING-OFFSET)
        t.setheading(veh.theta)
        t.back(SCALING/10)
        t.down()
        t.forward(SCALING/10)
        t.left(145)
        t.forward(SCALING/20)
        t.back(SCALING/20)
        t.right(145)
        t.right(145)
        t.forward(SCALING/20)
        t.back(SCALING/20)
        t.left(145)
        t.back(SCALING/10)


goal_pos = map.goal_pos_xy
for pos in goal_pos:
    t.up()
    t.goto(pos[0]*SCALING-OFFSET,pos[1]*SCALING-OFFSET)
    t.forward(map.disk_radius*SCALING*1.05)
    t.left(90)
    t.down()
    t.color("green")
    t.circle(map.disk_radius*SCALING*1.05)

t.color("black")

for d in range(len(starting_disk_pos)):
    disk_pos = starting_disk_pos[d]
    My_Push.PushToGoals(d,disk_pos)

turtle.done()