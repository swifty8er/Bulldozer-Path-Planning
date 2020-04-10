import turtle
import math
from RRT import RRT
from RRT import Status
from Vehicle import Vehicle
from Maps import Maps
from BasicGeometry import BasicGeometry
SCALING = 100.0
OFFSET = 300.0
NUM_NODES = 100
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

starting_disk_pos = map.initial_disk_pos_xy
for pos in starting_disk_pos:
    t.up()
    t.goto(pos[0]*SCALING-OFFSET,pos[1]*SCALING-OFFSET)
    t.forward(map.disk_radius*SCALING)
    t.left(90)
    t.down()
    t.color("blue")
    t.circle(map.disk_radius*SCALING)

goal_pos = map.goal_pos_xy
for pos in goal_pos:
    t.up()
    t.goto(pos[0]*SCALING-OFFSET,pos[1]*SCALING-OFFSET)
    t.forward(map.disk_radius*SCALING*1.05)
    t.left(90)
    t.down()
    t.color("green")
    t.circle(map.disk_radius*SCALING*1.05)

#t.speed("slowest")

#t.up()
#t.goto(1*SCALING-OFFSET,1*SCALING-OFFSET)
#t.setheading(90)
#t.down()
#for control in ControlsList:
#    (radius,dTheta,direction) = control
#    if direction == "F":
#        t.forward(radius)
#    elif direction == "R":
#        t.back(radius)
#    elif (direction == "FL"):
#        t.circle(radius*SCALING,dTheta)
#    elif (direction == "RL"):
#        t.circle(radius*SCALING,-1*dTheta)
#    else:
#        t.left(180)
#        if (direction == "RR"):
#            t.circle(radius*SCALING,dTheta)
#        else:
#            t.circle(radius*SCALING,-1*dTheta)
#    t.up()
#    t.goto(1*SCALING-OFFSET,1*SCALING-OFFSET)
#    t.setheading(90)
#    t.down()

t.speed("slowest")

MyRRT = RRT(map,StartVehiclePos,ControlsList,InverseControlMappings)
i = 0
while i < NUM_NODES:
    print("i = ",i)
    x_rand = MyRRT.generateRandomState()
    status = MyRRT.extend(x_rand,None)
    if (status == Status.ADVANCED or status == Status.REACHED):
        i+=1



turtle.tracer(False,5)
#t.speed("fastest")
for node in MyRRT.tree.keys():
    t.up()
    t.goto(node.x*SCALING-OFFSET,node.y*SCALING-OFFSET)
    t.setheading(node.theta)
    t.down()
    for n2 in MyRRT.tree[node].keys():
        if (MyRRT.tree[node][n2] in ControlsList and not MyRRT.edgeCollidesWithDirtPile(node,n2)):
            t.up()
            t.goto(n2.x*SCALING-OFFSET,n2.y*SCALING-OFFSET)
            t.down()
            t.dot(4)
            t.setheading(n2.theta)
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
            t.up()
            t.goto(node.x*SCALING-OFFSET,node.y*SCALING-OFFSET)
            t.setheading(node.theta)
            t.down()
            t.dot(4)
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
            (radius,dTheta,direction) = MyRRT.tree[node][n2]
            if direction == "F":
                t.forward(radius*SCALING)
            elif direction == "R":
                t.back(radius*SCALING)
            elif (direction == "FL"):
                t.circle(radius*SCALING,dTheta)
            elif (direction == "RL"):
                t.circle(radius*SCALING,-1*dTheta)
            else:
                t.left(180)
                if (direction == "RR"):
                    t.circle(radius*SCALING,dTheta)
                else:
                    t.circle(radius*SCALING,-1*dTheta)
            t.up()
            t.goto(node.x*SCALING-OFFSET,node.y*SCALING-OFFSET)
            t.setheading(node.theta)
            t.down()
        elif (MyRRT.tree[node][n2] != False):
            print(MyRRT.tree[node][n2])



turtle.done()