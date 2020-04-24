
import turtle
import math
from RRT import RRT
from RRT import Status
from Push_RRT import Push_RRT
from Vehicle import Vehicle
from Maps import Maps
from BasicGeometry import BasicGeometry
import matplotlib.pyplot as plt
fig = plt.figure()
axis = fig.add_subplot(1, 1, 1) # two rows, one column, first plot
plt.xlim(0,5)
plt.ylim(0,5)
v1 = Vehicle(1,1,90)
v2 = Vehicle(1.5,2,70)
curve = v1.createBezierCurveControl(v2)
print(curve.evaluate(0.0))
print(curve.evaluate(1.0))
print(curve.evaluate_hodograph(0.0))
print(curve.evaluate_hodograph(1.0))
curve.plot(100,ax=axis)

plt.show()

#SCALING = 100.0
#OFFSET = 300.0
#NUM_NODES = 1000

##(p1,p2) = BasicGeometry.twoCirclesIntersectionPoints(0.45,(2.5,1.5),0.4826,(1.79,1.915))
##print(p1,p2)
##exit(1)
#StartVehiclePos = Vehicle(1.5,2.5,270)
#ControlsList = [
#    (0.4,45,"FL"),
#    (0.4826,37.3,"FL"),
#    (0.593,30.37,"FL"),
#    (0.7493,24.02,"FL"),
#    (0.991,18.16,"FL"),
#    (1.405,12.8,"FL"),
#    (2.223,8.097,"FL"),
#    (4.199,4.286,"FL"),
#    ((0.4*math.pi)/4.0,0,"F"),
#    (0.4,45,"FR"),
#    (0.4826,37.3,"FR"),
#    (0.593,30.37,"FR"),
#    (0.7493,24.02,"FR"),
#    (0.991,18.16,"FR"),
#    (1.405,12.8,"FR"),
#    (2.223,8.097,"FR"),
#    (4.199,4.286,"FR"),
#    (0.4,45,"RL"),
#    (0.4826,37.3,"RL"),
#    (0.593,30.37,"RL"),
#    (0.7493,24.02,"RL"),
#    (0.991,18.16,"RL"),
#    (1.405,12.8,"RL"),
#    (2.223,8.097,"RL"),
#    (4.199,4.286,"RL"),
#    ((0.4*math.pi)/4.0,0,"R"),
#    (0.4,45,"RR"),
#    (0.4826,37.3,"RR"),
#    (0.593,30.37,"RR"),
#    (0.7493,24.02,"RR"),
#    (0.991,18.16,"RR"),
#    (1.405,12.8,"RR"),
#    (2.223,8.097,"RR"),
#    (4.199,4.286,"RR")]


#InverseControlMappings = [17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]

#MyMaps = Maps()
#map = MyMaps.test_maps[0]
#My_Push = Push_RRT(map)
#MyRRT = RRT(map,StartVehiclePos,ControlsList,InverseControlMappings)
#t = turtle.Pen()
#t.speed("slowest")
#t.up()
#t.goto(300,300)
#t.down()
#t.speed("fastest")
#edges = map.getMapEdgesAndObstacles()
#for edge in edges:
#    t.up()
#    t.goto(edge[0][0]*SCALING-OFFSET,edge[0][1]*SCALING-OFFSET)
#    t.down()
#    t.goto(edge[1][0]*SCALING-OFFSET,edge[1][1]*SCALING-OFFSET)

#starting_disk_pos = map.initial_disk_pos_xy
#for pos in starting_disk_pos:
#    t.up()
#    t.goto(pos[0]*SCALING-OFFSET,pos[1]*SCALING-OFFSET)
#    t.forward(map.disk_radius*SCALING)
#    t.left(90)
#    t.down()
#    t.color("blue")
#    t.circle(map.disk_radius*SCALING)



#goal_pos = map.goal_pos_xy
#for pos in goal_pos:
#    t.up()
#    t.goto(pos[0]*SCALING-OFFSET,pos[1]*SCALING-OFFSET)
#    t.forward(map.disk_radius*SCALING*1.05)
#    t.left(90)
#    t.down()
#    t.color("green")
#    t.circle(map.disk_radius*SCALING*1.05)

#t.color("black")
#i = 0
#while i < NUM_NODES:
#    print("i = ",i)
#    x_rand = MyRRT.generateRandomState()
#    status = MyRRT.extend(x_rand,None,[])
#    if (status == Status.ADVANCED or status == Status.REACHED or status == Status.COLLIDING):
#        i+=1

#My_Push.setRRT(MyRRT)
#turtle.tracer(False,5)
##t.speed(4)
##flag = False
#MyRRT.draw(t,SCALING,OFFSET)
#turtle.update()
##turtle.tracer(True)
#for i in range(len(starting_disk_pos)):
#    disk_pos = starting_disk_pos[i]
#    My_Push.PushToGoals(i,disk_pos,t)



#turtle.done()