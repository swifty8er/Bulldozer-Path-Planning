import math
from RRT import RRT
from RRT import Status
from Push_RRT import Push_RRT
from Vehicle import Vehicle
from Maps import Maps
from BasicGeometry import BasicGeometry
import matplotlib.pyplot as plt
fig = plt.figure()
axis = fig.add_subplot(2, 1, 1) # two rows, one column, first plot
ax2 = fig.add_subplot(2,1,2)
#plt.xlim(0,5)
#plt.ylim(0,5)
#v1 = Vehicle(2.6,3.45,306.13)
#v2 = Vehicle(3.07,2.92,241.91)
#curve = v1.createBezierCurveControl(v2)
#print("Tangent angle of starting path is =",BasicGeometry.getTangentAngleOfBezierCurveAtPoint(curve,0.0))
#print("Tangent angle of ending path is =",BasicGeometry.getTangentAngleOfBezierCurveAtPoint(curve,1.0))
#print(curve.evaluate_hodograph(0.0))
#print(curve.evaluate_hodograph(1.0))
#curve.plot(100,ax=axis)

#plt.show()

SCALING = 100.0
OFFSET = 300.0


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



MyMaps = Maps()
map = MyMaps.test_maps[0]
My_Push = Push_RRT(map,(0.4*math.pi)/4.0)
starting_xy = map.initial_vehicle_pos_xy[0].tolist()
StartVehiclePos = Vehicle(starting_xy[0],starting_xy[1],90)
MyRRT = RRT(map,StartVehiclePos,ControlsList,5000)

map.plotStartingMap(axis,True)
map.plotStartingMap(ax2,True)
plt.draw()
plt.pause(1)
plt.show(block=False)
i = 0
while i < MyRRT.num_nodes:
    print("i = ",i)
    x_rand = MyRRT.generateRandomState()
    status = MyRRT.extend(x_rand)
    if (status == Status.ADVANCED or status == Status.REACHED):
        i+=1


MyRRT.draw(axis)
plt.draw()
plt.pause(1)
plt.show(block=False)
My_Push.setRRT(MyRRT)
##t.speed(4)
##flag = False
#turtle.update()
##turtle.tracer(True)
starting_disk_pos = map.initial_disk_pos_xy
for i in range(len(starting_disk_pos)):
    disk_pos = starting_disk_pos[i]
    My_Push.PushToGoals(i,disk_pos,axis)
    My_Push.draw(ax2)
    plt.draw()
    plt.pause(1)
    plt.show(block=False)
    plt.pause(30)



