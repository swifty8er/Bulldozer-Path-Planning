import unittest
import math
from RRT import RRT
from RRT import Status
from Quadtree import Quadtree
from DistMetree import DistMetree
from Vehicle import Vehicle
from Maps import Maps
from BasicGeometry import BasicGeometry
NUM_NODES = 1000
MyMaps = Maps()
map = MyMaps.test_maps[0]
starting_xy = map.initial_vehicle_pos_xy
StartVehiclePos = Vehicle(starting_xy[0],starting_xy[1],90) #change to random heading
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
class Test_TestOctree(unittest.TestCase):
    def testQuadtree(self):
        print("Initalising RRT...")
        MyRRT = RRT(map,StartVehiclePos,ControlsList,NUM_NODES)
        i = 0
        print("Building RRT...")
        while i < MyRRT.num_nodes:
            print("i = ",i)
            x_rand = MyRRT.generateRandomState()
            status = MyRRT.extend(x_rand)
            if (status == Status.ADVANCED or status == Status.REACHED):
                i+=1
        print("Initialising DistMetree")
        MyDistMetree = DistMetree(StartVehiclePos,None,MyRRT.computeMaxDistanceMetricBetweenNodes(StartVehiclePos),180.0)
        i = 0
        for node in MyRRT.tree:
            MyDistMetree.addState(node)
            i += 1
            if i > MyRRT.num_nodes / 4.0:
                break
        for n in MyRRT.tree:
            if MyDistMetree.stateWithinRadiusOfQuery(n,0.1):
                print("Node (%.2f,%.2f,%.2f) has similar state in octree" % (n.x,n.y,n.theta))
            else:
                print("No similar state")
if __name__ == '__main__':
    unittest.main()
