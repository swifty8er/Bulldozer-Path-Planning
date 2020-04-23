import unittest
import math
from RRT import RRT
from RRT import Status
from Vehicle import Vehicle
from Maps import Maps
from BasicGeometry import BasicGeometry
NUM_NODES = 500
MyMaps = Maps()
map = MyMaps.test_maps[0]
StartVehiclePos = Vehicle(1.5,2.5,270)
InverseControlMappings = [17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]
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

class Test_TestRRT(unittest.TestCase):
    def test_init(self):
        #test initalising the RRT class
        MyRRT = RRT(map,StartVehiclePos,ControlsList,InverseControlMappings)
        #expect the result to have the starting state inserted as a vertex
        self.assertTrue(MyRRT.hasVertex(StartVehiclePos))
        # the structure is a symmetric matrix
        self.assertTrue(StartVehiclePos in MyRRT._tree[StartVehiclePos].keys())
        # edge between the starting state and itself should not exist (False)
        self.assertEqual(MyRRT._tree[StartVehiclePos][StartVehiclePos],False)



    def test_generate_random_state(self):
        MyRRT = RRT(map,StartVehiclePos,ControlsList,InverseControlMappings)
        # Test the return value
        self.assertNotEqual(MyRRT.generateRandomState(),None)
        randomState = MyRRT.generateRandomState()
        # test the return value
        self.assertIsInstance(randomState,Vehicle)
        # check state in range of map boundary
        self.assertTrue(randomState.x>=map.min_x)
        self.assertTrue(randomState.x<=map.max_x)
        self.assertTrue(randomState.y>=map.min_y)
        self.assertTrue(randomState.y<=map.max_y)
        self.assertTrue(randomState.theta>=0)
        self.assertTrue(randomState.theta<=360)
        self.assertFalse(MyRRT.testStateCollision(randomState))

    def test_extend(self):
        MyRRT = RRT(map,StartVehiclePos,ControlsList,InverseControlMappings)
        # this function mostly just uses the other ones, so just check return type
        self.assertIsInstance(MyRRT.extend(MyRRT.generateRandomState(),None,[]),Status)

    def test_nearest_neighbour(self):
        MyRRT = RRT(map,StartVehiclePos,ControlsList,InverseControlMappings)
        randomState = MyRRT.generateRandomState()
        # test random state
        self.assertIsInstance(randomState,Vehicle)
        nn = MyRRT.nearestNeighbour(randomState)
        # test nearest neighbour state
        self.assertIsInstance(nn,Vehicle)
        for node in MyRRT._tree.keys():
            if (node == nn):
                continue
            # check there is no closer node to the random state
            self.assertTrue(randomState.DistanceTo(node) > randomState.DistanceTo(nn))

    def test_generate_new_state(self):
        MyRRT = RRT(map,StartVehiclePos,ControlsList,InverseControlMappings)
        randomState = MyRRT.generateRandomState()
        nn = MyRRT.nearestNeighbour(randomState)
        (result,x_new,u_new) = MyRRT.generateNewState(randomState,nn)
        # test the return types
        self.assertIsInstance(result,bool)
        self.assertIsInstance(x_new,Vehicle)
        self.assertIsInstance(u_new,tuple)
        # test that the control is valid
        if result:
            self.assertTrue(u_new in ControlsList)
            # test in range of map boundary
            self.assertTrue(x_new.x>=map.min_x)
            self.assertTrue(x_new.x<=map.max_x)
            self.assertTrue(x_new.y>=map.min_y)
            self.assertTrue(x_new.y<=map.max_y)
            # also test no collision
            self.assertFalse(MyRRT.testMoveCollision(nn,u_new))

    def test_add_vertex(self):
        MyRRT = RRT(map,StartVehiclePos,ControlsList,InverseControlMappings)
        randomState = MyRRT.generateRandomState()
        nn = MyRRT.nearestNeighbour(randomState)
        (result,x_new,u_new) = MyRRT.generateNewState(randomState,nn)
        MyRRT.addVertex(x_new)
        # test key inserted in both levels
        self.assertTrue(x_new in MyRRT._tree.keys())
        self.assertTrue(x_new in MyRRT._tree[nn].keys())
        for k in MyRRT._tree.keys():
            # test edge initalisation
            self.assertEqual(MyRRT._tree[k][x_new],False)
        #test these specific edges
        self.assertEqual(MyRRT._tree[x_new][nn],False) # edge has not been inserted yet
        self.assertEqual(MyRRT._tree[nn][x_new],False) # edge has not been inserted yet



    def test_add_edge(self):
        MyRRT = RRT(map,StartVehiclePos,ControlsList,InverseControlMappings)
        randomState = MyRRT.generateRandomState()
        nn = MyRRT.nearestNeighbour(randomState)
        (result,x_new,u_new) = MyRRT.generateNewState(randomState,nn)
        u_inv = ControlsList[InverseControlMappings[ControlsList.index(u_new)]]
        MyRRT.addVertex(x_new)
        MyRRT.addEdge(x_new,nn,u_new,u_inv)
        # test vertex insertion
        self.assertTrue(x_new in MyRRT._tree.keys())
        self.assertTrue(nn in MyRRT._tree.keys())
        self.assertTrue(nn in MyRRT._tree[x_new].keys())
        self.assertTrue(x_new in MyRRT._tree[nn].keys())
        # test edges set correctly
        self.assertEqual(MyRRT._tree[x_new][nn],u_inv)
        self.assertEqual(MyRRT._tree[nn][x_new],u_new)

    def test_grow_tree(self):
        MyRRT = RRT(map,StartVehiclePos,ControlsList,InverseControlMappings)
        i = 0
        while i < NUM_NODES:
            x_rand = MyRRT.generateRandomState()
            self.assertNotEqual(x_rand,None)
            self.assertFalse(MyRRT.testStateCollision(x_rand))
            status = MyRRT.extend(x_rand,None,[])
            self.assertIsInstance(status,Status)
            if (status == Status.ADVANCED or status == Status.REACHED or status == Status.COLLIDING):
                for n1 in MyRRT.tree.keys():
                    for n2 in MyRRT.tree[n1].keys():
                        u1 = MyRRT.tree[n1][n2]
                        u2 = MyRRT.tree[n2][n1]
                        if (u1 != False and u2 != False):
                            test_n2 = n1.applyControl(u1[0],u1[1],u1[2])
                            test_n1 = n2.applyControl(u2[0],u2[1],u2[2])
                            self.assertEqual(test_n2,n2)
                            self.assertEqual(test_n1,n1)
                i+=1



if __name__ == '__main__':
    unittest.main()
   