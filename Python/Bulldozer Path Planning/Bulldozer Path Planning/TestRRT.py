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
        MyRRT = RRT(map,StartVehiclePos,ControlsList,NUM_NODES)
        #expect the result to have the starting state inserted as a vertex
        self.assertTrue(MyRRT.hasVertex(StartVehiclePos))



    def test_generate_random_state(self):
        MyRRT = RRT(map,StartVehiclePos,ControlsList,NUM_NODES)
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
        MyRRT = RRT(map,StartVehiclePos,ControlsList,NUM_NODES)
        # this function mostly just uses the other ones, so just check return type
        self.assertIsInstance(MyRRT.extend(MyRRT.generateRandomState()),Status)

    def test_nearest_neighbour(self):
        MyRRT = RRT(map,StartVehiclePos,ControlsList,NUM_NODES)
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
        MyRRT = RRT(map,StartVehiclePos,ControlsList,NUM_NODES)
        randomState = MyRRT.generateRandomState()
        nn = MyRRT.nearestNeighbour(randomState)
        (result,x_new,u_new) = MyRRT.generateNewState(randomState,nn)
        # test the return types
        self.assertIsInstance(result,bool)
        self.assertIsInstance(x_new,Vehicle)
        self.assertIsInstance(u_new,tuple)
        # test that the control is valid
        if result:
            # test in range of map boundary
            self.assertTrue(x_new.x>=map.min_x)
            self.assertTrue(x_new.x<=map.max_x)
            self.assertTrue(x_new.y>=map.min_y)
            self.assertTrue(x_new.y<=map.max_y)
            # also test no collision
            self.assertFalse(MyRRT.testMoveCollision(nn,u_new))



    def test_add_edge(self):
        MyRRT = RRT(map,StartVehiclePos,ControlsList,NUM_NODES)
        randomState = MyRRT.generateRandomState()
        nn = MyRRT.nearestNeighbour(randomState)
        (result,x_new,u_new) = MyRRT.generateNewState(randomState,nn)
        u_inv = MyRRT.getInverseControl(u_new)
        MyRRT.addEdge(x_new,nn,u_new,u_inv)
        # test vertex insertion
        self.assertTrue(x_new in MyRRT._tree.keys())
        self.assertTrue(nn in MyRRT._tree.keys())
        self.assertTrue(nn in MyRRT._tree[x_new].keys())
        self.assertTrue(x_new in MyRRT._tree[nn].keys())
        # test edges set correctly
        self.assertEqual(MyRRT._tree[x_new][nn],u_inv)
        self.assertEqual(MyRRT._tree[nn][x_new],u_new)
        # test edges are correct
        self.assertEqual(nn,x_new.applyControl(u_inv[0],u_inv[1],u_inv[2]))
        self.assertEqual(x_new,nn.applyControl(u_new[0],u_new[1],u_new[2]))
    
    def test_grow_tree(self):
        MyRRT = RRT(map,StartVehiclePos,ControlsList,NUM_NODES)
        i = 0
        while i < NUM_NODES:
            x_rand = MyRRT.generateRandomState()
            self.assertNotEqual(x_rand,None)
            self.assertFalse(MyRRT.testStateCollision(x_rand))
            status = MyRRT.extend(x_rand)
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
   