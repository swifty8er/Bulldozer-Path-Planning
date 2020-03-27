import unittest
import math
from RRT import RRT
from RRT import Status
from Vehicle import Vehicle
from Maps import Maps

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

class Test_TestRRT(unittest.TestCase):
    def test_init(self):
        #test initalising the RRT class
        MyRRT = RRT(map,StartVehiclePos,ControlsList)
        #expect the result to have the starting state inserted as a vertex
        self.assertTrue(MyRRT.hasVertex(StartVehiclePos))
        # the structure is a symmetric matrix
        self.assertTrue(StartVehiclePos in MyRRT._tree[StartVehiclePos].keys())
        # edge between the starting state and itself should not exist (False)
        self.assertEqual(MyRRT._tree[StartVehiclePos][StartVehiclePos],False)



    def test_generate_random_state(self):
        MyRRT = RRT(map,StartVehiclePos,ControlsList)
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
        # also test that no collision with obstacle

    def test_extend(self):
        MyRRT = RRT(map,StartVehiclePos,ControlsList)
        # this function mostly just uses the other ones, so just check return type
        self.assertIsInstance(MyRRT.extend(MyRRT.generateRandomState()),Status)

    def test_nearest_neighbour(self):
        MyRRT = RRT(map,StartVehiclePos,ControlsList)
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
        MyRRT = RRT(map,StartVehiclePos,ControlsList)
        randomState = MyRRT.generateRandomState()
        nn = MyRRT.nearestNeighbour(randomState)
        (result,x_new,u_new) = MyRRT.generateNewState(randomState,nn)
        # test the return types
        self.assertIsInstance(result,bool)
        self.assertIsInstance(x_new,Vehicle)
        self.assertIsInstance(u_new,tuple)
        # test that the control is valid
        self.assertTrue(u_new in ControlsList)
        # test in range of map boundary
        self.assertTrue(x_new.getX()>=map.min_x)
        self.assertTrue(x_new.getX()<=map.max_x)
        self.assertTrue(x_new.getY()>=map.min_y)
        self.assertTrue(x_new.getY()<=map.max_y)
        # also test no collision

    def test_add_vertex(self):
        MyRRT = RRT(map,StartVehiclePos,ControlsList)
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
        MyRRT = RRT(map,StartVehiclePos,ControlsList)
        randomState = MyRRT.generateRandomState()
        nn = MyRRT.nearestNeighbour(randomState)
        (result,x_new,u_new) = MyRRT.generateNewState(randomState,nn)
        MyRRT.addVertex(x_new)
        MyRRT.addEdge(x_new,nn,u_new)
        # test vertex insertion
        self.assertTrue(x_new in MyRRT._tree.keys())
        self.assertTrue(nn in MyRRT._tree.keys())
        self.assertTrue(nn in MyRRT._tree[x_new].keys())
        self.assertTrue(x_new in MyRRT._tree[nn].keys())
        # test edges set correctly
        self.assertEqual(MyRRT._tree[x_new][nn],u_new)
        self.assertEqual(MyRRT._tree[nn][x_new],u_new)


if __name__ == '__main__':
    unittest.main()
   