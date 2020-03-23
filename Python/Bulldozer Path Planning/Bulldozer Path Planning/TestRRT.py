import unittest
from RRT import RRT
from RRT import Status
from Vehicle import Vehicle
from Maps import Maps

MyMaps = Maps()
map = MyMaps.test_maps[0]
StartVehiclePos = Vehicle(1,1,90)
ControlsList = [
    (0.6,45),
    (0.739,36.5),
    (0.928,29.09),
    (1.204,22.42),
    (1.634,16.44),
    (2.42,11.15),
    (4.027,6.706),
    (8.157,3.31)
    ]

class Test_TestRRT(unittest.TestCase):
    def test_init(self):
        MyRRT = RRT(map,StartVehiclePos,ControlsList)
        self.assertTrue(MyRRT.hasVertex(StartVehiclePos))

    def test_generate_random_state(self):
        MyRRT = RRT(map,StartVehiclePos,ControlsList)
        self.assertNotEqual(MyRRT.generateRandomState(),None)
        randomState = MyRRT.generateRandomState()
        self.assertIsInstance(randomState,Vehicle)
        self.assertTrue(randomState.getX()>=map.min_x)
        self.assertTrue(randomState.getX()<=map.max_x)
        self.assertTrue(randomState.getY()>=map.min_y)
        self.assertTrue(randomState.getY()<=map.max_y)
        # also test that no collision with obstacle

    def test_extend(self):
        MyRRT = RRT(map,StartVehiclePos,ControlsList)
        self.assertIsInstance(MyRRT.extend(MyRRT.generateRandomState()),Status)

    def test_nearest_neighbour(self):
        MyRRT = RRT(map,StartVehiclePos,ControlsList)
        randomState = MyRRT.generateRandomState()
        self.assertIsInstance(randomState,Vehicle)
        nn = MyRRT.nearestNeighbour(randomState)
        self.assertIsInstance(nn,Vehicle)
        for node in MyRRT._tree.keys():
            self.assertTrue(randomState.DistanceTo(node) > randomState.DistanceTo(nn))


if __name__ == '__main__':
    unittest.main()
