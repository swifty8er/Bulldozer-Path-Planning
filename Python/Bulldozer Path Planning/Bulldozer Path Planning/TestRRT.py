import unittest
from RRT import RRT
from Vehicle import Vehicle
from Maps import Maps

MyMaps = Maps()
map = MyMaps.test_maps[0]

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
        StartVehiclePos = Vehicle(1,1,90)
        MyRRT = RRT(map,StartVehiclePos,ControlsList)
        self.assertTrue(MyRRT.hasVertex(StartVehiclePos))

    def test_generate_random_state(self):
        StartVehiclePos = Vehicle(1,1,90)
        MyRRT = RRT(map,StartVehiclePos,ControlsList)
        self.assertNotEqual(MyRRT.generateRandomState(),None)
        randomState = MyRRT.generateRandomState()
        self.assertTrue(randomState.getX()>=map.min_x)
        self.assertTrue(randomState.getX()<=map.max_x)
        self.assertTrue(randomState.getY()>=map.min_y)
        self.assertTrue(randomState.getY()<=map.max_y)
        # also test that no collision with obstacle


if __name__ == '__main__':
    unittest.main()
