import unittest
import RRT as RRT
import Vehicle as Vehicle

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
    def RRTInit(self):
        StartVehiclePos = Vehicle(1,1,90)
        MyRRT = RRT(StartVehiclePos,ControlsList)
        self.assertEqual(MyRRT.HasVertex(StartVehiclePos),True,"Starting position should be a vertex in the tree")

if __name__ == '__main__':
    unittest.main()
