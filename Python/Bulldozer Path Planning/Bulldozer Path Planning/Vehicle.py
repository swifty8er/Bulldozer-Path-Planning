WIDTH = 1.0
LENGTH = 1.0
class Vehicle:
    # initialise a vehicle, setting its x,y coord and heading
    def __init__(self,x,y,theta):
        self._x = x
        self._y = y
        self._theta = theta
        self._boundaryLeftX = self._x - 0.5*WIDTH
        self._boundaryRightX = self._x + 0.5*WIDTH
        self._boundaryTopY = self._y + 0.5*LENGTH
        self._boundaryBottomY = self._y - 0.5*LENGTH
        print("Initalised vehicle (%.2f,%.2f,%.2f)" % (self._x,self._y,self._theta))

    @property
    def getX(self):
        return self._x
    @property
    def getY(self):
        return self._y
    @property
    def getTheta(self):
        return self._theta

    def applyControl(self,radius,deltaTheta,direction):
        pass #apply the control (radius,deltaTheta) in the direction specified to generate a new Vehicle object


