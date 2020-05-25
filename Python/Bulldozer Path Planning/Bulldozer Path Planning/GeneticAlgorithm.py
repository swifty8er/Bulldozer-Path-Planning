import bezier
import random
import math
import numpy as np
from BasicGeometry import BasicGeometry
import scipy.integrate as integrate
from matplotlib import pyplot as plt
MIN_RADIUS = 0.4
KAPPA_MAX = 1.0/MIN_RADIUS


def integrand(t,curve):
    return pow(omega(t,curve),-2.0) * pow(gradMag(t,curve),-1.0)
   

def omega(t,curve):
    return math.sqrt(pow((BasicGeometry.evaluateKappa(curve,t)/KAPPA_MAX),2.0))

def gradMag(t,curve):
    first_derivative_point_array = curve.evaluate_hodograph(t)
    first_derivative_point = [i[0] for i in first_derivative_point_array]
    (dx,dy) = first_derivative_point
    return math.sqrt(dx*dx + dy*dy)

class GeneticAlgorithm:
    def __init__(self,map,start_pose,end_pose,population_size,crossover_prob,mutation_prob,init_size,num_control_points,disk_positions,ax1):
        self._map = map
        self._start_pose = start_pose
        self._end_pose = end_pose
        self._population_size = population_size
        self._init_size = init_size
        self._num_control_points = num_control_points
        self._crossover_prob = crossover_prob
        self._mutation_prob = mutation_prob
        self._disk_positions = disk_positions
        self._population = self.initalisePopulation(ax1)


    @property
    def population(self):
        return self._population


    def testCollision(self,curve):
        edges = self._map.getMapEdgesAndObstacles()
        s = 0.0
        while s<=1.0:
            point_list = curve.evaluate(s)
            point = [i[0] for i in point_list]
            for edge in edges:
                if self._map.disk_radius - BasicGeometry.point2LineDist(edge,point) > np.finfo(np.float32).eps:
                    return False
            for disk_pos in self._disk_positions:
                if 2 * self._map.disk_radius - BasicGeometry.ptDist(disk_pos,point) > np.finfo(np.float32).eps:
                    return False

            s+= 0.005
        return True

    def testRadiusOfCurvature(self,curve):
        s = 0.0
        while s <= 1.0:
            kappa = BasicGeometry.evaluateKappa(curve,s)
            radiusOfCurvature = 1.0/kappa
            if radiusOfCurvature < MIN_RADIUS:
                return False
            s += 0.001
        return True


    def fitnessFunction(self,curve):
        return curve.length
        #return integrate.quad(integrand,0,1,args=(curve))[0]

    def initalisePopulation(self,ax1):
        population = []
        x_points_start = [self._start_pose.x,self._start_pose.x+0.1*math.cos(math.radians(self._start_pose.theta))]
        x_points_end = [self._end_pose.x-0.1*math.cos(math.radians(self._end_pose.theta)),self._end_pose.x]
        y_points_start = [self._start_pose.y,self._start_pose.y+0.1*math.sin(math.radians(self._start_pose.theta))]
        y_points_end = [self._end_pose.y-0.1*math.sin(math.radians(self._end_pose.theta)),self._end_pose.y]
        x = 0
        while x < self._init_size and len(population) < self._population_size:
            x_points_middle = []
            y_points_middle = []
            for i in range(self._num_control_points):
                x_points_middle.append(random.uniform(self._map.min_x,self._map.max_x))
                y_points_middle.append(random.uniform(self._map.min_y,self._map.max_y))
            x_points = x_points_start + x_points_middle + x_points_end
            y_points = y_points_start + y_points_middle + y_points_end
            nodes = np.asfortranarray([x_points,y_points])
            curve = bezier.Curve(nodes,degree=self._num_control_points+3)
            print(curve.nodes)
            curve.plot(100,'blue',ax=ax1)
            plt.draw()
            plt.pause(0.1)
            plt.show()
            if self.testRadiusOfCurvature(curve) and self.testCollision(curve):
                print("Found valid curve")
                population.append(curve)
            x+=1
        return population
            
        