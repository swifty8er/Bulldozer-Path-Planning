import math
import numpy as np


class BasicGeometry():
    #functions for basic geometry and list finding
    @staticmethod
    def arcLineCollisionIterative(start_position,control,line,num_steps,disk_radius):
        angle = control[1]
        delta_angle = angle/float(num_steps)
        for i in range(num_steps+1):
            new_position = start_position.applyControl(control[0],delta_angle,control[2])
            point = (new_position.x,new_position.y)
            dist = BasicGeometry.point2LineDist(line,point)
            if dist < disk_radius:
                return True
            delta_angle *= 2.0
        return False

    @staticmethod
    def doLinesIntersect(line_1, line_2):
        x1 = line_1[0][0]
        x2 = line_1[1][0]
        x3 = line_2[0][0]
        x4 = line_2[1][0]
        y1 = line_1[0][1]
        y2 = line_1[1][1]
        y3 = line_2[0][1]
        y4 = line_2[1][1]
        diff_x = x1 - x3
        diff_y = y1 - y3
        denominator = (x4 - x3)*(y1 - y2) - (x1 - x2)*(y4 - y3)
        if (denominator != 0):
            t1 = ((y3 - y4)*diff_x + (x4 - x3)*diff_y)/denominator
            t2 = ((y1 - y2)*diff_x + (x2 - x1)*diff_y)/denominator
            if ((t1 <= 1) and (t1 >= 0) and (t2 <= 1) and (t2 >= 0)):
                intersect = True
            else:
                intersect = False
        else:
            intersect = False

        return intersect

    @staticmethod
    #find the perpendicuar distance between a point and a line if within the
    #bounds of the line (line is not extended) otherwise find the distance to the closest point
    def point2LineDist(line, point):
        x1 = line[0][0]
        x2 = line[1][0]
        y1 = line[0][1]
        y2 = line[1][1]
        if (abs(y1-y2) < np.finfo(np.float32).eps):
            inbounds = ((point[0] <= max([x1,x2])) and (point[0] >= min([x1,x2])))
        else:
            perp_gradient = -(x1-x2)/(y1-y2)
            perp_intercept_1 = y1 - perp_gradient*x1
            perp_intercept_2 = y2 - perp_gradient*x2
            perp_lines_y = [perp_gradient*point[0] + perp_intercept_1]
            perp_lines_y.append(perp_gradient*point[0] + perp_intercept_2)
            inbounds = ((point[1] <= max(perp_lines_y)) and (point[1] >= min(perp_lines_y)))

        if (inbounds):
            a = np.array([x1,y1,0]) - np.array([x2,y2,0])
            b = np.array([point[0],point[1],0]) - np.array([x2,y2,0])
            dist = np.linalg.norm(np.cross(a,b)) / np.linalg.norm(a)
        else:
            distances = [math.sqrt((point[0]-x1)**2+(point[1]-y1)**2)]
            distances.append(math.sqrt((point[0]-x2)**2+(point[1]-y2)**2))
            dist = min(distances)

        return dist

    @staticmethod
    def isPointInArray(array, point, tolerance):
        index = -1
        i = 0
        if len(array) > 0 and len(array[0]) > 0 and len(point) > 0:
            if len(array[0]) < len(point):
                num_of_comp = len(array)
            else:
                num_of_comp = len(point)
            while ((i < len(array)) and (index == -1)):
                j = 0
                equal = True
                while j < num_of_comp and equal == True: 
                    if (abs(array[i][j] - point[j]) >= tolerance):
                        equal = False
                    j += 1
                if equal == True:
                    index = i
                i += 1
        return index

    @staticmethod
    def circlePoints(centre, radius, segments):
        x_axis = []
        y_axis = []
        for i in range(segments):
            x = centre[0] + radius*math.cos((i-1)*(2*math.pi/(segments-1)))
            y = centre[1] + radius*math.sin((i-1)*(2*math.pi/(segments-1)))
            x_axis.append(x)
            y_axis.append(y)

        return [x_axis, y_axis]

    @staticmethod
    def doesCircleIntersectLine(centre, radius, line):
        dist = BasicGeometry.point2LineDist(line, centre)
        if (dist <= radius):
            intersect = True;
        else:
            intersect = False;

        return intersect

    @staticmethod
    def ptDist(pt1, pt2):
        distance = math.sqrt((pt1[0]-pt2[0])**2+(pt1[1]-pt2[1])**2)
        return distance

    @staticmethod
    def findClosestPoint(start_point, dest_points):
        closest_dist = math.inf
        #check all destination points
        i = 0
        for dest_point in dest_points:
            #find the distance between the start point and that particular destination point
            curr_dist = BasicGeometry.ptDist(start_point, dest_point)
            if (curr_dist < closest_dist):
                closest_dist = curr_dist
                closest_point = i
            i += 1
        return closest_point