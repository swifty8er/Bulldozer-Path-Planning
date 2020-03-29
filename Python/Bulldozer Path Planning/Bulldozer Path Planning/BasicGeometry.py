import math
import numpy as np


class BasicGeometry():
    #functions for basic geometry and list finding
    @staticmethod
    def arcLineCollisionAlgorithm(start_position,control,line,disk_radius):
        direction = control[2]
        end_position = start_position.applyControl(control[0],control[1],control[2])
        if direction == "F" or direction == "R":
            p1 = (start_position.x,start_position.y)
            p2 = (end_position.x,end_position.y)
            p3 = (line[0][0],line[0][1])
            p4 = (line[1][0],line[1][1])
            R1_2 = (p2[0]-p1[0])**2+(p2[1]-p1[1])**2
            R2_2 = (p4[0]-p3[0])**2+(p4[1]-p3[1])**2
            D_4321 = (p4[0]-p3[0])*(p2[0]-p1[0])+(p4[1]-p3[1])*(p2[1]-p1[1])
            D_3121 = (p3[0]-p1[0])*(p2[0]-p1[0])+(p3[1]-p1[1])*(p2[1]-p1[1])
            D_4331 = (p4[0]-p3[0])*(p3[0]-p1[0])+(p4[1]-p3[1])*(p3[1]-p1[1])

            s = ((D_4321 * D_4331) + (D_3121 * R2_2))/(R1_2*R2_2 + D_4321**2)

            t = ((D_4321 * D_3121) - (D_4331 * R1_2))/(R1_2*R2_2 + D_4321**2)

            if 0 <= s and s <= 1 and 0 <= t and t <= 1:
                L1_s_x = (1-s)*p1[0] + s*p2[0]
                L1_s_y = (1-s)*p1[1] + s*p2[1]

                L2_t_x = (1-t)*p3[0] + t*p4[0]
                L2_t_y = (1-t)*p3[1] + t*p4[1]

                if (disk_radius - BasicGeometry.ptDist((L1_s_x,L1_s_y),(L2_t_x,L2_t_y)) ) > np.finfo(np.float32).eps:
                    return True
                else:
                    return False
            else:
                D_4121 = np.dot(BasicGeometry.tuple_sub(p4,p1),BasicGeometry.tuple_sub(p2,p1))
                D_4332 = np.dot(BasicGeometry.tuple_sub(p4,p3),BasicGeometry.tuple_sub(p3,p2))
                #others could be converted to dot product form too

                s_p3 = (D_3121/R1_2)

                s_p4 = (D_4121/R1_2)

                t_p1 = (-1*D_4331/R2_2)

                t_p2 = (-1*D_4332/R2_2)
                distances = []

                P_3 = BasicGeometry.GetPointOnLine(p1,p2,s_p3)
                distances.append(BasicGeometry.ptDist(P_3,p3))
                P_4 = BasicGeometry.GetPointOnLine(p1,p2,s_p4)
                distances.append(BasicGeometry.ptDist(P_4,p4))
                P_1 = BasicGeometry.GetPointOnLine(p3,p4,t_p1)
                distances.append(BasicGeometry.ptDist(P_1,p1))
                P_2 = BasicGeometry.GetPointOnLine(p3,p4,t_p2)
                distances.append(BasicGeometry.ptDist(P_2,p2))
                if (disk_radius - min(distances) ) > np.finfo(np.float32).eps:
                    return True
                else:
                    return False
            

        else:
            y2 = line[1][1]
            x2 = line[1][0]

            y1 = line[0][1]
            x1 = line[0][0]

            m1 = (y2-y1)/(x2-x1)
            b = y2 - m1*x2

            m2 = -1/m1
            
            if direction == "FL" or direction == "RL":
                a = start_position.x + control[0]*math.cos(math.radians(start_position.theta)+math.pi/2)
                b = start_position.y + control[0]*math.sin(math.radians(start_position.theta)+math.pi/2)
            else:
                a = start_position.x + control[0]*math.cos(math.radians(start_position.theta)-math.pi/2)
                b = start_position.y + control[0]*math.sin(math.radians(start_position.theta)-math.pi/2)

            c = ((x2-x1)/(y2-y1))*a + b

            x = ( ((x2-x1)/(y2-y1))*a + b - y2 + ((y2-y1)/(x2-x1))*x2 ) / ( ((y2-y1)/(x2-x1)) + ((x2-x1)/(y2-y1)) )

            y = m2*x + c

            test_y = m1*x + b

            if (math.fabs(test_y-y) > np.finfo(np.float32).eps):
                raise Exception("Error finding intersection point of perp bisector and line")

            v_x = x-a
            v_y = y-b

            alpha = math.atan2(v_y,v_x)

            gamma_1 = math.atan2(start_position.y,start_position.x)
            gamma_2 = math.atan2(end_position.y,end_position.x)

            start_point = (start_position.x,start_position.y)
            end_point = (end_position.x,end_position.y)
            circle_centre = (a,b)

            if (alpha<0 and gamma_2<=alpha and alpha<=gamma_1) or (alpha>=0 and gamma_2>=alpha and alpha>= gamma_1):
                D = math.sqrt(v_x**2 + v_y**2)
                d = D - control[0]
                if (disk_radius - d ) > np.finfo(np.float32).eps:
                    return True
                else:
                    return False
            elif (disk_radius - BasicGeometry.ptDist(line[0],start_point)) > np.finfo(np.float32).eps:
                return True
            elif (disk_radius - BasicGeometry.ptDist(line[1],start_point)) > np.finfo(np.float32).eps:
                return True
            elif (disk_radius - BasicGeometry.ptDist(line[0],end_point)) > np.finfo(np.float32).eps:
                return True
            elif (disk_radius - BasicGeometry.ptDist(line[1],end_point)) > np.finfo(np.float32).eps:
                return True
            elif (disk_radius - (BasicGeometry.ptDist(line[0],circle_centre) - control[0])) > np.finfo(np.float32).eps:
                return True
            elif (disk_radius - (BasicGeometry.ptDist(line[1],circle_centre) - control[0])) > np.finfo(np.float32).eps:
                return True
            else:
                return False


    @staticmethod
    def GetPointOnLine(p1,p2,param):
        return ((1-param)*p1[0] + param*p2[0] , (1-param)*p1[1] + param*p2[1])

    @staticmethod
    def tuple_sub(t1,t2):
        return (t1[0]-t2[0],t1[1]-t2[1])

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