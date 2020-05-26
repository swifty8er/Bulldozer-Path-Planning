import math
import numpy as np
import bezier
from scipy import special as sp


class BasicGeometry():
    #functions for basic geometry and list finding
    @staticmethod
    def manhattanDistance(pos1,pos2):
        (x1,y1) = pos1
        (x2,y2) = pos2
        return abs(x1-x2) + abs(y1-y2)


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
                if (0<= s_p3 and s_p3 <= 1):
                    P_3 = BasicGeometry.GetPointOnLine(p1,p2,s_p3)
                elif s_p3>1:
                    P_3 = p2
                else:
                    P_3 = p1
                distances.append(BasicGeometry.ptDist(P_3,p3))
                if (0 <= s_p4 and s_p4 <= 1):
                    P_4 = BasicGeometry.GetPointOnLine(p1,p2,s_p4)
                elif s_p4>1:
                    P_4 = p2
                else:
                    P_4 = p1
                distances.append(BasicGeometry.ptDist(P_4,p4))
                if (0<=t_p1 and t_p1<=1):
                    P_1 = BasicGeometry.GetPointOnLine(p3,p4,t_p1)
                elif t_p1>1:
                    P_1 = p4
                else:
                    P_1 = p3
                distances.append(BasicGeometry.ptDist(P_1,p1))
                if (0<=t_p2 and t_p2 <=1):
                    P_2 = BasicGeometry.GetPointOnLine(p3,p4,t_p2)
                elif t_p2>1:
                    P_2 = p4
                else:
                    P_2 = p3
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
            if (x2-x1 == 0):
                m1 = None
                m2 = 0
            else:
                m1 = (y2-y1)/(x2-x1)
                b_1 = y2 - m1*x2
                if m1 == 0:
                    m2 = None
                else:
                    m2 = -1/m1
            
            if direction == "FL" or direction == "RL":
                a = start_position.x + control[0]*math.cos(math.radians(start_position.theta)+math.pi/2)
                b = start_position.y + control[0]*math.sin(math.radians(start_position.theta)+math.pi/2)
            else:
                a = start_position.x + control[0]*math.cos(math.radians(start_position.theta)-math.pi/2)
                b = start_position.y + control[0]*math.sin(math.radians(start_position.theta)-math.pi/2)


            if m2 == None:
                x = a
                y = b_1
            elif m2 == 0 and m1 == None:
                y = b
                x = x1
            else:
                c = ((x2-x1)/(y2-y1))*a + b

                x = ( ((x2-x1)/(y2-y1))*a + b - y2 + ((y2-y1)/(x2-x1))*x2 ) / ( ((y2-y1)/(x2-x1)) + ((x2-x1)/(y2-y1)) )

                y = m2*x + c

                test_y = m1*x + b_1

                if (math.fabs(test_y-y) > np.finfo(np.float32).eps):
                    raise Exception("Error finding intersection point of perp bisector and line")


           
            v_x = x-a
            v_y = y-b
            circle_centre = (a,b)
            start_point = (start_position.x,start_position.y)
            end_point = (end_position.x,end_position.y)
            #check that x,y are on the LINE SEGMENT...
            if (x2-x1) == 0:
                s_1 = None
            else:
                s_1 = (x-x1)/(x2-x1)
            if (y2-y1) == 0:
                s_2 = None
            else:
                s_2 = (y-y1)/(y2-y1)

            if (s_1!=None and s_2!=None and 0<= s_1 and s_1 <= 1 and 0<= s_2 and s_2 <=1 and s_1 == s_2):

                alpha = math.atan2(v_y,v_x)

                gamma_1 = math.atan2(start_position.y,start_position.x)
                gamma_2 = math.atan2(end_position.y,end_position.x)


                if (alpha<0 and gamma_2<=alpha and alpha<=gamma_1) or (alpha>=0 and gamma_2>=alpha and alpha>= gamma_1):
                    D = math.sqrt(v_x**2 + v_y**2)
                    d = D - control[0]
                    if (disk_radius - d ) > np.finfo(np.float32).eps:
                        return True
                    else:
                        return False
            else:
                if BasicGeometry.circleArcIntersectsLine(circle_centre,control[0],line):
                    return True
                elif (disk_radius - BasicGeometry.point2LineDist(line,start_point)) > np.finfo(np.float32).eps:
                    return True
                elif (disk_radius - BasicGeometry.point2LineDist(line,end_point)) > np.finfo(np.float32).eps:
                    return True
                else:
                    vec_between_points = BasicGeometry.vec_from_points(line[0],circle_centre)
                    beta = math.atan2(vec_between_points[1],vec_between_points[0])
                    radial_vec = (a+control[0]*math.cos(beta),b+control[0]*math.sin(beta))
                    if (disk_radius - BasicGeometry.vec_mag(BasicGeometry.vec_sub(vec_between_points,radial_vec))) > np.finfo(np.float32).eps:
                        return True
                    vec_between_points = BasicGeometry.vec_from_points(line[1],circle_centre)
                    beta = math.atan2(vec_between_points[1],vec_between_points[0])
                    radial_vec = (a+control[0]*math.cos(beta),b+control[0]*math.sin(beta))
                    if (disk_radius - BasicGeometry.vec_mag(BasicGeometry.vec_sub(vec_between_points,radial_vec))) > np.finfo(np.float32).eps:
                        return True
              
                    return False


    @staticmethod
    def circleArcIntersectsLine(circle_centre,radius,line):
        y2 = line[1][1]
        x2 = line[1][0]
        y1 = line[0][1]
        x1 = line[0][0]
        if (y2-y1 == 0):
            y = y1
            if (x1<x2):
                x_range = [x1,x2]
            else:
                x_range = [x2,x1]
            delta = radius**2 - (y-circle_centre[1])**2
            if (delta<0):
                return False
            the_x = math.sqrt(delta) + circle_centre[0]
            if (the_x >= x_range[0] and the_x <= x_range[1]):
                return True
            else:
                return False
        elif (x2-x1==0):
            x = x1
            if (y1 < y2):
                y_range = [y1,y2]
            else:
                y_range = [y2,y1]
            delta = radius**2 - (x-circle_centre[0])**2
            if (delta<0):
                return False
            the_y = math.sqrt(delta) + circle_centre[1]
            if (the_y >= y_range[0] and the_y <= y_range[1]):
                return True
            else:
                return False
        else:
            return False #work for any line y=mx+c

    @staticmethod
    def vec_from_points(p1,p2):
        return (p2[0]-p1[0],p2[1]-p1[1])


    @staticmethod
    #return the angle of a vector from the x-axis in radians
    def vector_angle(vector):
        alpha = math.atan2(vector[1],vector[0])
        if alpha < 0:
            return 2*math.pi + alpha
        else:
            return alpha

    @staticmethod
    def vec_mag(vector):
        return math.sqrt(vector[0]**2+vector[1]**2)

    @staticmethod
    def vec_sub(v1,v2):
        return (v1[0]-v2[0],v1[1]-v2[1])

    @staticmethod
    def GetPointOnLine(p1,p2,param):
        return ((1-param)*p1[0] + param*p2[0] , (1-param)*p1[1] + param*p2[1])

    @staticmethod
    def tuple_sub(t1,t2):
        return (t1[0]-t2[0],t1[1]-t2[1])

    @staticmethod
    def arcLineCollisionIterative(start_position,control,line,num_steps,disk_radius):
        if control[2] == "F" or control[2] == "R":
            end_position = startPosition.applyControl(control[0],control[1],control[2])
            L2 = [[start_position.x,start_position.y],[end_position.x,end_position.y]]
            return BasicGeometry.doLinesIntersect(line,L2)
        else:
            angle = control[1]
            delta_angle = angle/float(num_steps)
            the_angle = delta_angle
            for i in range(num_steps+1):
                new_position = start_position.applyControl(control[0],the_angle,control[2])
                point = (new_position.x,new_position.y)
                dist = BasicGeometry.point2LineDist(line,point)
                if (disk_radius - dist) > np.finfo(np.float32).eps:
                    return True
                the_angle += delta_angle
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
    #return the tangent angle of a bezier curve at a point along the curve in degrees
    def getTangentAngleOfBezierCurveAtPoint(bezierCurve,s):
        point_array = bezierCurve.evaluate(s).tolist()
        point = [i[0] for i in point_array]
        d_points = bezierCurve.evaluate_hodograph(s).tolist()
        d_list = [i[0] for i in d_points]
        (dx,dy) = (d_list[0],d_list[1])
        angle = BasicGeometry.vector_angle(BasicGeometry.vec_from_points((point[0],point[1]),(point[0]+dx,point[1]+dy)))
        return math.degrees(angle)

    @staticmethod
    def getGradientOfLine(p1,p2):
        (x1,y1) = p1
        (x2,y2) = p2
        if (x2-x1) == 0:
            return math.inf
        else:
            return (y2-y1)/(x2-x1)

    @staticmethod
    def getKappa(t,bezierCurve,ddy,ddx):
        first_derivative_point_array = bezierCurve.evaluate_hodograph(t)
        first_derivative_point = [i[0] for i in first_derivative_point_array]
        (dx,dy) = first_derivative_point
        numerator = dx * ddy - ddx * dy
        denominator = pow(dx*dx + dy*dy, 1.5)
        return abs(numerator / denominator)
    
    
    @staticmethod
    def evaluateKappa(curve,t):
        first_derivative_point_array = curve.evaluate_hodograph(t)
        first_derivative_point = [i[0] for i in first_derivative_point_array]
        (dx,dy) = first_derivative_point
        (ddx,ddy) = BasicGeometry.getSecondDerivativeOfBezierCurve(curve,t)
        numerator = dx * ddy - ddx * dy
        denominator = pow(dx*dx + dy*dy, 1.5)
        kappa = abs(numerator / denominator)
        return kappa



    @staticmethod
    def getSecondDerivativeOfBezierCurve(bezierCurve,t):
        originalControlPoints = bezierCurve.nodes
        x_points = originalControlPoints[0]
        y_points = originalControlPoints[1]
        points = []
        for i in range(len(x_points)):
            points.append((x_points[i],y_points[i]))
        n = bezierCurve.degree
        first_derivative_points = BasicGeometry.reduceControlPoints(points,n)
        second_derivative_points = BasicGeometry.reduceControlPoints(first_derivative_points,n-1)
        k = n-1
        x_sum = 0
        y_sum = 0
        for i in range(k):
            bin = sp.comb(k,i)
            rest = bin*pow((1-t),(k-i))*pow(t,i)
            x_sum += rest*second_derivative_points[i][0]
            y_sum += rest*second_derivative_points[i][1]
        return (x_sum,y_sum)



    @staticmethod
    def reduceControlPoints(points,n):
        new_points = []
        for i in range(n):
            new_point = (n*(points[i+1][0]-points[i][0]),n*(points[i+1][1]-points[i][1]))
            new_points.append(new_point)
        return new_points


    @staticmethod
    def findVectorLinesIntersectionPoint(x1,y1,theta1,x2,y2,theta2):
        if round(math.cos(math.radians(theta1)),4) == 0:
            if round(math.cos(math.radians(theta2)),4) == 0:
                if (x1 == x2):
                    return (None,math.inf)
                else:
                    return None
            else:
                t2 = (x1-x2)/math.cos(math.radians(theta2))
        elif round(math.tan(math.radians(theta1)),4) == 0:
            if round(math.tan(math.radians(theta2)),4) == 0:
                if (y1 == y2):
                    return (math.inf,None)
                else:
                    return None
            else:
                t2 = (y1-y2)/math.sin(math.radians(theta2))
        else:
            if ( (y1-y2) + (x2-x1)*math.tan(math.radians(theta1))) == 0:
                # deal with this case later
                return None
            else:
                t2 = (math.sin(math.radians(theta2)) - math.cos(math.radians(theta2))* math.tan(math.radians(theta1)))/( (y1-y2) + (x2-x1)*math.tan(math.radians(theta1)))
        
        x = x2 + t2*math.cos(math.radians(theta2))
        y = y2 + t2*math.sin(math.radians(theta2))
        return (x,y)

    @staticmethod
    def vehicleDiskGoalAngle(veh_pose,disk_pos,goal_pos):
        veh_pos = (veh_pose.x,veh_pose.y)
        a = BasicGeometry.ptDist(veh_pos,disk_pos)
        b = BasicGeometry.ptDist(disk_pos,goal_pos)
        c = BasicGeometry.ptDist(goal_pos,veh_pos)
        cosC = (a*a + b*b - c*c)/(2*a*b)
        return math.degrees(math.acos(round(cosC,2)))

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
    def lengthOfVectorOfClosestApproach(x1,y1,theta,x2,y2):
        x = x2-x1
        y = y2-y1
        return math.sqrt( (x-math.cos(math.radians(theta))*(math.cos(math.radians(theta))*x+math.sin(math.radians(theta))*y))**2  +  (y-math.sin(math.radians(theta))*(math.cos(math.radians(theta))*x+math.sin(math.radians(theta))*y))**2)


    @staticmethod
    # Find the "closeness" between a disk and a goal in terms of pushability
    # Theta in radians
    def GoalDistanceMetric(x1,y1,theta,x2,y2):
        if math.sin(theta) < 0.25:
            return BasicGeometry.ptDist([x1,y1],[x2,y2])
        else:
            return math.sin(theta) * (BasicGeometry.ptDist([x1,y1],[x2,y2])) ** 2


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

    @staticmethod
    def getPerpLine(line):
        (x1,y1) = line[0]
        (x2,y2) = line[1]
       
        if round((x2-x1),3) == 0:
            return (0,y1,0)
        elif round((y2-y1),3) == 0:
            return (None,0,x1)
        else:
            m1 = (y2-y1)/(x2-x1)
            perp_m = -1/m1
            # (x1,y1) is a point on the perp line
            c = y1 - perp_m*x1
            return (perp_m,c,0)


    @staticmethod
    def twoCirclesIntersectionPoints(circle_1_radius,circle_1_centre,circle_2_radius,circle_2_centre):
        (a,b) = circle_1_centre
        (p,q) = circle_2_centre
        r1 = circle_1_radius
        r2 = circle_2_radius
        D_1 = (r2**2 - r1**2) - (p**2 - a**2) - (q**2 - b**2)
        D_2 = 2*(b-q)
        D_3 = 2*(a-p)
        if D_2 == 0:
            x1 = D_1/D_3
            desc = r1**2 - (x1-a)**2
            if desc>=0:
                y1 = b + math.sqrt(desc)
                y2 = b - math.sqrt(desc)
                return ((x1,y1),(x1,y2))
            else:
                return ((None,None),(None,None))
        A = D_2**2 + D_3**2
        B = 2*D_2*D_3*a - 2*D_1*D_2 - 2*D_3**2*b
        C = D_1**2 - 2*D_1*D_3*a + D_3**2*(a**2 + b**2 - r1**2)
        desc = B**2 - 4*A*C
        if desc >= 0:
            y1 = (-B+math.sqrt(desc))/(2*A)
            y2 = (-B-math.sqrt(desc))/(2*A)
            x1 = (D_1-D_2*y1)/(D_3)
            x2 = (D_1-D_2*y2)/(D_3)

            if (x1<0 or y1<0):
                p1 = (None,None)
            else:
                p1 = (x1,y1)
            if (x2<0 or y2<0):
                p2 = (None,None)
            else:
                p2 = (x2,y2)

            return (p1,p2)
        else:
            return ((None,None),(None,None)) #complex roots


   

        