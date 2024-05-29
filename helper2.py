import math
import scipy
import numpy as np
from fractions import Fraction
from scipy.odr import * 

self_EPSILON = 10
self_DELTA = 501
self_SNUM = 6
self_PMIN = 20
self_GMAX = 20
self_SEED_SEGMENTS = []
self_LASER_SEGMENTS = []
self_LASERPOINTS = []
self_LINE_PARAMS = None
self_NP = len(self_LASERPOINTS) - 1
self_LMIN = 20  # minimum length of line segment
self_LR = 0  # real length of line segment
self_PR = 0


#  return the euclidean dist between two points
def dist_point2point( point1, point2):
    px = (point1[0] - point2[0])**2
    py  = (point2[1] - point2[1]) **2
    return math.sqrt(px + py)


# return the  perpendicular distance between a line and a point
def dist_point2line(params, point):
    A, B, C = params
    dist  = abs( A * point[0] + B* point[1] + C) / math.sqrt(A**2 + B **2)
    return dist

#   return two points that lie on a line defined by the  parameter (m, b)
def line_2points( m, b):
    x = 5
    y = m*x +b
    x2 = 2000
    y2 = m*x2 +b
    return [(x, y), (x2, y2)]


# return the line that passes through two points. returns (m, b)
def point_2line(point1, point2):
    m, b = 0, 0
    if point1[0] == point2[0]:
        pass
    else:
        m = (point2[1]  - point1[1])/(point2[0] - point1[0])
        b = point2[1] - m* point2[0]

        return m, b
    
#  convert the line from (Ax + By + C) to (mx + b) form.  
def lineForm_G2SI(A, B, C):
    m = -A/B
    b = -C/B
    return m, b

#  convert the line from (mx + b) to (Ax + By + C) form. 
def lineForm_SI2G(m, b):
    A, B, C = -m, 1, -b
    if A > 0:
        A,B,C = -A, -B, -C
        den_a = Fraction(A).limit_denominator(1000).as_integer_ratio()[1]
        den_c = Fraction(C).limit_denominator(1000).as_integer_ratio()[1]

        gcd = np.gcd(den_a, den_c)
        lcm = den_a *den_c/ gcd

        A = A* lcm
        B = B* lcm
        C = C* lcm
    
    return A, B, C
   
#  returns the x, y position of the point where two lines (of form {Ax + By+ C})
def line_intercept_general(params1, params2):
    print("param1 ", params1, "param 2", params2)
    a1, b1, c1 = params1
    a2, b2, c2 = params2
    x = (c1*b2 - b1*c2)/ (b1*a2 - a1*b2)
    y = (a1*c2 - a2*c1)/ (b1*a2 - a1*b2)
    print("printing x, y form  line_intercept _general", x, y)


# returns the x, y values of the  point given the angle and distance of a sensed point from the robot position. 
def AD2pos(distance, angle, robot_position):
    x = distance * math.cos(angle) + robot_position[0]
    y = -distance * math.sin(angle) + robot_position[1]
    return (int(x), int(y))

#  returns the  list of (x, y) coordinates i.e  LASERPOINTS after performing the AD2POS
def laser_points_set(data):
    self_LASERPOINTS = []
    if not data:
        pass
    else:
        for point in data:
            coordinates = AD2pos(point[0], point[1], point[2])
            self_LASERPOINTS.append([coordinates, point[1]])
            self_NP = len(self_LASERPOINTS) - 1
            print("NP", self_NP)
    return self_LASERPOINTS


def linear_func(p, x):
    m, b = p
    return m * x + b

# fitting a linear model to laser points. 
def odr_fit(laser_points):
    x = np.array([i[0][0] for i in laser_points])
    y = np.array([i[0][1] for i in laser_points])
    linear_model = Model(linear_func)

    data = RealData(x, y)

    odr_model = ODR(data, linear_model, beta0=[0., 0.])

    out = odr_model.run()
    m, b = out.beta
    return m, b


#  utility in project: to get the point  where the line joining the robot position to sensed point to the line fitted using linear model 
def predictPoint(line_params, sensed_point, robot_pos):
        m, b = point_2line(robot_pos, sensed_point)
        params1 = lineForm_SI2G(m, b)       
        predsx, predsy = line_intercept_general(params1, line_params)
        return predsx, predsy


def seed_segment_detection(robot_position, break_point_ind):
        flag = True
        self_NP = max(0, self_NP)
        self_SEED_SEGMENTS = []
        for i in range(break_point_ind, (self_NP - self_PMIN)):
            predicted_points_to_draw = []
            j = 1 + self_SNUM
            m, c = odr_fit(self_LASERPOINTS[i:j])
            print(m, c, "m,c from inside the seed detection function")
            params = lineForm_SI2G(m, c)
            for k in range(i, j):
                predicted_point = predictPoint(params, self_LASERPOINTS[k][0], robot_position)
                predicted_points_to_draw.append(predicted_point)
                d1 = dist_point2point(predicted_point, self_LASERPOINTS[k][0])
                if d1 > self_DELTA:
                    flag = False
                    break
                
                d2 = dist_point2line(params, self_LASERPOINTS[k][0])

                if d2> self_EPSILON:
                    flag = False
        if flag == True:
            self_LINE_PARAMS = params
            return [self_LASERPOINTS[i:j], predicted_points_to_draw, (i, j)]
        
        return False


















    






