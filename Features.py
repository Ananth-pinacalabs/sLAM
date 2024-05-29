import math
import scipy
import numpy as np
from fractions import Fraction
from scipy.odr import * 


Landmarks = []

class featureDetection():
    def __init__(self):
        

        self.EPSILON = 10
        self.DELTA = 501
        self.SNUM = 6
        self.PMIN = 20
        self.GMAX = 15
        self.SEED_SEGMENTS = []
        self.LASER_SEGMENTS = []
        self.LASERPOINTS = []
        self.LINE_PARAMS = None
        self.NP = len(self.LASERPOINTS) - 1
        self.LMIN = 20 # minimum length of line segmemt
        self.LR = 0 # real length of line segment
        self.PR = 0

    def dist_point2point(self, point1, point2):
        px = (point1[0] - point2[0])**2
        py  = (point2[1] - point2[1]) **2
        return math.sqrt(px + py)
    
    def dist_point2line(self, params, point):
        A, B, C = params
        dist  = abs( A * point[0] + B* point[1] + C) / math.sqrt(A**2 + B **2)
        return dist
    
    def line_2points(self, m, b):
        x = 5
        y = m*x +b
        x2 = 2000
        y2 = m*x2 +b
        return [(x, y), (x2, y2)]
    
    # line in general form to slope- intercept form
    def lineForm_G2SI(self, A, B, C):
        m = -A/B
        b = -C/B
        return m, b
    
    def lineForm_SI2G(self, m, b):
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
        
    # find the intersection point btw two lines. 
    # this assumes that  the lines  definitely intersect
    def line_intercept_general(self, params1, params2):
        # print("param1 ", params1, "param 2", params2)
        a1, b1, c1 = params1
        a2, b2, c2 = params2
        x = (c1*b2 - b1*c2)/ (b1*a2 - a1*b2)
        y = (a1*c2 - a2*c1)/ (b1*a2 - a1*b2)
        # print("printing x, y form  line_intercept _general", x, y)
        
        
        return x, y
    
    def point_2line(self,point1, point2):
        m, b = 0, 0
        if point1[0] == point2[0]:
            pass
        else:
            m = (point2[1]  - point1[1])/(point2[0] - point1[0])
            b = point2[1] - m * point2[0]

        
        return m, b

    def projection_point2line(self,point, m, b ):
        x, y = point
        m2 = -1 /m
        c2 = y - m2 * x
        intersection_x = - (b - c2) / (m - m2)
        intersection_y = m2 * intersection_x + c2
        return intersection_x, intersection_y
    
    def AD2pos(self, distance, angle, robot_position):
        x = distance * math.cos(angle) + robot_position[0]
        y = -distance * math.sin(angle) + robot_position[1]
        return (int(x), int(y))

    def laser_points_set(self, data):
        self.LASERPOINTS = []
        if not data:
            pass
        else:
            for point in data:
                coordinates = self.AD2pos(point[0], point[1], point[2])
                self.LASERPOINTS.append([coordinates, point[1]])
        self.NP = len(self.LASERPOINTS) - 1
#  The np variable is the total number of laser points
# Define a function (quadratic in our case to fit to our data)
    def linear_func(self, p, x):
        m, b = p
        return m * x + b

    def odr_fit(self, laser_points):
        x = np.array([i[0][0] for i in laser_points])
        y = np.array([i[0][1] for i in laser_points])
        linear_model = Model(self.linear_func)

        data = RealData(x, y)

        odr_model = ODR(data, linear_model, beta0=[0., 0.])

        out = odr_model.run()
        m, b = out.beta
        return m, b
    
    def predictPoint(self, line_params, sensed_point, robot_pos):
        m, b = self.point_2line(robot_pos, sensed_point)
        # print(m,b)
        params1 = self.lineForm_SI2G(m, b)
        # print("printing  the params  from lineForm_SI2G", params1)
       
        predsx, predsy = self.line_intercept_general(params1, line_params)
        return predsx, predsy 

    def seed_segment_detection(self, robot_position, break_point_ind):
        flag = True
        self.NP = max(0, self.NP)
        
        self.SEED_SEGMENTS = []
        for i in range(break_point_ind, (self.NP - self.PMIN)):
            predicted_points_to_draw = []
            j = i + self.SNUM
            print(self.LASERPOINTS[i:j])            
            m, c = self.odr_fit(self.LASERPOINTS[i:j])
            
            # print(m, c, "m,c from inside the seed detection function")
            params = self.lineForm_SI2G(m, c)
            for k in range(i, j):
                predicted_point = self.predictPoint(params, self.LASERPOINTS[k][0], robot_position)
                predicted_points_to_draw.append(predicted_point)
                d1 = self.dist_point2point(predicted_point, self.LASERPOINTS[k][0])
                if d1 > self.DELTA:
                    flag = False
                    break
                
                d2 = self.dist_point2line(params, self.LASERPOINTS[k][0])

                if d2>self.EPSILON:
                    flag = False
        if flag == True:
            self.LINE_PARAMS = params
            return [self.LASERPOINTS[i:j], predicted_points_to_draw, (i, j)]
        
        return False

    def seed_segment_growing(self, indices, break_point):
        line_eq = self.LINE_PARAMS
        i, j = indices
        PB, PF = max(break_point, i - 1), min(j + 1, len(self.LASERPOINTS) - 1)

        while self.dist_point2line(line_eq, self.LASERPOINTS[PF][0]) < self.EPSILON:
            if PF > self.NP - 1:
                break
            else:
                m, b = self.odr_fit(self.LASERPOINTS[PB:PF])
                line_eq = self.lineForm_SI2G(m, b)
                POINT = self.LASERPOINTS[PF][0]
            PF += 1                
            NEXTPOINT = self.LASERPOINTS[PF][0]
            if self.dist_point2point(POINT, NEXTPOINT) > self.GMAX:
                break

        PF = PF - 1

        while self.dist_point2line(line_eq, self.LASERPOINTS[PB][0]):
            if PB < break_point:
                break
            else:
                m, b = self.odr_fit(self.LASERPOINTS[PB:PF])
                line_eq = self.lineForm_SI2G(m, b)
                POINT = self.LASERPOINTS[PB][0]
                
                
            PB = PB - 1
            NEXTPOINT = self.LASERPOINTS[PB][0]
            if self.dist_point2point(POINT, NEXTPOINT) > self.GMAX:
                break

        PB = PB + 1
        LR = self.dist_point2point(self.LASERPOINTS[PB][0], self.LASERPOINTS[PF][0])
# LR = self.dist_point2point(self.LASERPOINTS[PB][0], self.LASERPOINTS[PF][0])
        PR = len(self.LASERPOINTS[PB:PF])
        if (LR >= self.LMIN) and (PR >= self.PMIN):
            self.LINE_PARAMS = line_eq
            m, b = self.lineForm_G2SI(line_eq[0], line_eq[1], line_eq[2])
            self.two_points = self.line_2points(m, b)
            self.LASER_SEGMENTS.append((self.LASERPOINTS[PB + 1][0], self.LASERPOINTS[PF - 1][0]))
            return [self.LASERPOINTS[PB:PF], self.two_points, (self.LASERPOINTS[PB + 1][0], self.LASERPOINTS[PF - 1][0]), PF, line_eq, (m, b)]
        else:
            return False
