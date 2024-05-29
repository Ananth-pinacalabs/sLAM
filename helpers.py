
import numpy as np
import math
import pygame
from scipy.odr import * 
from fractions import Fraction

EPSILON = 10
DELTA = 501
SNUM = 6
PMIN = 20
GMAX = 20
SEED_SEGMENTS = []
LASER_SEGMENTS = []
LASERPOINTS = []
LINE_PARAMS = None
NP = len(LASERPOINTS) - 1
LMIN = 20  # minimum length of line segment
LR = 0  # real length of line segment
PR = 0


def AD2pos(distance, angle, robot_position):
    x = distance * math.cos(angle) + robot_position[0]
    y = -distance * math.sin(angle) + robot_position[1]
    return (int(x), int(y))

def uncertainty_add(distance, angle , sigma):
    mean = np.array([distance, angle])
    covarience = np.diag(sigma**2 )
    distance, angle = np.random.multivariate_normal(mean, covarience)
    angle = max(angle, 0)
    return [distance, angle]

def get_distance(position, obstaclePosition):
        px = (position[0] - obstaclePosition[0])**2
        py = (position[1] - obstaclePosition[1])**2
        return np.sqrt(px + py)

def sense_obstacles(position, Range, W, H, map, sigma):
    data = []
    x1, y1 = position[0], position[1]
    for angle in np.linspace(0, 2*math.pi, 60, False):
        x2, y2 = (x1 + Range * math.cos(angle), y1 - Range * math.sin(angle))
        # map.set_at((int(x2), int(y2)), (255, 0, 0))
        for i in range(0, 100):
            u = i / 100
            x = int(x2 * u + x1 * (1 - u))
            y = int(y2 * u + y1 * (1 - u))
            # map.set_at((int(x), int(y)), (255, 0, 0))
            if 0 < x < W and 0 < y < H:
                color = map.get_at((x, y))
                # print(color)
                if (color[0], color[1], color[2]) == (0, 0, 0):
                    # print("entered loop")
                    distance = get_distance(position, (x, y))
                    # print("position", position, "(x, y) ", (x, y), "distance",  distance)
                    output = uncertainty_add(distance, angle, sigma)
                    # print(output)
                    output.append(position)
                    # store the measurements
                    data.append(output)
                    break
    return data

def laser_points_set(data, NP):
        LASERPOINTS = []
        if not data:
            pass
        else:
            for point in data:
                coordinates = AD2pos(point[0], point[1], point[2])
                LASERPOINTS.append([coordinates, point[1]])
        NP = len(LASERPOINTS) - 1
        return NP, LASERPOINTS



import math 
import numpy as np

# Define constants


def dist_point2point(point1, point2):
    px = (point1[0] - point2[0])**2
    py = (point2[1] - point2[1])**2
    return math.sqrt(px + py)

def dist_point2line(params, point):
    A, B, C = params
    dist = abs(A * point[0] + B * point[1] + C) / math.sqrt(A**2 + B**2)
    return dist

def line_2points(m, b):
    x = 5
    y = m * x + b
    x2 = 2000
    y2 = m * x2 + b
    return [(x, y), (x2, y2)]

def lineForm_G2SI(A, B, C):
    m = -A / B
    b = -C / B
    return m, b

def lineForm_SI2G(m, b):
    A, B, C = -m, 1, -b
    if A > 0:
        A, B, C = -A, -B, -C
        den_a = Fraction(A).limit_denominator(1000).as_integer_ratio()[1]
        den_c = Fraction(C).limit_denominator(1000).as_integer_ratio()[1]
        gcd = np.gcd(den_a, den_c)
        lcm = den_a * den_c / gcd
        A = A * lcm
        B = B * lcm
        C = C * lcm
    return A, B, C

def line_intercept_general(params1, params2):
    a1, b1, c1 = params1
    a2, b2, c2 = params2
    x = (c1 * b2 - b1 * c2) / (b1 * a2 - a1 * b2)
    y = (a1 * c2 - a2 * c1) / (b1 * a2 - a1 * b2)
    return x, y

def point_2line(point1, point2):
    m, b = 0, 0
    if point1[0] == point2[0]:
        pass
    else:
        m = (point2[1] - point1[1]) / (point2[0] - point1[0])
        b = point2[1] - m * point2[0]
    return m, b

def projection_point2line(point, m, b):
    x, y = point
    m2 = -1 / m
    c2 = y - m2 * x
    intersection_x = -(b - c2) / (m - m2)
    intersection_y = m2 * intersection_x + c2
    return intersection_x, intersection_y

def AD2pos(distance, angle, robot_position):
    x = distance * math.cos(angle) + robot_position[0]
    y = -distance * math.sin(angle) + robot_position[1]
    return (int(x), int(y))

def laser_points_set(data):
    LASERPOINTS = []
    if not data:
        pass
    else:
        for point in data:
            coordinates = AD2pos(point[0], point[1], point[2])
            LASERPOINTS.append([coordinates, point[1]])
    NP = len(LASERPOINTS) - 1

# Define a function (quadratic in our case to fit to our data)
def linear_func(p, x):
    m, b = p
    return m * x + b

def odr_fit(laser_points):
    x = np.array([i[0][0] for i in laser_points])
    y = np.array([i[0][1] for i in laser_points])
    linear_model = Model(linear_func)
    data = RealData(x, y)
    odr_model = ODR(data, linear_model, beta0=[0., 0.])
    out = odr_model.run()
    m, b = out.beta
    return m, b

def predictPoint(line_params, sensed_point, robot_pos):
    m, b = point_2line(robot_pos, sensed_point)
    params1 = lineForm_SI2G(m, b)
    predsx, predsy = line_intercept_general(params1, line_params)
    return predsx, predsy 

def seed_segment_detection(robot_position, break_point_ind):
    flag = True
    NP = max(0, NP)
    SEED_SEGMENTS = []
    for i in range(break_point_ind, (NP - PMIN)):
        predicted_points_to_draw = []
        j = 1 + SNUM
        m, c = odr_fit(LASERPOINTS[i:j])
        params = lineForm_SI2G(m, c)
        for k in range(i, j):
            predicted_point = predictPoint(params, LASERPOINTS[k][0], robot_position)
            predicted_points_to_draw.append(predicted_point)
            d1 = dist_point2point(predicted_point, LASERPOINTS[k][0])
            if d1 > DELTA:
                flag = False
                break
            
            d2 = dist_point2line(params, LASERPOINTS[k][0])

            if d2 > EPSILON:
                flag = False
    if flag == True:
        LINE_PARAMS = params
        return [LASERPOINTS[i:j], predicted_points_to_draw, (i, j)]
    
    return False

def seed_segment_growing(indices, break_point):
    line_eq = LINE_PARAMS
    i, j = indices
    PB, PF = max(break_point, i - 1), min(j + 1, len(LASERPOINTS) - 1)

    while dist_point2line(line_eq, LASERPOINTS[PF][0]) < EPSILON:
        if PF > NP - 1:
            break
        else:
            m, b = odr_fit(LASERPOINTS[PB:PF])
            line_eq = lineForm_SI2G(m, b)
            POINT = LASERPOINTS[PF][0]
        PF += 1                
        NEXTPOINT = LASERPOINTS[PF][0]
        if dist_point2point(POINT, NEXTPOINT) > GMAX:
            break

    PF = PF - 1

    while dist_point2line(line_eq, LASERPOINTS[PB][0]):
        if PB < break_point:
            break
        else:
            m, b = odr_fit(LASERPOINTS[PB:PF])
            line_eq = lineForm_SI2G(m,

