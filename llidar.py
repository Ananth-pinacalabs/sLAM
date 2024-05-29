import math 
import pygame
import numpy as np
from helpers import *


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
PR = 0  # number of points in the line segment




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

def laser_points_set(self, data, NP, ):
        LASERPOINTS = []
        if not data:
            pass
        else:
            for point in data:
                coordinates = AD2pos(point[0], point[1], point[2])
                LASERPOINTS.append([coordinates, point[1]])
        NP = len(LASERPOINTS) - 1
        return NP, LASERPOINTS

# some importanat variables
sigma = np.array((0.5, .01))
Range = 200




pygame.init()
image = pygame.image.load("map.png")
pygame.display.set_caption("lidar - simulation")
map = pygame.display.set_mode((1200, 600))
map_copy = map.copy()
map.blit(image, (0, 0))
running = True



while running:
    FEATURE_DETECTION = True
    BREAK_POINT_IND = 0
    ENDPOINTS = [0, 0]
    sensorON = False
    PREDICTED_POINTS_TODRAW = []



    
    map.blit(image, (0, 0))
    events = pygame.event.get()
    for event in events:
        if event.type == pygame.QUIT:
            running = False

    position = pygame.mouse.get_pos()
    # print("outside function", position)

    pygame.draw.circle(map,  (255, 0, 0), position,  20)
    sensor_data =  sense_obstacles(position, Range, 1200, 600, map_copy, sigma)
    LASERPOINTS, NP  = laser_points_set(sensor_data, sensor_data, NP)
    # print("laser_points", LASERPOINTS, NP)

    while BREAK_POINT_IND < (NP - PMIN):
        seedSeg = seed_segment_detection(position, BREAK_POINT_IND)
        if seedSeg == False:
            break

    # print(data)

    pygame.display.update()