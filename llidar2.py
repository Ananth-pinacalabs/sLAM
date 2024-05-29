import env, sensors, Features
import random
import pygame
import math
import numpy as np


def random_color():
    levels = range(32, 256, 32)
    return tuple(random.choice(levels) for _ in range(3))


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





pygame.init()
image = pygame.image.load("map.png")
pygame.display.set_caption("lidar - simulation")
map = pygame.display.set_mode((1200, 600))
map_copy = map.copy()
map.blit(image, (0, 0))
running = True



FeatureMAP = Features.featureDetection()
environment = env.buildEnvironment((600, 1200))
originalMap = environment.map.copy()

laser = sensors.LaserSensor(200, originalMap, uncertainty=(0.5, 0.01))
environment.map.fill((255, 255, 255))
environment.infomap = environment.map.copy()
originalMap = environment.map.copy()


FEATURE_DETECTION = True
BREAK_POINT_IND = 0


while running:
    environment.infomap = originalMap.copy()
    FEATURE_DETECTION = True
    BREAK_POINT_IND = 0
    ENDPOINTS = [0, 0]
    sensorON = False
    PREDICTED_POINTS_TODRAW = []

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if pygame.mouse.get_focused():
            sensorON = True
        elif not pygame.mouse.get_focused():
            sensorON = False

    if sensorON:
        position = pygame.mouse.get_pos()
        laser.position = position
        sensor_data = laser.sense_obstacles()
        FeatureMAP.laser_points_set(sensor_data)

        while BREAK_POINT_IND < (FeatureMAP.NP - FeatureMAP.PMIN):
            seedSeg = FeatureMAP.seed_segment_detection(laser.position, BREAK_POINT_IND)
            if seedSeg == False:
                break

            else:
                seedSegment = seedSeg[0]
                PREDICTED_POINTS_TODRAW = seedSeg[1]
                INDICES = seedSeg[2]
                results = FeatureMAP.seed_segment_growing(INDICES, BREAK_POINT_IND)
                if results == False:
                    BREAK_POINT_IND = INDICES[1]
                    continue
                else:
                    line_eq = results[1]
                    m, c = results[5]
                    line_seg = results[0]
                    OUTERMOST = results[2]
                    BREAK_POINT_IND = results[3]
                    ENDPOINTS[0] = FeatureMAP.projection_point2line(OUTERMOST[0], m, c)
                    ENDPOINTS[1] = FeatureMAP.projection_point2line(OUTERMOST[1], m, c)
                    COLOR = random_color()

                    for point in line_seg:
                        environment.infomap.set_at((int(point[0][0]), int(point[0][1])), (0, 255, 0))
                        pygame.draw.circle(environment.infomap, COLOR, (int(point[0][0]), int(point[0][1])), 2, 0)
                    pygame.draw.line(environment.infomap, (255, 0, 0), ENDPOINTS[0], ENDPOINTS[1], 2)
                    environment.datastorage(sensor_data)
            environment.map.blit(environment.infomap, (0, 0))
            pygame.display.update()









