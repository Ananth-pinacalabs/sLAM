import pygame
import math
import numpy as np

def uncertainty_add(distance, angle , sigma):
    mean = np.array([distance, angle])
    covarience = np.diag(sigma**2 )
    distance, angle = np.random.multivariate_normal(mean, covarience)
    angle = max(angle, 0)
    return [distance, angle]


class LaserSensor():
    def __init__(self, range, map, uncertainty):
        self.range = range
        self. speed = 4
        self. sigma = np.array((uncertainty[0], uncertainty[1]))
        self.position = (0,0)
        self.map = map
        self.h, self.w = pygame.display.get_surface().get_size()
        self.sensed_obstacles = []

    def distance(self, obstaclePosition):
        px = (self.position[0] - obstaclePosition[0])**2
        py = (self.position[1] - obstaclePosition[1])**2
        return np.sqrt(px + py)
    
    def sense_obstacles(self):
        data = []
        x1, y1  = self.position[0], self.position[1]
        for angle in np.linspace(0, 2*math.pi, 60, False):
            x2, y2  = (x1 + self.range * math.cos(angle), y1 -  self.range *math.sin(angle))
            # print(x2, y2)
            # self.map.set_at((int(x2), int(y2)), (0, 0, 225))
            # smpling point in the with in  the range of the sensor
            #  the loop performs simple interpolation to extract potential points

            #  if the pixel at the point is black them you break  the loop since an obstcle has been found.
            for i in range(0, 100):
                u = i/100
                x = int(x2 * u  + x1 *(1-u))
                y  = int(y2* u + y1 *(1-u))
                if 0 < x < self.w and 0< y<self.h:
                    color = self.map.get_at((x, y))
                    if (color[0], color[1], color[2]) == (0, 0, 0):
                        distance = self.distance((x, y))
                        output = uncertainty_add(distance, angle, self.sigma)
                        output.append(self.position)
                        # store the positions
                        data.append(output)
                        break
            if len(data)> 0:
                # print(data)
                return data


            else:
                return False
        

    def get_interpolation_points(self, position, x2, y2):
        points = []
        x1, y1  = position[0], position[1]
        for i in range(0, 100):
            u = i/100
            x = int(x2 * u  + x1 *(1-u))
            y  = int(y2* u + y1 *(1-u))
            points.append((x, y))
        return points


    def get_search_radius(self, position):
        radius = []
        x1, y1  = position[0], position[1]
        for angle in np.linspace(0, 2*math.pi, 60, False):
            x2, y2  = (x1 + self.range * math.cos(angle), y1 -  self.range *math.sin(angle))
            radius.append((x2, y2))
        return radius

