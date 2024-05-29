import env, sensors, Features
import pygame
import random
import numpy as np
import math
from helper2 import *
from Features import featureDetection



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
        for i in range(0, 100):
            u = i / 100
            x = int(x2 * u + x1 * (1 - u))
            y = int(y2 * u + y1 * (1 - u))
            if 0 < x < W and 0 < y < H:
                color = map.get_at((x, y))
                # print(color)
                if (color[0], color[1], color[2]) == (0, 0, 0):
                    distance = get_distance(position, (x, y))
                    output = uncertainty_add(distance, angle, sigma)
                    output.append(position)
                    data.append(output)
                    break
    return data

def random_color():
    levels = range(32, 256, 32)
    return tuple(random.choice(levels) for _ in range(3))





FeatureMAP = Features.featureDetection()
environment = env.buildEnvironment((600, 1200))
originalMap = environment.map.copy()
# environment.map.fill((255, 255, 255))
environment.infomap = environment.map.copy()


laser = sensors.LaserSensor(200, originalMap, np.array((.5, .01)))
feature_detector = Features.featureDetection()
running = True




while running:
    # refreshing your background. 
    # environment.map.blit(environment.externalMap, (0, 0))
    environment.infomap = originalMap.copy()
    FEATURE_DETECTION = True
    BREAK_POINT_IND = 0
    ENDPOINTS = [0, 0]
    sensorON = False
    PREDICTED_POINTS_TODRAW = [] 

    events = pygame.event.get()
    
    
    for event in events:
        if event.type == pygame.QUIT:
            running = False
            
        if pygame.mouse.get_focused():
            
            # tracking your cursor.
            position = pygame.mouse.get_pos()
            # pygame.draw.circle(environment.map, (255, 0, 0), position, 5)
            laser.position = position
            
            # sensing obstacles. 
            sensor_data = sense_obstacles(position, 200, 1200, 600, originalMap, np.array((0.5, 0.01)))
            # print("sensor_data", sensor_data) # For debugging 
            
            # updating the LASERPOINTS within the class. 
            feature_detector.laser_points_set(sensor_data)
            # print("laser_points", feature_detector.LASERPOINTS) # For debugging
            
            
            
            
            while(BREAK_POINT_IND < (feature_detector.NP - feature_detector.PMIN)):
                flag = True
                seedSegment = False
                feature_detector.NP = max(0, feature_detector.NP)
                
                
                

                
                
                #  in the list of LASERPOINTS we take PMIN points at each interation, and see if this can form a valid segment.
                #  we start assuming that the segment is a seed segment.
                #  fit a line over  the points.
                #  two test
                #       - is the line at > epsilon distance form point.
                #       - is the actual point  at > delta distance from the prediced point.
                
                
                
                # loop to get sets of pmin laserpoints.
                
                
                for i in range(BREAK_POINT_IND, (feature_detector.NP - feature_detector.PMIN)):
                    
                    flag = True
                    predicted_points_to_draw = []
                    j = i + feature_detector.SNUM                    
                    m, c = feature_detector.odr_fit(feature_detector.LASERPOINTS[i:j])
                    params = feature_detector.lineForm_SI2G(m, c)
                    
                    big_gap = False
                    # print("printing the distances.. ")
                    for k in range(i, j-1):
                        dist = get_distance(feature_detector.LASERPOINTS[k][0], feature_detector.LASERPOINTS[k+1][0])
                        if dist > 15:
                            flag = False
                            break 
                    
                    
                    for k in range(i, j):
                        
                        predicted_point = feature_detector.predictPoint(params, feature_detector.LASERPOINTS[k][0], position)
                        predicted_points_to_draw.append(predicted_point)
                        
                        # testing the validity of the segments. 
                        d1 = feature_detector.dist_point2point(predicted_point, feature_detector.LASERPOINTS[k][0])
                                               
                        if d1 > feature_detector.DELTA:
                            flag = False
                            break
                        
                        d2 = feature_detector.dist_point2line(params, feature_detector.LASERPOINTS[k][0])

                        if d2 > feature_detector.EPSILON:
                            flag = False
                            
                    if flag == True:
                        feature_detector.LINE_PARAMS = params                        
                        seedSegment = (feature_detector.LASERPOINTS[i:j], predicted_points_to_draw, (i, j))
                        
                        
                        # debugging here!
                        for point in seedSegment[0]:
                                                        
                            # pygame.draw.circle(environment.map, (0, 0, 255), point[0], 3)
                            pass
                        # pygame.draw.line(environment.map, (255, 0, 0), seedSegment[0][0][0], seedSegment[0][-1][0], 3)
                        break
                    
                    
                    
                    
                if seedSegment!= False:
                    points = seedSegment[0]
                    predicted_points_to_draw = seedSegment[1]
                    indices = seedSegment[2]
                    print("indices: ", indices, "breakpoint", BREAK_POINT_IND)                               
                    results = feature_detector.seed_segment_growing(indices, BREAK_POINT_IND)
                    
                                       
                    print("printing the results", results)          
                    
                    if results!= False:
                        line_eq = results[1] 
                        m, c = results[5]
                        line_seg = results[0]
                        OUTERMOST = results[2]
                        BREAK_POINT_IND = results[3]
                        ENDPOINTS[0] = FeatureMAP.projection_point2line(OUTERMOST[0], m, c)
                        ENDPOINTS[1] = FeatureMAP.projection_point2line(OUTERMOST[1], m, c)
                        COLOR = random_color()
                        # print(line_seg)


                        for point in line_seg:
                            environment.map.set_at((int(point[0][0]), int(point[0][1])), (0, 255, 0))
                            pygame.draw.circle(environment.map, COLOR, (int(point[0][0]), int(point[0][1])), 3, 0)
                            pygame.draw.line(environment.map, (255, 0, 0), ENDPOINTS[0], ENDPOINTS[1], 5 )
                                      
                    
                
                break
                
                        
                    
                
                
                
                
                
                
                
                
                
                
                
                
            
            
            
            
             

    
    pygame.display.update()