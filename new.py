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
    environment.map.blit(environment.externalMap, (0, 0))
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
            position = pygame.mouse.get_pos()
            # pygame.draw.circle(environment.map, (255, 0, 0), position, 5)
            laser.position = position
            sensor_data = sense_obstacles(position, 200, 1200, 600, originalMap, np.array((0.5, 0.01)))
            # print("sensor_data", sensor_data)
            feature_detector.laser_points_set(sensor_data)
            # print("NP", feature_detector.NP)
            # print("laser_points", feature_detector.LASERPOINTS)
            


            while(BREAK_POINT_IND < (feature_detector.NP - feature_detector.PMIN)):
                
                
                flag = True
                seedSegment = False
                feature_detector.NP = max(0, feature_detector.NP)
                # print(feature_detector.NP - feature_detector.PMIN)


                # iterating through the list of laserpoints
                for i in range(BREAK_POINT_IND, (feature_detector.NP - feature_detector.PMIN)):
                    # print("inside loop", i)
                    flag = True
                    predicted_points_to_draw = []
                    j = i + feature_detector.SNUM
                    # print("i", i, "j", j)
                    # print(feature_detector.LASERPOINTS[i:j])
                    m, c = feature_detector.odr_fit(feature_detector.LASERPOINTS[i:j])
                    # print( m , c, "m", "c")
                    params = feature_detector.lineForm_SI2G(m, c)
                    # print("params", params)
                    # testing the point from i to j to see if the i - j segment forms a valid segment. 
                    for k in range(i, j):
                        # break
                        # print(feature_detector.LASERPOINTS[k][0], "Kth point")
                        # print("point to line", feature_detector.point_2line(position, feature_detector.LASERPOINTS[k][0]))

                        predicted_point = feature_detector.predictPoint(params, feature_detector.LASERPOINTS[k][0], position)
                        # print("predicted_point", predicted_point)
                        predicted_points_to_draw.append(predicted_point)
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
                        # results = feature_detector.seed_segment_growing(seedSegment[2], BREAK_POINT_IND)
                        # print("the results are ", results)
                        # print(feature_detector.LASERPOINTS[i], feature_detector.LASERPOINTS[j])

                        # pygame.draw.line(environment.map, (255, 0, 0), feature_detector.LASERPOINTS[i][0], feature_detector.LASERPOINTS[j][0], 5)
                        print(flag)                        
                        break

                    print(flag)                                                    
                break



            # print("printing the seed segments", seedSegment)
            if seedSegment!= False:
                points = seedSegment[0]
                predicted_points_to_draw = seedSegment[1]
                indices = seedSegment[2]

                for point in points:
                    pygame.draw.circle(environment.map, (0, 0, 255), point[0], 5)
                
                    pass
                
                print(points)
                pygame.draw.line(environment.map, (255, 0, 0), points[0][0], points[-1][0], 3)


# strating here 
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
                    print(line_seg)


                    for point in line_seg:
                        environment.map.set_at((int(point[0][0]), int(point[0][1])), (0, 255, 0))
                        pygame.draw.circle(environment.map, COLOR, (int(point[0][0]), int(point[0][1])), 3, 0)
                        # pygame.draw.line(environment.map, (255, 0, 0), ENDPOINTS[0], ENDPOINTS[1], 5 )

#  till here!


            #     if seedSeg == False:
            #         break
            #     else:
            #         seedSegment = seedSeg[0]
            #         # print(seedSegment)


                # PREDICTED_POINTS_TODRAW = seedSeg[1]
                # INDICES = seedSeg[2]
                # results = FeatureMAP.seed_segment_growing(INDICES, BREAK_POINT_IND)
                # if results == False:
                #     BREAK_POINT_IND = INDICES[1]
                #     continue
                # else:
                #     line_eq = results[1]
                #     m, c = results[5]
                #     line_seg = results[0]
                #     OUTERMOST = results[2]
                #     BREAK_POINT_IND = results[3]
                #     ENDPOINTS[0] = FeatureMAP.projection_point2line(OUTERMOST[0], m, c)
                #     ENDPOINTS[1] = FeatureMAP.projection_point2line(OUTERMOST[1], m, c)
                #     COLOR = random_color()
                    
                # print("yes")




            
           


        
        # FeatureMAP.laser_points_set(sensor_data)
        # # print(sensor_data)
        # # print(FeatureMAP.NP, FeatureMAP.LASERPOINTS)

        # while BREAK_POINT_IND < (FeatureMAP.NP - FeatureMAP.PMIN):
        #     seedSeg = FeatureMAP.seed_segment_detection(laser.position, BREAK_POINT_IND)
        #     print(seedSeg)
        #     break



            

    
    pygame.display.update()