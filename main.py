import pygame
from env import *
from  sensors import *
import env, sensors
import math

running = True
environment = env.buildEnvironment(MapDimensions=(600, 1200))
environment.originalMap = environment.map.copy()
laser = sensors.LaserSensor(200, environment.originalMap, uncertainty= (0.5, .01))
# environment.map.fill((0, 0, 0))
environment.infomap = environment.map.copy()

# for i in range(1200):
#     for k in range(600):
#         color = environment.map.get_at((i, k))
#         if (color[0], color[1], color[2]) == (0, 0, 0):
#             environment.map.set_at((i, k), (0, 255, 255))





  
while running:
    environment.infomap.fill((0, 0, 0))
    sensorON = False
    for event  in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if pygame.mouse.get_focused():
            sensorON = True
        elif not pygame.mouse.get_focused():
            sensorON = False
    if sensorON:
        position = pygame.mouse.get_pos()
        print(*position)
        environment.map.set_at(position, (0, 0, 0))
        radius =  laser.get_search_radius(position = position)
        
        # sort of debugging code that lights up th
        for  i in radius:
            print(int(i[0]), int(i[1]))
            environment.map.set_at((int(i[0]), int(i[1])), (0, 255, 0))
            points = laser.get_interpolation_points(position, i[0], i[1])
            for k in points:
                environment.map.set_at((int(k[0]), int(k[1])), (255, 0, 0))
                if 0 < k[0] < laser.w and 0< k[1]<laser.h:
                    color = environment.map.get_at((int(k[0]), int(k[1])))
                    if (color[0], color[1], color[2]) == (0, 0, 0):
                        environment.map.set_at((int(k[0]), int(k[1])), (0, 225, 225))
                        break

        
        laser.position = position
        sensor_data = laser.sense_obstacles()
        # print(sensor_data)
        if  not sensor_data == False: 
            for point in sensor_data:
                environment.map.set_at((int(point[0]), int(point[1])), (0, 0, 225))
        environment.show_sensordata()

        environment.map.blit(environment.infomap, (0,0))
        pygame.display.update()


    # pygame.display.update()