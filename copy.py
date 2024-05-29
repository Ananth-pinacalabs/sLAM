import pygame
import math
import numpy as np

"""
some importamt functions 


"""
def AD2pos(distance, angle, robotPosition):
    x = distance * math.cos(angle) +robotPosition[0]
    y = - distance * math.sin(angle) +robotPosition[1]
    return (int(x), int(y))

def get_distance(obstaclePosition):
        px = (position[0] - obstaclePosition[0])**2
        py = (position[1] - obstaclePosition[1])**2
        return np.sqrt(px + py)


def uncertainty_add(distance, angle , sigma):
    mean = np.array([distance, angle])
    covarience = np.diag(sigma**2 )
    distance, angle = np.random.multivariate_normal(mean, covarience)
    angle = max(angle, 0)
    return [distance, angle]




def sense_obstacles(position, map , Range, sigma ):
    w, h = map.get_size()
    data = []
    
  
    x1, y1  = position[0], position[1]
    for angle in np.linspace(0, 2*math.pi, 60, False):
        x2, y2  = (x1 + Range * math.cos(angle), y1 - Range *math.sin(angle))
        # print(x2, y2)
        # self.map.set_at((int(x2), int(y2)), (0, 0, 225))
        # smpling point in the with in  the range of the sensor
        #  the loop performs simple interpolation to extract potential points

        #  if the pixel at the point is black them you break  the loop since an obstcle has been found.
        for i in range(0, 100):
            u = i/100
            x = int(x2 * u  + x1 *(1-u))
            y  = int(y2* u + y1 *(1-u))
            if 0 < x < w and  0 < y < h:
                color = map.get_at((x, y))
                if (color[0], color[1], color[2]) == (0, 0, 0):
                    distance = get_distance((x, y))
                    # print(distance)
                    output = uncertainty_add(distance, angle, sigma)
                    # print(output)
                    output.append(position)

                    obs_position = AD2pos(distance=output[0], angle = output[1], robotPosition= position)
                    # print("hello", obs_position)
                    # store the positions
                    print("data b4", data)
                    data.append(obs_position) 
                    print("data", data)                    
                    break
                # print(data)

    return data       
            

      



pygame.init()
image = pygame.image.load("map.png")
pygame.display.set_caption("name")
map = pygame.display.set_mode((1200, 600))
map.blit(image, (0, 0))


map_copy = map.copy()
map.fill((0,0,0))
running = True


while running:

    map.blit(image, (0, 0))
    events = pygame.event.get()
    for event in events:
        if event.type == pygame.QUIT:
            running = False

    position = pygame.mouse.get_pos()
    
    pygame.draw.rect(map, (255, 0, 0), (*position, 20, 20))


    

    
    # code to sense sense_obstacles

    w, h = map_copy.get_size()
    data = []
    
    Range = 200
    sigma = np.array((0.5, .01))
    x1, y1  = position[0], position[1]
    for angle in np.linspace(0, 2*math.pi, 60, False):
        x2, y2  = (x1 + Range * math.cos(angle), y1 - Range *math.sin(angle))
        map.set_at((int(x2), int(y2)), (0,0,255))
        # print(x2, y2)
        # self.map.set_at((int(x2), int(y2)), (0, 0, 225))
        # smpling point in the with in  the range of the sensor
        #  the loop performs simple interpolation to extract potential points

        #  if the pixel at the point is black them you break  the loop since an obstcle has been found.
        for i in range(0, 100):
            u = i/100
            x = int(x2 * u  + x1 *(1-u))
            y  = int(y2* u + y1 *(1-u))
            map.set_at((x, y), (0, 255, 0))
            if 0 < x < w and  0 < y < h:
                color = map_copy.get_at((x, y))
                if (color[0], color[1], color[2]) == (0, 0, 0):
                    # map.set_at((x, y), (0, 255, 0))
                    distance = get_distance((x, y))
                    # print(distance)
                    output = uncertainty_add(distance, angle, sigma)
                    # print(output)
                    output.append(position)

                    obs_position = AD2pos(distance=output[0], angle = output[1], robotPosition= position)
                    map.set_at(obs_position, (255, 0, 0))
                    # print(obs_position)
                    # print(obs_position)
                    
                    # print("hello", obs_position)
                    # store the positions
                    # print("data b4", data)
                    data.append(obs_position) 
                    # print("data", data)                    
                    break
            # print(data)




            # print(data)


















    # ends here




    
    # data = []


    # sensed_obstacles = sense_obstacles(position = position, 
    #                                    map = map_copy, 
    #                                    Range = 20,
    #                                    sigma =np.array((0.5, .01)))
    



    # # map.set_at(sensed_obstacles[0], (255, 0, 0))




    # print(sensed_obstacles)
    
    # for point in sensed_obstacles:
    #     map.set_at(point, (255, 0, 0))
    # # print(sensed_obstacles)
    

        

    

    pygame.display.update()