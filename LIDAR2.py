import pygame
import numpy as np
from PIL import Image
import math

# Initialize Pygame
pygame.init()

# Constants
WIDTH, HEIGHT = 800, 600
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
POINT_COLOR = (0, 255, 0)
POINT_SIZE = 2
LIDAR_RANGE = 200  # Max range for LIDAR in pixels
NUM_RAYS = 360  # Number of rays to cast

# Load and process maze image
maze_image = Image.open('map.png').convert('L')
maze_data = np.array(maze_image)
height, width = maze_data.shape

# Scale the maze to fit the screen
scale_x = WIDTH / width
scale_y = HEIGHT / height
scale = min(scale_x, scale_y)
maze_data = np.array(maze_image.resize((int(width * scale), int(height * scale))))

# Initialize the screen
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("LIDAR Point Cloud Simulation with TurtleBot")

# Function to detect walls using LIDAR rays
def cast_rays(x, y):
    points = []
    for angle in range(NUM_RAYS):
        radian_angle = math.radians(angle)
        for r in range(LIDAR_RANGE):
            target_x = int(x + r * math.cos(radian_angle))
            target_y = int(y + r * math.sin(radian_angle))
            if target_x < 0 or target_x >= WIDTH or target_y < 0 or target_y >= HEIGHT:
                break
            if maze_data[target_y, target_x] < 128:  # Wall detected
                points.append((target_x, target_y))
                break
    return points

# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Clear the screen
    screen.fill(BLACK)
    pygame.draw.rect(screen, (255, 255, 255), (0,0, 800, 400), 2 )

    # Get the mouse position
    mouse_x, mouse_y = pygame.mouse.get_pos()

    # Generate LIDAR points from the mouse position
    point_cloud = cast_rays(mouse_x, mouse_y)

    # Draw the point cloud
    for point in point_cloud:
        # pygame.draw.circle(screen, POINT_COLOR, point, POINT_SIZE)
        screen.set_at(point, (255, 0, 0))

    # Draw the mouse position
    # pygame.draw.circle(screen, WHITE, (mouse_x, mouse_y), 5)

    # Update the display
    pygame.display.flip()

pygame.quit()
