import pygame
import numpy as np
from PIL import Image
import math
import random

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
NUM_PARTICLES = 100  # Number of particles for SLAM

# Load and process maze image
maze_image = Image.open('map.png').convert('L')
maze_data = np.array(maze_image)
original_height, original_width = maze_data.shape

# Scale the maze to fit the screen
scale_x = WIDTH / original_width
scale_y = HEIGHT / original_height
scale = min(scale_x, scale_y)
scaled_width, scaled_height = int(original_width * scale), int(original_height * scale)
maze_data = np.array(maze_image.resize((scaled_width, scaled_height)))

# Initialize the screens
screen1 = pygame.display.set_mode((WIDTH, HEIGHT))
screen2 = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("SLAM with LIDAR Simulation")

# Particle class for SLAM
class Particle:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.weight = 1.0

    def move(self, dx, dy):
        self.x += dx
        self.y += dy

    def set_weight(self, weight):
        self.weight = weight

# Function to detect walls using LIDAR rays
def cast_rays(x, y, map_data):
    points = []
    for angle in range(NUM_RAYS):
        radian_angle = math.radians(angle)
        for r in range(LIDAR_RANGE):
            target_x = int(x + r * math.cos(radian_angle))
            target_y = int(y + r * math.sin(radian_angle))
            if target_x < 0 or target_x >= scaled_width or target_y < 0 or target_y >= scaled_height:
                break
            if map_data[target_y, target_x] < 128:  # Wall detected
                points.append((target_x, target_y))
                break
    return points

# Initialize particles
particles = [Particle(random.randint(0, scaled_width-1), random.randint(0, scaled_height-1)) for _ in range(NUM_PARTICLES)]

# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Clear the screens
    screen1.fill(BLACK)
    screen2.fill(BLACK)

    # Get the mouse position
    mouse_x, mouse_y = pygame.mouse.get_pos()

    # Ensure the mouse position respects the scaled dimensions
    mouse_x = min(max(mouse_x, 0), scaled_width - 1)
    mouse_y = min(max(mouse_y, 0), scaled_height - 1)

    # Move particles and update weights
    for particle in particles:
        dx = random.gauss(0, 1)  # Simulate small random movements
        dy = random.gauss(0, 1)
        particle.move(dx, dy)
        
        # Cast rays from the particle position
        particle_points = cast_rays(particle.x, particle.y, maze_data)
        
        # Calculate weight (simplified version: number of matched points)
        weight = sum(1 for px, py in particle_points if maze_data[py, px] < 128)
        particle.set_weight(weight)

    # Normalize weights
    total_weight = sum(p.weight for p in particles)
    for particle in particles:
        particle.set_weight(particle.weight / total_weight if total_weight > 0 else 1.0 / NUM_PARTICLES)

    # Resample particles based on weights
    new_particles = []
    for _ in range(NUM_PARTICLES):
        chosen_particle = random.choices(particles, weights=[p.weight for p in particles], k=1)[0]
        new_particles.append(Particle(chosen_particle.x, chosen_particle.y))
    particles = new_particles

    # Estimate position
    estimated_x = sum(p.x * p.weight for p in particles)
    estimated_y = sum(p.y * p.weight for p in particles)

    # Generate LIDAR points from the estimated position
    point_cloud = cast_rays(int(estimated_x), int(estimated_y), maze_data)

    # Draw the map and the LIDAR sensor range on screen1
    map_surface = pygame.surfarray.make_surface(np.repeat(maze_data[:, :, np.newaxis], 3, axis=2))
    screen1.blit(pygame.transform.scale(map_surface, (scaled_width, scaled_height)), (0, 0))
    for angle in range(NUM_RAYS):
        radian_angle = math.radians(angle)
        end_x = int(mouse_x + LIDAR_RANGE * math.cos(radian_angle))
        end_y = int(mouse_y + LIDAR_RANGE * math.sin(radian_angle))
        pygame.draw.line(screen1, WHITE, (mouse_x, mouse_y), (end_x, end_y), 1)

    # Draw the mouse position
    pygame.draw.circle(screen1, WHITE, (mouse_x, mouse_y), 5)

    # Draw the LIDAR point cloud on screen2
    for point in point_cloud:
        pygame.draw.circle(screen2, POINT_COLOR, point, POINT_SIZE)

    # Update the displays
    pygame.display.flip()

pygame.quit()
