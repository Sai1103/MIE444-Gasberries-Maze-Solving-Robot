import pygame
import math
import copy
import original_mcl_helper_Copy as mh
from original_mcl_helper_Copy import Rover, Particle

# Initialize pygame
pygame.init()

# Constants (some are same as mcl_helper.py, will clean up in future)
GRID_WIDTH = 96 + 2 # inches (one extra inch on each side for border)
GRID_HEIGHT = 48 + 2 # inches (one extra inch on each side for border)
ppi = 5
WINDOW_WIDTH = GRID_WIDTH * ppi
WINDOW_HEIGHT = GRID_HEIGHT * ppi
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
MAX_SENSOR_READING = 6000.0 / 127.0
MIN_SENSOR_READING = 3.0 / 2.54

# Simulation parameters
NUM_PARTICLES = mh.NUM_PARTICLES
FORWARD_VELOCITY = 4 * ppi  # inches per second
ANGULAR_VELOCITY = math.radians(120)  # 120 degrees per second
    
    
grid = mh.init_grid()
reduced_grid = copy.deepcopy(grid)
grid = mh.expand_grid(grid, ppi)
valid_positions = mh.create_valid_positions(grid)


# Initialize rover and particles
rover = Rover(grid, valid_positions)
particles = [Particle(grid, valid_positions) for _ in range(NUM_PARTICLES)]
pred_x, pred_y, pred_theta = mh.estimate(particles)

# Initialize window
print(len(grid[0]))
print(len(grid))
window = pygame.display.set_mode((len(grid[0]), len(grid)))
pygame.display.set_caption("Rover Simulation with Edges as Obstacles")

# Main loop
running = True
clock = pygame.time.Clock()

while running:
    window.fill(WHITE)
    mh.draw_grid(window, reduced_grid)

    # User input for rover control
    keys = pygame.key.get_pressed()
    if keys[pygame.K_w]:
        rover.vel_forward = FORWARD_VELOCITY
    elif keys[pygame.K_s]:
        rover.vel_forward = -FORWARD_VELOCITY
    else:
        rover.vel_forward = 0

    if keys[pygame.K_a]:
        rover.vel_angular = -ANGULAR_VELOCITY
    elif keys[pygame.K_d]:
        rover.vel_angular = ANGULAR_VELOCITY
    else:
        rover.vel_angular = 0

    # Move rover and particles
    dt = clock.get_time() / 1000  # Time in seconds
    rover.move(dt, grid)
    mh.update_particles(particles, rover, dt, grid)  # Pass dt to particles

    # Draw particles with orientation lines
    for particle in particles:
        particle_pos = (int(particle.x), int(particle.y))
        pygame.draw.circle(window, RED, particle_pos, ppi / 2)
        mh.draw_orientation(window, particle_pos[0], particle_pos[1], particle.theta, length=ppi * 2, color=RED)

    # Draw rover as a square and its orientation
    rover_pos = (int(rover.x), int(rover.y))
    rover_rect = pygame.Rect(rover_pos[0] - ppi, rover_pos[1] - ppi, 2 * ppi, 2 * ppi)  # Rover as a larger square
    pygame.draw.rect(window, BLACK, rover_rect)  # Draw rover in black
    mh.draw_orientation(window, rover_pos[0], rover_pos[1], rover.theta, length=ppi, color=GREEN)

    # Lidar scan for the rover and draw points
    lidar_distances = rover.lidar_scan(grid)
    particle_lidar_distances = []

    # Update particle weights and resample
    for particle in particles:
        particle_lidar_distances = particle.lidar_scan_fast()
        particle_pos = (int(particle.x), int(particle.y))
        particle.update_weight(particle_lidar_distances, lidar_distances)
    
    if rover.vel_forward != 0 or rover.vel_angular != 0:
        particles, variance = mh.resample_particles(particles, grid, valid_positions, pred_x, pred_y)        
        pred_x, pred_y, pred_theta = mh.estimate(particles)

    # Event handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    pygame.display.flip()
    clock.tick(60)  # 60 FPS
    
pygame.quit()