import pygame
import math
import copy
import numpy as np
import lidar_lookup_helper as mh
from lidar_lookup_helper import Rover

# Initialize pygame
pygame.init()

# Constants (some are same as mcl_helper.py, will clean up in future)
GRID_WIDTH = 96 + 2 # inches (one extra inch on each side for border)
GRID_HEIGHT = 48 + 2 # inches (one extra inch on each side for border)
ppi = 12
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
ANGULAR_VELOCITY = math.radians(1)  # 120 degrees per second

    
grid = mh.init_grid()
reduced_grid = copy.deepcopy(grid)
grid = mh.expand_grid(grid, ppi)
valid_positions = mh.create_valid_positions(grid)
print(len(valid_positions))
# Create a fast membership set of valid positions as tuples (x,y)
valid_position_set = set((p[0], p[1]) for p in valid_positions)


# Initialize rover and particles
rover = Rover(grid, valid_positions)
#particles = [Particle(grid, valid_positions) for _ in range(NUM_PARTICLES)]
#pred_x, pred_y, pred_theta = mh.estimate(particles)

#print(rover.lidar_scan(grid).size)

# Initialize window
print(len(grid[0]))
print(len(grid))

window = pygame.display.set_mode((len(grid[0]), len(grid)))
pygame.display.set_caption("Rover Simulation with Edges as Obstacles")

#angle_table = np.arange(0, 360, 1) * math.pi / 180.0  # 0 to 359 degrees in radians

# Main loop
running = True
clock = pygame.time.Clock()


# === Initialize rover and lookup ===
rover = Rover(grid, valid_positions)
lidar_lookup = np.memmap(
    "lidar_lookup_fast.dat",
    dtype=np.float32, mode="w+",
    shape=(len(grid[0]), len(grid), 360)
)
print(len(valid_positions))

step = 1  # Adjust step size for faster data collection
while running:
    '''
    1. Spawn the rover in a valid position -> do so using the helper function
    2. At each location record the distances of all sensors, save as [x,y,angle of robot = 0-360] where [x][y][angle] = [72 points of distances]
    step by step make it move and record data at all angles
    '''
    for x in range(0, len(grid[0]), step):
        for y in range(0, len(grid), step):
            # Check the candidate (x,y) against the set of valid positions.
            # Note: valid_positions contains lists [x,y], so testing a tuple (x,y)
            # against that list would always fail; use the set of tuples instead.
            if (x, y) not in valid_position_set:
                print(f"Skipping invalid position: ({x}, {y})")
                continue
            rover.x = x
            rover.y = y
            
            # call the scan and store 360 distances into memmap
            scan = rover.lidar_scan(grid)  # expected: list/array length 360
            print(f"Collecting data at position ({x}, {y}) -> scan length: {np.size(scan)}")
            # Ensure scan is a numpy array with correct dtype and length
            scan = np.asarray(scan, dtype=np.float32)
            if scan.size != 360:
                # Defensive: if scan is empty or wrong length, fill with max readings
                print(f"Warning: scan at ({x},{y}) returned size {scan.size}; filling with MAX_SENSOR_READING")
                scan = np.full((360,), float(MAX_SENSOR_READING), dtype=np.float32)
            lidar_lookup[x, y, :] = scan
            
            if y % 10 == 0:
                # Draw rover as a square and its orientation
                rover_pos = (int(rover.x), int(rover.y))
                rover_rect = pygame.Rect(rover_pos[0] - ppi, rover_pos[1] - ppi, 2 * ppi, 2 * ppi)  # Rover as a larger square
                
                mh.draw_grid(window, reduced_grid)
                pygame.draw.rect(window, BLACK, rover_rect)  # Draw rover in black
                mh.draw_orientation(window, rover_pos[0], rover_pos[1], rover.theta, length=ppi, color=GREEN)
                
                pygame.display.flip()
                clock.tick(240)  # 60 FPS
    # Save the lidar lookup table
    import os
    print("Current working directory:", os.getcwd())

    np.savez_compressed("lidar_lookup_v3.npz", lidar_lookup=lidar_lookup)
    
    print("Lidar lookup table saved successfully.")
    
    pygame.quit()
    running = False
    clock.tick(240)  # 60 FPS
                
    
    '''
    for i in range(0, len(valid_positions), 450):
        rover.x = valid_positions[i][0]
        rover.y = valid_positions[i][1]
        
        for posAngle in range(360):
            lidar_lookup[rover.x][rover.y][posAngle, :] = rover.lidar_scan(grid)
            rover.theta = angle_table[posAngle]
            
            print(f"Collecting data at position {i}, angle {posAngle}")
            if posAngle % 135 == 0:
                # Draw rover as a square and its orientation
                rover_pos = (int(rover.x), int(rover.y))
                rover_rect = pygame.Rect(rover_pos[0] - ppi, rover_pos[1] - ppi, 2 * ppi, 2 * ppi)  # Rover as a larger square
                
                mh.draw_grid(window, reduced_grid)
                pygame.draw.rect(window, BLACK, rover_rect)  # Draw rover in black
                mh.draw_orientation(window, rover_pos[0], rover_pos[1], rover.theta, length=ppi, color=GREEN)
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            pygame.display.flip()
            clock.tick(240)  # 60 FPS
            
    # Save the lidar lookup table
    import os
    print("Current working directory:", os.getcwd())
    print("Saving lookup table to:", os.path.abspath("lidar_lookup_fast.npz"))

    np.savez_compressed("lidar_lookup_fast.npz", lidar_lookup=lidar_lookup)
    
    running = False
    pygame.quit()
    '''
    '''
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
    #mh.update_particles(particles, rover, dt, grid)  # Pass dt to particles

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
    # particle_lidar_distances = []

    # Update particle weights and resample
    for particle in particles:
        particle_lidar_distances = particle.lidar_scan(grid)
        particle_pos = (int(particle.x), int(particle.y))
        particle.update_weight(particle_lidar_distances, lidar_distances)
    
    if rover.vel_forward != 0 or rover.vel_angular != 0:
        particles, variance = mh.resample_particles(particles, grid, valid_positions, pred_x, pred_y)        
        pred_x, pred_y, pred_theta = mh.estimate(particles)
    '''