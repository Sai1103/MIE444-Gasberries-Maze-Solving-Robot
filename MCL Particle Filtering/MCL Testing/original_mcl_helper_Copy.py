import pygame
import random
import math
import numpy as np
from filterpy.monte_carlo import stratified_resample
from pathlib import Path

# Constants
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
NUM_PARTICLES = 5000
FORWARD_VELOCITY = 4 * ppi  # units per second
ANGULAR_VELOCITY = math.radians(120)  # 60 degrees per second

# Noise parameters
MOVEMENT_NOISE = 0.25
MIN_SENSOR_STD = 0.5
MAX_SENSOR_STD = 2.5
certainty = 0

'''
with open('sensor_data_12_ppi_15_beam_angle.pkl', 'rb') as f:
    loaded_sensor_readings = pickle.load(f)
'''
print("Loading lidar lookup table...")
data_file = Path(__file__).parent / "lidar_lookup_v2.npz"
if not data_file.exists():
    raise FileNotFoundError(f"Data file not found: {data_file!s}")
# load the compressed .npz file
with np.load(data_file) as f:
    # adjust key if needed — typically 'lidar_lookup'
    if 'lidar_lookup' in f:
        loaded_sensor_readings = f['lidar_lookup']
    else:
        # fallback if it's the only array in the file
        loaded_sensor_readings = f[list(f.keys())[0]]


def init_grid():
    # Set unmovable space that is not an obstacle
    grid = np.full((GRID_HEIGHT, GRID_WIDTH), 2, dtype=int)
    
    # Set free movable space
    grid[4:46, 4:10] = 0
    grid[4:22, 10:22] = 0
    grid[4:10, 22:40] = 0
    grid[28:40, 28:34] = 0
    grid[4:22, 40:46] = 0
    grid[16:22, 46:64] = 0
    grid[4:46, 64:70] = 0
    grid[40:46, 10:64] = 0
    grid[16:22, 64:88] = 0
    grid[4:46, 88:94] = 0

    # Set borders as obstacles (1)
    grid[0, 0:GRID_WIDTH] = 1
    grid[GRID_HEIGHT - 1, 0:GRID_WIDTH] = 1

    grid[0:GRID_HEIGHT, 0] = 1
    grid[0:GRID_HEIGHT, GRID_WIDTH - 1] = 1

    # Set obstacles to match maze
    grid[25:37, 13:25] = 1
    grid[13:25, 25:37] = 1
    grid[25:37, 37:61] = 1
    grid[1:13, 49:61] = 1
    grid[1:13, 73:85] = 1
    grid[25:49, 73:85] = 1

    return grid


def expand_grid(grid, factor):
    """Expands each cell in the 2D grid into a larger cell of `factor x factor`.
    
    Args:
        grid (np.array): The original 2D numpy array (grid).
        factor (int): The size to expand each cell into (default is 12).
    
    Returns:
        np.array: A new 2D numpy array where each cell from the original grid is expanded.
    """
    # Use numpy's repeat function to expand rows and columns
    expanded_grid = np.repeat(np.repeat(np.array(grid), factor, axis=0), factor, axis=1)
    return expanded_grid


def display_grid(reduced_grid):
    for i in range(len(reduced_grid)):
        for j in range(len(reduced_grid[0])):
            if reduced_grid[i][j] == 0:
                print(reduced_grid[i][j], end=" ")
            elif reduced_grid[i][j] == 1:
                print("-", end=" ")
            else:
                print("*", end=" ")
        print()
        
        
def create_valid_positions(grid):
    valid_positions = []
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j] == 0:
                valid_positions.append([j, i])
    return valid_positions


def draw_grid(window, grid):
    for y in range(len(grid)):
        for x in range(len(grid[0])):
            if grid[y][x] == 1:
                color = BLACK
            elif grid[y][x] == 2:
                color = BLUE
            elif grid[y][x] != 0:
                color = GREEN
            else:
                color = WHITE
            pygame.draw.rect(window, color, (x * ppi, y * ppi, ppi, ppi))


def normal_pdf(x, mean, std_dev):
    """Calculate the PDF of a normal distribution at x given mean and std_dev."""
    coefficient = 1 / (std_dev * np.sqrt(2 * np.pi))
    exponent = np.exp(-0.5 * ((x - mean) / std_dev) ** 2)
    return coefficient * exponent


def particle_variance(particles, weights, pred_x, pred_y):    
    variance_x = np.average([(p.x - pred_x)**2 / ppi**2 for p in particles], weights = weights)
    variance_y = np.average([(p.y - pred_y)**2 / ppi**2 for p in particles], weights = weights)
    
    # Average the variances to get a single spread measure
    return (variance_x + variance_y) / 2

def estimate(particles):
    particles_x = [p.x for p in particles]
    particles_y = [p.y for p in particles]
    particles_theta = [p.theta for p in particles]
    weights = [p.weight for p in particles]
    mean_x = np.average(particles_x, weights = weights)
    mean_y = np.average(particles_y, weights = weights)
    mean_theta = np.average(particles_theta, weights = weights)
    
    return mean_x, mean_y, mean_theta


def normalize_angle(angle):
    return int(angle % 360)


def resample_particles(particles, grid, valid_positions, pred_x, pred_y):
    weights = [p.weight for p in particles]
    total_weight = sum(weights)    
    # Normalize weights
    weights = [w / total_weight for w in weights]
    variance = particle_variance(particles, weights, pred_x, pred_y)
    certainty = ppi**2 * 2 / (variance)
    print(f"Certainty: {min(certainty, 1)}")
    adjusted_num_particles = max(int(NUM_PARTICLES * max(1 - certainty, 0)), 1000)

    # resampling algorithm
    indexes = stratified_resample(weights)
        
    # resample from index
    particles = [particles[i] for i in indexes[:adjusted_num_particles]]
    # print(len(particles))
    
    # Jitter the particles' positions and orientations to keep them close to their original pose
    def jitter_particle(particle):
        # Add a small random noise to position (pose.x, pose.y) and orientation (pose.theta)
        noise_scale = max(1 - certainty, 0)
        new_x = particle.x + np.random.normal(0, 0.5 + 2 * noise_scale)  # Jitter in x-axis
        new_y = particle.y + np.random.normal(0, 0.5 + 2 * noise_scale)  # Jitter in y-axis
        new_theta = particle.theta + np.random.normal(0, 0.1)  # Jitter in orientation
        
        # Create a new particle with a similar pose
        new_particle = Particle(grid, valid_positions)
        if 0 <= int(new_x) < len(grid[0]) and 0 <= int(new_y) < len(grid) and grid[int(new_y)][int(new_x)] == 0:
            new_particle.x = new_x
            new_particle.y = new_y
        else:
            new_particle.x = particle.x
            new_particle.y = particle.y
        new_particle.theta = new_theta
        new_particle.weight = 1.0 / len(particles)  # Reset weights to be equal
        return new_particle
    
    # Apply jitter to resampled particles and also add some random jitter for new exploration
    jittered_particles = [jitter_particle(p) for p in particles]
    
    return jittered_particles, variance


def update_particles(particles, rover, dt, grid):
    """Move particle weights."""
    for particle in particles:
        particle.move(rover.vel_forward, rover.vel_angular, dt, grid)


# Helper function to draw orientation lines
def draw_orientation(window, x, y, theta, length=10, color=BLACK):
    """Draw an orientation line based on the angle theta."""
    end_x = x + length * math.cos(theta)
    end_y = y + length * math.sin(theta)
    pygame.draw.line(window, color, (x, y), (end_x, end_y), 2)


# Rover class
class Rover:
    def __init__(self, grid, valid_positions):
        while True:
            pos = random.choice(valid_positions)
            self.x = pos[0]
            self.y = pos[1]
            if grid[self.y][self.x] == 0:
                break
        self.theta = random.uniform(0, 2 * math.pi)
        self.vel_forward = 0
        self.vel_angular = 0

    def move(self, dt, grid):
        # Apply noise to movement
        forward_noise = random.gauss(0, MOVEMENT_NOISE)
        angular_noise = random.gauss(0, math.radians(2))

        # Calculate proposed new position
        new_x = self.x + (self.vel_forward + forward_noise) * math.cos(self.theta) * dt
        new_y = self.y + (self.vel_forward + forward_noise) * math.sin(self.theta) * dt
        new_theta = self.theta + (self.vel_angular + angular_noise) * dt

        # Check if new position is within drivable area (not in a black/1 area)
        if 0 <= int(new_x) < len(grid[0])  and 0 <= int(new_y) < len(grid)  and grid[int(new_y)][int(new_x)] == 0:
            self.x = new_x
            self.y = new_y
        self.theta = new_theta
    
    def lidar_scan(self, grid):
        """Simulate a lidar scan with a 180-degree beam width for the rover,
        using a 15-degree beam width (fan) for each scan angle."""
        num_scan_angles = 72
        scan_angles = np.linspace(0, 360, num_scan_angles) * (math.pi / 180)  # Main scan angles
        #beam_half_angle = 7.5 * (math.pi / 180)  # Half of 15 degrees in radians
        lidar_distances = np.zeros(num_scan_angles)

        for index, angle in enumerate(scan_angles):
            scan_angle = self.theta + angle
            sensor_x = self.x + 3.0 * ppi * math.cos(scan_angle)
            sensor_y =  self.y + 3.0 * ppi * math.sin(scan_angle)
            
            # Fan out with multiple scan lines within ±7.5 degrees
            num_beams = 1
            #beam_angles = np.linspace(scan_angle - beam_half_angle, scan_angle + beam_half_angle, num_beams)

            # Track the shortest distance found within this beam
            min_distance = MAX_SENSOR_READING
            hit_found = False

            #for beam_angle in beam_angles:
            for dist in np.linspace(0, (MAX_SENSOR_READING) * ppi, 500):  # High-resolution scan range
                scan_x = dist * math.cos(scan_angle) + sensor_x
                scan_y = dist * math.sin(scan_angle) + sensor_y

                # Check for boundaries or obstacles
                if 0 <= int(scan_x) < len(grid[0]) and 0 <= int(scan_y) < len(grid):
                    if grid[int(scan_y)][int(scan_x)] == 1:  # Hit an obstacle
                        # Return the exact pixel point for the lidar
                        boundary_x = int(scan_x)
                        boundary_y = int(scan_y)
                        lidar_distance = math.sqrt((boundary_x - sensor_x)**2 + (boundary_y - sensor_y)**2)

                        # Keep track of the closest hit distance within this beam
                        min_distance = min(min_distance, lidar_distance / float(ppi))
                        hit_found = True
                        break  # Stop scanning further in this direction
                # If no obstacle was found within the beam angle, move to the next one

            # Append the shortest distance found for this scan angle
            if hit_found:
                # lidar_distances.append(max(min_distance, MIN_SENSOR_READING))
                lidar_distances[index] = max(min_distance, MIN_SENSOR_READING)
                # lidar_distances[index] = min_distance
            else:
                # No obstacle found within the beam
                lidar_distances[index] = MAX_SENSOR_READING
                # lidar_distances.append(MAX_SENSOR_READING)

        return lidar_distances   
    
    def lidar_scan_fast(self):
        """Simulate a lidar scan with a 180-degree beam width for the particle,
        using a 15-degree beam width (fan) for each scan angle."""
        # scan_angles = np.linspace(0, 359, 360) * (math.pi / 180)  # Main scan angles
        scan_angles = np.linspace(0, 360, 73)  # Main scan angles
        lidar_distances = []  # List to hold shortest lidar hit distance for each scan angle

        for angle in scan_angles:
            scan_angle = self.theta * (180 / math.pi) + angle
            lidar_distances.append(loaded_sensor_readings[int(self.x)][int(self.y)][normalize_angle(scan_angle)])
        
        return lidar_distances
    

# Particle class
class Particle:
    def __init__(self, grid, valid_positions):
        while True:
            pos = random.choice(valid_positions)
            self.x = pos[0]
            self.y = pos[1]
            if grid[self.y][self.x] == 0:
                break
        self.theta = random.uniform(0, 2 * math.pi)
        self.weight = 1.0 / NUM_PARTICLES

    def move(self, forward_velocity, angular_velocity, dt, grid):
        # Add movement noise
        forward_noise = random.gauss(0, MOVEMENT_NOISE)
        angular_noise = random.gauss(0, math.radians(2))

        # Calculate new position
        new_x = self.x + (forward_velocity + forward_noise) * math.cos(self.theta) * dt
        new_y = self.y + (forward_velocity + forward_noise) * math.sin(self.theta) * dt
        new_theta = self.theta + (angular_velocity + angular_noise) * dt

        # Ensure particle stays within the drivable area
        if 0 <= int(new_x) < len(grid[0])  and 0 <= int(new_y) < len(grid)  and grid[int(new_y)][int(new_x)] == 0:
            self.x = new_x
            self.y = new_y
        self.theta = new_theta
        
    def update_weight(self, lidar_distances, expected_distances):
        """Update particle weight based on the lidar readings."""
        # Calculate how closely the lidar readings match expected points     
        sensor_std = MIN_SENSOR_STD + (MAX_SENSOR_STD - MIN_SENSOR_STD) * max(1 - certainty, 0)
        for lidar, expected in zip(lidar_distances, expected_distances):
            self.weight *= normal_pdf(expected, lidar, sensor_std * ppi)
        self.weight += 1.e-200

    def lidar_scan(self, grid):
        """Simulate 15 degree beam angle"""
        scan_angles = np.linspace(0, 359, 360) * (math.pi / 180)  # Main scan angles
        beam_half_angle = 7.5 * (math.pi / 180)  # Half of 15 degrees in radians
        lidar_distances = []  # List to hold shortest lidar hit distance for each scan angle

        for angle in scan_angles:
            scan_angle = self.theta + angle
            sensor_x = self.x + 3.0 * ppi * math.cos(scan_angle)
            sensor_y =  self.y + 3.0 * ppi * math.sin(scan_angle)
            
            # Fan out with multiple scan lines within Â±12.5 degrees
            beam_angles = np.linspace(scan_angle - beam_half_angle, scan_angle + beam_half_angle, 10)

            # Track the shortest distance found within this beam
            min_distance = MAX_SENSOR_READING
            hit_found = False

            for beam_angle in beam_angles:
                for dist in np.linspace(0, (MAX_SENSOR_READING) * ppi, 500):  # High-resolution scan range
                    scan_x = dist * math.cos(beam_angle) + sensor_x
                    scan_y = dist * math.sin(beam_angle) + sensor_y

                    # Check for boundaries or obstacles
                    if 0 <= int(scan_x) < len(grid[0]) and 0 <= int(scan_y) < len(grid):
                        if grid[int(scan_y)][int(scan_x)] == 1:  # Hit an obstacle
                            # Return the exact pixel point for the lidar
                            boundary_x = int(scan_x)
                            boundary_y = int(scan_y)
                            lidar_distance = math.sqrt((boundary_x - sensor_x)**2 + (boundary_y - sensor_y)**2)

                            # Keep track of the closest hit distance within this beam
                            min_distance = min(min_distance, lidar_distance / float(ppi))
                            hit_found = True
                            break  # Stop scanning further in this direction
                # If no obstacle was found within the beam angle, move to the next one

            # Append the shortest distance found for this scan angle
            if hit_found:
                lidar_distances.append(max(min_distance, MIN_SENSOR_READING))
            else:
                # No obstacle found within the beam; use -1 to indicate max range
                lidar_distances.append(MAX_SENSOR_READING)

        return lidar_distances
    
    
    def lidar_scan_fast(self):
        """Simulate a lidar scan with a 180-degree beam width for the particle,
        using a 15-degree beam width (fan) for each scan angle."""
        # scan_angles = np.linspace(0, 359, 360) * (math.pi / 180)  # Main scan angles
        scan_angles = np.linspace(0, 360, 72)  # Main scan angles
        lidar_distances = []  # List to hold shortest lidar hit distance for each scan angle

        for angle in scan_angles:
            scan_angle = self.theta * (180 / math.pi) + angle
            lidar_distances.append(loaded_sensor_readings[int(self.x)][int(self.y)][normalize_angle(scan_angle)])
        
        return lidar_distances