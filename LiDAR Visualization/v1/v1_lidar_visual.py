import pygame
import math
import sys
import random
import lidar_processing_data as lpd
import Movement as mv

'''
This code visualizes LIDAR data collected from an Arduino via Bluetooth.
'''
# --- Setup ---
pygame.init()
WIDTH, HEIGHT = 800, 600
CENTER = (WIDTH // 2, HEIGHT // 2)
window = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("LiDAR World Mapping")

# --- Colors ---
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED   = (255, 0, 0)
BLUE  = (0, 128, 255)

# --- Robot setup ---
rect_w, rect_h = 60, 40
robot_rect = pygame.Rect(0, 0, rect_w, rect_h)
robot_rect.center = CENTER

# --- Simulation parameters ---
SCALE = 0.5       # pixels per mm (tune to fit)
move_speed = 10   # pixels per key press
rotation_speed = 5  # degrees per key press

# --- World state ---
world_points = []       # accumulated lidar map points
robot_x, robot_y = 0, 0 # world coordinates (center)
robot_theta = 0         # robot orientation (degrees)

# --- Simulated LiDAR (replace this with lpd.collect_lidar() if available) ---
def fake_lidar_scan():
    """Simulate a 360° lidar scan with some random walls."""
    points = []
    for angle in range(0, 360, 5):
        # random distance between 100–400 pixels
        dist = 300 + 50 * math.sin(math.radians(angle * 2)) + random.uniform(-10, 10)
        points.append((angle, dist))
    return points


#real_lidar_scan = fake_lidar_scan  # Replace with actual LiDAR function if available
def real_lidar_scan():
    # Placeholder for real LiDAR data collection
    lidar_data = lpd.collect_lidar()
    lidar_data = [(math.radians(angle), distance*2) for angle, distance in lidar_data]
    return lidar_data


# Define which lidar angle should be considered the robot's forward direction.
# By default we make 270° the front of the robot so that a lidar reading
# at angle==270 maps to +x (robot forward in the code's coordinate convention).
FRONT_ANGLE = 90

def polar_to_cartesian(lidar_data):
    points = []
    for angle, distance in lidar_data:
        if distance > 0:
            # Shift incoming lidar angle so FRONT_ANGLE becomes 0° (robot forward)
            adj_angle = (angle - FRONT_ANGLE) % 360
            rad = math.radians(adj_angle)
            x = distance * math.cos(rad)
            y = distance * math.sin(rad)
            points.append((x, y))
    return points

# --- Main loop ---
clock = pygame.time.Clock()
running = True

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

    keys = pygame.key.get_pressed()

    # --- Movement controls (translate world opposite to robot motion) ---
    if keys[pygame.K_w]:
        robot_x += move_speed * math.cos(math.radians(robot_theta))
        robot_y += move_speed * math.sin(math.radians(robot_theta))
        #mv.commands('w')
    if keys[pygame.K_s]:
        robot_x -= move_speed * math.cos(math.radians(robot_theta))
        robot_y -= move_speed * math.sin(math.radians(robot_theta))
        #ser.write(cmd)
        #mv.commands('s')
    if keys[pygame.K_a]:
        robot_theta += rotation_speed
        #mv.commands('a')
    if keys[pygame.K_d]:
        robot_theta -= rotation_speed
        #mv.commands('d')

    # --- Get lidar data (real or simulated) ---
    lidar_data = fake_lidar_scan()
    scan_points = polar_to_cartesian(lidar_data)

    # Convert LiDAR points to world coordinates
    for x, y in scan_points:
        # rotate relative to robot orientation
        rotated_x = x * math.cos(math.radians(robot_theta)) - y * math.sin(math.radians(robot_theta))
        rotated_y = x * math.sin(math.radians(robot_theta)) + y * math.cos(math.radians(robot_theta))
        # translate relative to robot world position
        world_points.append((rotated_x + robot_x, rotated_y + robot_y))

    # --- Draw ---
    window.fill(WHITE)

    # Draw all world points relative to robot center
    for wx, wy in world_points:
        screen_x = CENTER[0] + int((wx - robot_x) * SCALE)
        screen_y = CENTER[1] - int((wy - robot_y) * SCALE)
        pygame.draw.circle(window, RED, (screen_x, screen_y), 2)

    # Draw robot
    pygame.draw.rect(window, BLUE, robot_rect)
    pygame.draw.line(
        window, BLACK, CENTER,
        (CENTER[0] + 40 * math.cos(math.radians(robot_theta)),
         CENTER[1] - 40 * math.sin(math.radians(robot_theta))), 3
    )

    pygame.display.flip()
    clock.tick(30)
