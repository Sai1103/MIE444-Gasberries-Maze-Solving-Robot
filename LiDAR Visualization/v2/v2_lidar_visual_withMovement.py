# import serial
# import time
# import math
# import pygame
# import numpy as np
# from pygame.locals import *

# # ==================== INITIALIZATION ====================

# def init_pygame(width=1000, height=1000):
#     pygame.init()
#     window = pygame.display.set_mode((width, height))
#     pygame.display.set_caption("LiDAR Visualization + Motor Control")
#     return window


# # ==================== VISUALIZATION ====================

# def visualize_lidar(window, data_buffer, map_points, scale=100.0):
#     """Draws current + recent LiDAR readings."""
#     window.fill((255, 255, 255))
#     center = (window.get_width() // 2, window.get_height() // 2)

#     # Center cross and dot
#     pygame.draw.line(window, (180, 180, 180), (center[0], 0), (center[0], window.get_height()), 1)
#     pygame.draw.line(window, (180, 180, 180), (0, center[1]), (window.get_width(), center[1]), 1)
#     pygame.draw.circle(window, (255, 0, 0), center, 6)

#     # History (gray)
#     for angle, distance, _ in map_points:
#         if distance <= 0 or distance > 30:
#             continue
#         x = center[0] + distance * math.cos(math.radians(angle)) * scale
#         y = center[1] - distance * math.sin(math.radians(angle)) * scale
#         pygame.draw.circle(window, (160, 160, 160), (int(x), int(y)), 2)

#     # Current readings (green)
#     for angle, distance in data_buffer:
#         if distance <= 0 or distance > 30:
#             continue
#         x = center[0] + distance * math.cos(math.radians(angle)) * scale
#         y = center[1] - distance * math.sin(math.radians(angle)) * scale
#         pygame.draw.circle(window, (0, 200, 0), (int(x), int(y)), 4)

#     pygame.display.flip()


# # ==================== MOTOR COMMANDS ====================

# def send_motor_command(ser, cmd):
#     """
#     Sends a single-character motor command to Arduino via Bluetooth.
#     Commands: w/a/s/d/q/e/x
#     """
#     try:
#         ser.write(cmd.encode())
#         print(f"➡️  Sent command: {cmd}")
#     except Exception as e:
#         print(f"⚠️  Failed to send command '{cmd}': {e}")


# # ==================== MAIN LOOP ====================

# def main():
#     # Adjust COM port as needed
#     ser = serial.Serial('COM10', 115200, timeout=0.05)
#     time.sleep(1)
#     print("✅ Connected to HC-05")

#     window = init_pygame()
#     clock = pygame.time.Clock()

#     num_points = 72
#     data_buffer = np.zeros((num_points, 2))
#     map_points = []  # (angle, distance, timestamp)
#     index = 0
#     MEMORY_DURATION = 2.0  # seconds
#     scale = 10.0

#     running = True
#     while running:
#         now = time.time()

#         # === Handle Events (Quit + Keyboard Motor Commands) ===
#         for event in pygame.event.get():
#             if event.type == QUIT:
#                 running = False
#             elif event.type == KEYDOWN:
#                 key = pygame.key.name(event.key)
#                 if key in ['w', 'a', 's', 'd', 'q', 'e', 'x']:
#                     send_motor_command(ser, key)

#         # === Read LiDAR Data ===
#         while ser.in_waiting:
#             line = ser.readline().decode(errors='ignore').strip()
#             if not line:
#                 continue
#             try:
#                 angle, distance = map(float, line.split(','))
#                 data_buffer[index % num_points] = [angle, distance]
#                 index += 1
#                 map_points.append((angle, distance, now))
#             except ValueError:
#                 continue

#         # Remove points older than 2 s
#         map_points = [(a, d, t) for (a, d, t) in map_points if now - t <= MEMORY_DURATION]

#         # Draw visualization
#         visualize_lidar(window, data_buffer, map_points, scale=scale)

#         clock.tick(240)  # fast update for low latency

#     ser.close()
#     pygame.quit()


# if __name__ == "__main__":
#     main()

import serial
import time
import math
import pygame
import numpy as np
from pygame.locals import *

# ==================== INITIALIZATION ====================

def init_pygame(width=1000, height=1000):
    pygame.init()
    window = pygame.display.set_mode((width, height))
    pygame.display.set_caption("LiDAR Visualization + Motor Control (90° Front)")
    return window

# ==================== VISUALIZATION ====================

def visualize_lidar(window, data_buffer, map_points, scale=100.0):
    """Draws current + recent LiDAR readings with 90° = front."""
    window.fill((255, 255, 255))
    center = (window.get_width() // 2, window.get_height() // 2)

    # Center cross and red dot
    pygame.draw.line(window, (180, 180, 180), (center[0], 0), (center[0], window.get_height()), 1)
    pygame.draw.line(window, (180, 180, 180), (0, center[1]), (window.get_width(), center[1]), 1)
    pygame.draw.circle(window, (255, 0, 0), center, 6)

    # --- Draw memory points (gray) ---
    for angle, distance, _ in map_points:
        if distance <= 0 or distance > 30:
            continue
        # Rotate so 90° faces "up" (front)
        rotated_angle = angle - 90
        x = center[0] + distance * math.cos(math.radians(rotated_angle)) * scale
        y = center[1] - distance * math.sin(math.radians(rotated_angle)) * scale
        pygame.draw.circle(window, (160, 160, 160), (int(x), int(y)), 2)

    # --- Draw current scan (green) ---
    for angle, distance in data_buffer:
        if distance <= 0 or distance > 30:
            continue
        rotated_angle = angle - 90
        x = center[0] + distance * math.cos(math.radians(rotated_angle)) * scale
        y = center[1] - distance * math.sin(math.radians(rotated_angle)) * scale
        pygame.draw.circle(window, (0, 200, 0), (int(x), int(y)), 4)

    pygame.display.flip()


# ==================== MOTOR COMMANDS ====================

def send_motor_command(ser, cmd):
    """Sends a one-character motor command to Arduino via Bluetooth."""
    try:
        ser.write(cmd.encode())
        print(f"➡️  Sent command: {cmd}")
    except Exception as e:
        print(f"⚠️  Failed to send command '{cmd}': {e}")


# ==================== MAIN LOOP ====================

def main():
    ser = serial.Serial('COM10', 115200, timeout=0.05)
    time.sleep(1)
    print("✅ Connected to HC-05")

    window = init_pygame()
    clock = pygame.time.Clock()

    num_points = 72
    data_buffer = np.zeros((num_points, 2))
    map_points = []  # (angle, distance, timestamp)
    index = 0
    MEMORY_DURATION = 2.0  # seconds
    scale = 10.0

    running = True
    while running:
        now = time.time()

        # === Handle Events ===
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
            elif event.type == KEYDOWN:
                key = pygame.key.name(event.key)
                if key in ['w', 'a', 's', 'd', 'q', 'e', 'x']:
                    send_motor_command(ser, key)

        # === Read LiDAR Data ===
        while ser.in_waiting:
            line = ser.readline().decode(errors='ignore').strip()
            if not line:
                continue
            try:
                angle, distance = map(float, line.split(','))
                data_buffer[index % num_points] = [angle, distance]
                index += 1
                map_points.append((angle, distance, now))
            except ValueError:
                continue

        # Remove old memory points (older than 2 s)
        map_points = [(a, d, t) for (a, d, t) in map_points if now - t <= MEMORY_DURATION]

        # Visualize everything
        visualize_lidar(window, data_buffer, map_points, scale=scale)

        clock.tick(240)

    ser.close()
    pygame.quit()


if __name__ == "__main__":
    main()
