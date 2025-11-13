"""This code imports the lidar data from the arduino ide
    via the HC-05 Bluetooth Module and creates a matrix 
    that continuosly stores data as it comes in"""

import serial
# from tabulate import tabulate
import time
import numpy as np
import math
# insert the COM port number associated with the HC-05's serial port

def collect_lidar(data=None,i=0):
    timeout = 5  # seconds
    
    # importing the serial data
    ser = serial.Serial('COM10', 115200, timeout=1) #9600 for the HC-05
    time.sleep(2)
    print("Connected to HC-05")
    #ser.write('Hello')
    
    if data is None:
        data = np.zeros((36,2))
    i = 0
    angle_prev = 0
    distance_prev = 0
    last_data_time = time.time()
    
    while True:
        line = ser.readline().decode(errors='ignore').strip()
        print(line)
        if line:
            last_data_time = time.time()
            try:
                angle, distance = map(float, line.split(','))
                # now start collecting the data
                # we can start with this then we can move forward with appending at random angles
                if angle_prev != angle:
                    data[i][0] = math.radians(angle)
                    data[i][1] = distance
                    angle_prev = angle
                    distance_prev = distance
                    i+=1
                elif angle_prev == angle:
                    if distance_prev > distance:
                        data[i-1][1] = distance
                if i == 36:
                    ser.close()
                    return data
            except ValueError:
                continue
        if time.time() - last_data_time > timeout:
            print(f"Timeout after {timeout}s — returning partial data ({i} points).")
            ser.close()
            return data[:i]  # Return whatever was collected so far

def measure_risk(data):
    i = 1
    angle_index = 0
    original_data = data[i-1][1]
    while i < len(data):
        if data[i][1] > original_data:
            original_data = data[i][1]
            angle_index = i
        i+=1
    # return the angle associated with the largest distance
    return math.radians(data[angle_index][0])

print(collect_lidar())
