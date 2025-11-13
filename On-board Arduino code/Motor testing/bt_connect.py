import serial
import time
import sys
import msvcrt

# Replace COM8 with your HC-05's outgoing port
ser = serial.Serial('COM12', 9600, timeout=1)  # 9600 is default HC-05 baud rate
time.sleep(2)  # Give the connection a moment to initialize

print("Connected to HC-05")

# --- Sending data ---
#ser.write(b'Hello from Python!\n')  # b'' sends bytes
#print("Message sent!")
#ser.write(b'Hello from Python!\n')  # b'' sends bytes
#ser.write(b'w')
# print("w Message sent!")
# time.sleep(5)
# ser.write(b'a')
# print("a Message sent!")
# time.sleep(5)
# ser.write(b'd')
# print("d Message sent!")

# --- Reading data back ---
try:
    while True:
        if msvcrt.kbhit():  # check if a key was pressed
            ch = msvcrt.getwch()  # read a single Unicode character
            if ch == '\x1b':  # ESC to quit
                print("\nExiting...")
                break
            ser.write(ch.encode('utf-8'))  # send the character
            print(f"Sent: {ch!r}")
finally:
    ser.close()