# Minimum y position the robot can go to
min_defense_position = 160
# Maximum y position
max_defense_position = 200
# Start the robot at the minimum y position
y_position = min_defense_position
# increment tracks the direction of y motion. Positive increases y, negative decreases y.
increment = 1



# This example algorithm will follow the puck in the X direction
# The five variables obtained from the vision program in c++ is fed to this function
# The function will tell what the robot to do through the 6 output variables (last line of the function 'returns' the outputs)
def example_function(dt, puckCoordX, puckCoordY, robotCoordX, robotCoordY):
    
    # y_position and increment needs to be updated in this function and used across different times. Need to be global
    global y_position, increment
  
    # x position simply follows the puck
    target_x_new = puckCoordX
    target_y_new = max_defense_position
  	
    max_speed = 20000
    max_accel = 150
  
    return target_x_new, target_y_new, max_speed, max_accel, robotCoordX, robotCoordY
  
  
# You do NOT have to change ANY code below, unless you want to name your control algorithm function different from 'example_function'. Then you can change 'example_function' below into your own function.
# The PORT should match port1 or port2 in vision.cpp for communication to be successful.
import numpy as np
import socket
#import serial
from default_algorithm import policy

#print('Test communication with Arduino. If successful, robot should move.')
#ser = serial.Serial('COM4')  # open serial port
itb = lambda x: int(x).to_bytes(2, 'big', signed=True) # Integer to bytes
#data = b'mm2' + itb(100) + itb(300) + itb(1000) + itb(100) + itb(0) + itb(0)
#ser.write(data)

HOST = "127.0.0.1"  # This is the standard address for describing the device itself (localhost) rather than another host on the internet or other networks
PORT = 65432  # Port to listen on (non-privileged ports are > 1023)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()
    with conn:
        print(f"Connected by {addr}")
        timer_packet_old = 0
        while True:
            data = False
            while not data:
                data = conn.recv(13)
            bti = lambda x: int.from_bytes(data[x:x+2], 'big', signed=True) # Bytes to integers.
            cam_timestamp, puckCoordX, puckCoordY, robotCoordX, robotCoordY = bti(3), bti(5), bti(7), bti(9), bti(11)
        
            # An example function that uses the input information from the vision system to determine where the robot should go next
            # target_x_new, target_y_new, max_speed, max_accel, robotCoordX, robotCoordY = example_function(cam_timestamp, puckCoordX, puckCoordY, robotCoordX, robotCoordY)
        
            # Default policy
            dt = cam_timestamp - timer_packet_old
            target_x_new, target_y_new, max_speed, max_accel, robotCoordX, robotCoordY = policy(dt, puckCoordX, puckCoordY, robotCoordX, robotCoordY)
            timer_packet_old = cam_timestamp
        
            data = b'mm2' + itb(target_x_new) + itb(target_y_new) + itb(max_speed) + itb(max_accel) + itb(robotCoordX) + itb(robotCoordY)
            conn.sendall(data)
            #ser.write(data)
            #print("x: {}, y: {}, s: {}, a: {}, rx: {}, ry: {}".format(target_x_new, target_y_new, max_speed, max_accel, robotCoordX, robotCoordY))
