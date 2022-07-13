# Minimum y position the robot can go to
min_defense_position = 160
# Maximum y position
max_defense_position = 200
# Start the robot at the minimum y position
y_position = min_defense_position
# increment tracks the direction of y motion. Positive increases y, negative decreases y.
increment = 1



# This example algorithm will follow the puck in the X direction, and oscillate slowly in the Y direction
# The five variables obtained from the vision program in c++ is fed to this function
# The function will tell what the robot to do through the 6 output variables (last line of the function 'returns' the outputs)
example_function(cam_timestamp, puckCoordX, puckCoordY, robotCoordX, robotCoordY)
    
    # y_position and increment needs to be updated in this function and used across different times. Need to be global
    global y_position, increment
    
    # x position simply follows the puck
    target_x_new = puckCoordX
    # set to previously updated y_position
    target_y_new = y_position
    
    # update y_position for the next time
    y_position = y_position + increment
    # change the direction of y motion to smaller y if at maximum defense position
    if y_position >= max_defense_position:
    	increment = -1
    # change the direction of y motion to larger y if at minimum defense position
    if y_position <= min_defense_position:
    	increment = 1
    	
    max_speed = 20000
    max_accel = 150
    
    return target_x_new, target_y_new, max_speed, max_accel, robotCoordX, robotCoordY
    
    
# You do NOT have to change ANY code below, unless you want to name your control algorithm function different from 'example_function'. Then you can change 'example_function' below into your own function.
# The PORT should match port1 or port2 in vision.cpp for communication to be successful.
HOST = "127.0.0.1"  # This is the standard address for describing the device itself (localhost) rather than another host on the internet or other networks
PORT = 65431  # Port to listen on (non-privileged ports are > 1023)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()
    with conn:
        print(f"Connected by {addr}")
        timer_packet_old = 0
        while True:
            data = conn.recv(13)
            if not data:
                break
            bti = lambda x: int.from_bytes(data[x:x+2], 'big', signed=True) # Bytes to integers.
            cam_timestamp, puckCoordX, puckCoordY, robotCoordX, robotCoordY = bti(3), bti(5), bti(7), bti(9), bti(11)
            
            # An example function that uses the input information from the vision system to determine where the robot should go next
            target_x_new, target_y_new, max_speed, max_accel, robotCoordX, robotCoordY = example_function(cam_timestamp, puckCoordX, puckCoordY, robotCoordX, robotCoordY)
            
            data = b'mm2' + itb(target_x_new) + itb(target_y_new) + itb(max_speed) + itb(max_accel) + itb(robotCoordX) + itb(robotCoordY)
  	    conn.sendall(data)
            print("x: {}, y: {}, s: {}, a: {}, rx: {}, ry: {}".format(target_x_new, target_y_new, max_speed, max_accel, robotCoordX, robotCoordY))
