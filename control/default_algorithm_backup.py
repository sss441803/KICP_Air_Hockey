import time

# Puck trayectory prediction
# Robot position detection and missing steps check Control
puckCoordX = 0
puckCoordY = 0
puckOldCoordX = 0
puckOldCoordY = 0
puckSpeedX = 0
puckSpeedY = 0
puckOldSpeedX = 0
puckOldSpeedY = 0

defense_position = 160
attack_position = 250

predict_status = -1
predict_bounce = -1
predict_bounce_status = -1
predict_time = 0
predict_time_attack = 0
predict_x_attack = 0
attack_status = -1
attack_pos_x = 0
attack_pos_y = 0
attack_time = 0
predict_x = 0
predict_x_old = 0
predict_y = 0
predict_y_old = 0
puckSpeedXAverage = 0
puckSpeedYAverage = 0

robot_status = 0
robotCoordX = 0
robotCoordY = 0

max_accel = 150
max_speed = 20000

MAX_SPEED = 20000
MAX_ACCEL = 150

# This is the center of the table. All units in milimeters
TABLE_LENGTH = 905
TABLE_WIDTH = 475
ROBOT_CENTER_X = TABLE_WIDTH/2   # Center of robot.
ROBOT_CENTER_Y = TABLE_LENGTH/2

# Absolute Min and Max robot positions in mm (measured from center of robot pusher)
ROBOT_MIN_X = 20
ROBOT_MIN_Y = 160
ROBOT_MAX_X = TABLE_WIDTH-ROBOT_MIN_X
ROBOT_MAX_Y = 400

# PuckSize (puck radio in mm)
PUCK_SIZE = 20

timer_packet_old = 0

def packetRead():
  
  global puckCoordX, puckCoordY, puckOldCoordX, puckOldCoordY, puckSpeedX, puckSpeedY, puckOldSpeedX, puckOldSpeedY, defense_position, attack_position, predict_status, predict_bounce, predict_bounce_status, predict_time, predict_time_attack, predict_x_attack, attack_status, attack_pos_x, attack_pos_y, attack_time, predict_x, predict_x_old, predict_y, predict_y_old, puckSpeedXAverage, puckSpeedYAverage, robot_status,robotCoordX, robotCoordY, max_accel, max_speed
  # Parameters check:
  if ((puckCoordX > TABLE_WIDTH) or (puckCoordY > TABLE_LENGTH) or (robotCoordX > TABLE_WIDTH) or (robotCoordY > ROBOT_CENTER_Y)):
    robot_status = 0

# Trajectory prediction. time in ms
def cameraProcess(time):
  
  global puckCoordX, puckCoordY, puckOldCoordX, puckOldCoordY, puckSpeedX, puckSpeedY, puckOldSpeedX, puckOldSpeedY, defense_position, attack_position, predict_status, predict_bounce, predict_bounce_status, predict_time, predict_time_attack, predict_x_attack, attack_status, attack_pos_x, attack_pos_y, attack_time, predict_x, predict_x_old, predict_y, predict_y_old, puckSpeedXAverage, puckSpeedYAverage, robot_status,robotCoordX, robotCoordY, max_accel, max_speed

  # Speed calculation on each axis
  vectorX = (puckCoordX - puckOldCoordX)
  vectorY = (puckCoordY - puckOldCoordY)
  puckOldCoordX = puckCoordX
  puckOldCoordY = puckCoordY

  puckOldSpeedX = puckSpeedX
  puckOldSpeedY = puckSpeedY
  puckSpeedX = vectorX * 100 / time # speed in dm/ms (we use this units to not overflow the variable)
  puckSpeedY = vectorY * 100 / time

  # Noise detection, if there are a big speeds this should be noise
  if ((puckSpeedX < -1000) or (puckSpeedX > 1000) or (puckSpeedY < -1000) or (puckSpeedY > 1000)):
  
    predict_status = -1
    predict_x_old = -1
    return
  

  #if (predict_status == -1):  # Noise on last reading?
  
  puckSpeedXAverage = puckSpeedX
  puckSpeedYAverage = puckSpeedY
  
  #else:
  
  #  # if there are low accelerations (similar speeds on readings) we apply an average filtering with the previous value...
  #  if (abs(puckSpeedX - puckOldSpeedX) < 50):
  #    puckSpeedXAverage = (puckSpeedX + puckOldSpeedX)/2
  #  else:
  #    puckSpeedXAverage = puckSpeedX
  #  if (abs(puckSpeedY - puckOldSpeedY) < 50):
  #    puckSpeedYAverage = (puckSpeedY + puckOldSpeedY)/2
  #  else:
  #    puckSpeedYAverage = puckSpeedY
  

  # Absolute speed and direction (not needed now and are slow to calculate...)
  #puckSpeed = sqrt(vectorX*vectorX + vectorY*vectorY)*1000.0/time
  #puckDirection = atan2(vectorY,vectorX)

  predict_x_attack = -1

  # It�s time to predict...
  # Based on actual position and move vector we need to know the future...
  # Posible impact? speed Y is negative when the puck is moving to the robot
  if (puckSpeedYAverage < -25):
  
    predict_status = 1
    # Puck is comming...
    # We need to predict the puck position when it reaches our goal Y position = defense_position
    # slope formula: m = (y2-y1)/(x2-x1)
    if (vectorX == 0):  # To avoid division by 0
      slope = 9999999
    else:
      slope = vectorY / vectorX

    # Prediction of the new x position at defense position: x2 = (y2-y1)/m + x1
    predict_y = defense_position + PUCK_SIZE
    predict_x = (predict_y - puckCoordY) / slope
    predict_x += puckCoordX
    # Prediction of the new x position at attack position
    predict_x_attack = ((attack_position + PUCK_SIZE) - puckCoordY) / slope
    predict_x_attack += puckCoordX

    # puck has a bounce with side wall?
    if ((predict_x < PUCK_SIZE) or (predict_x > (TABLE_WIDTH - PUCK_SIZE))):
    
      predict_status = 2
      predict_bounce = 1
      predict_bounce_status = 1
      # We start a new prediction
      # Wich side?
      if (predict_x < PUCK_SIZE):
      
        #Left side. We calculare the impact point
        bounce_x = PUCK_SIZE
      
      else:
      
        #Right side. We calculare the impact point
        bounce_x = (TABLE_WIDTH - PUCK_SIZE)
      
      bounce_y = (bounce_x - puckCoordX) * slope + puckCoordY
      predict_time = (bounce_y - puckCoordY) * 100 / puckSpeedY # time until bouce
      # bounce prediction => slope change  with the bounce, we only need to change the sign, easy!!
      slope = -slope
      predict_y = defense_position + PUCK_SIZE
      predict_x = (predict_y - bounce_y) / slope
      predict_x += bounce_x

      if ((predict_x < PUCK_SIZE) or (predict_x > (TABLE_WIDTH - PUCK_SIZE))): # New bounce with side wall?
      
        # We do nothing then... with two bounces there are small risk of goal...
        predict_x_old = -1
        predict_status = 0
      
      else:
      
        # only one side bounce...
        # If the puckSpeedY has changed a lot this mean that the puck has touch one side
        if (abs(puckSpeedY - puckOldSpeedY) > 50):
        
          # We dont make a new prediction...
          predict_x_old = -1
        
        else:
        
          # average of the results (some noise filtering)
          if (predict_x_old != -1):
            predict_x = (predict_x_old + predict_x) / 2
          predict_x_old = predict_x
          # We introduce a factor (120 instead of 100) to model the bounce (20% loss in speed)(to improcve...)
          predict_time = predict_time + (predict_y - puckCoordY) * 120 / puckSpeedY # in ms
        
      
    
    else:  # No bounce, direct impact
    
      if (predict_bounce_status == 1):  # This is the first direct impact trajectory after a bounce
      
        # We dont predict nothing new...
        predict_bounce_status = 0
      
      else:
      
        # average of the results (some noise filtering)
        if (predict_x_old > 0):
          predict_x = (predict_x_old + predict_x) / 2
        predict_x_old = predict_x

        predict_time = ((defense_position + PUCK_SIZE) - puckCoordY) * 100 / puckSpeedY # in ms
        predict_time_attack = ((attack_position + PUCK_SIZE) - puckCoordY) * 100 / puckSpeedY # in ms
      
    
  
  else: # Puck is moving slowly or to the other side
  
    predict_x_old = -1
    predict_status = 0
    predict_bounce = 0
    predict_bounce_status = 0
  


# Return the predicted position of the puck in predict_time miliseconds
def predictPuckXPosition(predict_time):
  global puckCoordX, puckCoordY, puckOldCoordX, puckOldCoordY, puckSpeedX, puckSpeedY, puckOldSpeedX, puckOldSpeedY, defense_position, attack_position, predict_status, predict_bounce, predict_bounce_status, predict_time_attack, predict_x_attack, attack_status, attack_pos_x, attack_pos_y, attack_time, predict_x, predict_x_old, predict_y, predict_y_old, puckSpeedXAverage, puckSpeedYAverage, robot_status,robotCoordX, robotCoordY, max_accel, max_speed
  return (puckCoordX + puckSpeedXAverage * predict_time / 100)


# Return the predicted position of the puck in predict_time miliseconds
def predictPuckYPosition(predict_time):
  global puckCoordX, puckCoordY, puckOldCoordX, puckOldCoordY, puckSpeedX, puckSpeedY, puckOldSpeedX, puckOldSpeedY, defense_position, attack_position, predict_status, predict_bounce, predict_bounce_status, predict_time_attack, predict_x_attack, attack_status, attack_pos_x, attack_pos_y, attack_time, predict_x, predict_x_old, predict_y, predict_y_old, puckSpeedXAverage, puckSpeedYAverage, robot_status,robotCoordX, robotCoordY, max_accel, max_speed
  return (puckCoordY + puckSpeedYAverage * predict_time / 100)


def constrain(n, minn, maxn):
  return max(min(maxn, n), minn)


def newDataStrategy():

  global puckCoordX, puckCoordY, puckOldCoordX, puckOldCoordY, puckSpeedX, puckSpeedY, puckOldSpeedX, puckOldSpeedY, defense_position, attack_position, predict_status, predict_bounce, predict_bounce_status, predict_time, predict_time_attack, predict_x_attack, attack_status, attack_pos_x, attack_pos_y, attack_time, predict_x, predict_x_old, predict_y, predict_y_old, puckSpeedXAverage, puckSpeedYAverage, robot_status,robotCoordX, robotCoordY, max_accel, max_speed
  # predict_status == 0 => No risk
  # predict_status == 1 => Puck is moving to our field directly
  # predict_status == 2 => Puck is moving to our field with a bounce
  # predict_status == 3 => ?

  # Default
  robot_status = 0   # Going to initial position (defense)

  if (predict_status == 1): # Puck comming?
  
    if (predict_bounce == 0):  # Direct impact?
    
      if ((predict_x > (ROBOT_MIN_X + 50)) and (predict_x < (ROBOT_MAX_X - 50))):
      
        if (puckSpeedYAverage > -250):
          robot_status = 2  # defense+attack
        else:
          robot_status = 1  # Puck too fast => only defense
      
      else:
      
        if (predict_time < 400):
          robot_status = 1 #1  # Defense
        else:
          robot_status = 0
      
    
    else: # Puck come from a bounce?
      if puckSpeedYAverage > -160: # Puck is moving fast?
        robot_status = 2  # Defense+Attack
      else:
        robot_status = 1  # Defense (too fast...)
    

  

  # Prediction with side bound
  if (predict_status == 2):
  
    # Limit movement
    predict_x = constrain(predict_x, ROBOT_CENTER_X - (PUCK_SIZE * 5), ROBOT_CENTER_X + (PUCK_SIZE * 5))
    robot_status = 1   # only defense mode
  

  # If the puck is moving slowly in the robot field we could start an attack
  if ((predict_status == 0) and (puckCoordY < (ROBOT_CENTER_Y - 20)) and (abs(puckSpeedY) < 45)):
  
    robot_status = 3

  #print('predict: ', predict_status)
  


def setPosition_straight(target_x_new, target_y_new):
  global puckCoordX, puckCoordY, puckOldCoordX, puckOldCoordY, puckSpeedX, puckSpeedY, puckOldSpeedX, puckOldSpeedY, defense_position, attack_position, predict_status, predict_bounce, predict_bounce_status, predict_time, predict_time_attack, predict_x_attack, attack_status, attack_pos_x, attack_pos_y, attack_time, predict_x, predict_x_old, predict_y, predict_y_old, puckSpeedXAverage, puckSpeedYAverage, robot_status,robotCoordX, robotCoordY, max_accel, max_speed
  max_accel = MAX_ACCEL
  #itb = lambda x: int(x).to_bytes(2, 'big', signed=True) # Integer to bytes
  #print(int(target_x_new), int(target_y_new), int(max_speed), max_accel, robotCoordX, robotCoordY)
  #target_x_new, target_y_new, max_speed, max_accel, robotCoordX, robotCoordY = 239, 783, 822, 42, 5, 433
  #target_x_new, target_y_new, max_speed, max_accel, robotCoordX, robotCoordY = 237, 160, 13333, 150, 42, 5
  #data = b'mm2' + itb(100) + itb(200) + itb(100) + itb(200) + itb(100)
  #data = b'mm2' + itb(22463+100) + itb(783) + itb(822) + itb(42) + itb(5)
  return target_x_new, target_y_new, max_speed, max_accel, robotCoordX, robotCoordY
  #conn.sendall(data)


# Robot Moves depends directly on robot status
# robot status:
#   0: Go to defense position
#   1: Defense mode (only move on X axis on the defense line)
#   2: Defense + attach mode
#   3: Attack mode
#   4: ?? REMOVE ??
#   5: Manual mode => User send direct commands to robot
def robotStrategy():

  global puckCoordX, puckCoordY, puckOldCoordX, puckOldCoordY, puckSpeedX, puckSpeedY, puckOldSpeedX, puckOldSpeedY, defense_position, attack_position, predict_status, predict_bounce, predict_bounce_status, predict_time, predict_time_attack, predict_x_attack, attack_status, attack_pos_x, attack_pos_y, attack_time, predict_x, predict_x_old, predict_y, predict_y_old, puckSpeedXAverage, puckSpeedYAverage, robot_status,robotCoordX, robotCoordY, max_accel, max_speed, com_pos_x, com_pos_y

  #print("robot: ", robot_status)

  if robot_status == 0:
    # Go to defense position
    com_pos_y = defense_position
    com_pos_x = ROBOT_CENTER_X  #center X axis
    max_speed = (MAX_SPEED / 3) * 2 # Return a bit more slowly...      
    attack_time = 0

  elif robot_status == 1:
    # Defense mode (only move on X axis on the defense line)
    predict_x = constrain(predict_x, (PUCK_SIZE * 3), TABLE_WIDTH - (PUCK_SIZE * 3))  # we leave some space near the borders...
    com_pos_y = defense_position
    com_pos_x = predict_x
    max_speed = MAX_SPEED
    attack_time = 0

  elif robot_status == 2:
    # Defense+attack
    if (predict_time_attack < 150): # If time is less than 150ms we start the attack HACELO DEPENDIENTE DE LA VELOCIDAD?? NO, solo depende de cuanto tardemos desde defensa a ataque...
    
      com_pos_y = attack_position + PUCK_SIZE * 4 # we need some override
      com_pos_x = predict_x_attack
      max_speed = MAX_SPEED
    
    else:      # Defense position
    
      com_pos_y = predict_y
      com_pos_x = predict_x  # predict_x_attack
      attack_time = 0
      max_speed = MAX_SPEED

  elif robot_status == 3:
    # ATTACK MODE
    if (attack_time == 0):
    
      attack_predict_x = predictPuckXPosition(500)
      attack_predict_y = predictPuckYPosition(500)
      if ((attack_predict_x > (PUCK_SIZE * 3)) and (attack_predict_x < (TABLE_WIDTH - (PUCK_SIZE * 3))) and (attack_predict_y > (PUCK_SIZE * 4)) and (attack_predict_y < (ROBOT_CENTER_Y - (PUCK_SIZE * 4)))):
      
        attack_time = time.time()*1000 + 500  # Prepare an attack in 500ms
        attack_pos_x = attack_predict_x  # predict_x
        attack_pos_y = attack_predict_y  # predict_y
        print("AM: {}, {}".format(attack_pos_x, attack_pos_y))
        # Go to pre-attack position
        com_pos_x = attack_pos_x
        com_pos_y = attack_pos_y - PUCK_SIZE * 3
        max_speed = MAX_SPEED / 2          
        attack_status = 1
      
      else:
      
        attack_time = 0  # Continue waiting for the right attack moment...
        attack_status = 0
        # And go to defense position
        com_pos_y = defense_position
        com_pos_x = ROBOT_CENTER_X  #center X axis
        max_speed = (MAX_SPEED / 3) * 2          
        
    else:
    
      if (attack_status == 1):
      
        if ((attack_time - time.time()*1000) < 200):  # less than 200ms to start the attack
        
          # Attack movement
          com_pos_x = predictPuckXPosition(200)
          com_pos_y = predictPuckYPosition(200) + 50

          print("ATTACK: {}, {}".format(com_pos_x, com_pos_y))

          max_speed = MAX_SPEED

          attack_status = 2 # Attacking
        
        else:  # attack_status=1 but itÃƒâ€šÃ‚Â´s no time to attack yet
        
          # Go to pre-attack position
          com_pos_x = attack_pos_x
          com_pos_y = attack_pos_y - PUCK_SIZE * 3
          max_speed = MAX_SPEED / 2            
        
      if (attack_status == 2):
                
        if (time.time()*1000 > (attack_time + 80)): # Attack move is done? => Reset to defense position
        
          print("RESET")
          attack_time = 0
          robot_status = 0
          attack_status = 0

  else:
    # Default : go to defense position
    com_pos_y = defense_position
    com_pos_x = ROBOT_CENTER_X # center
    attack_time = 0



def policy(dt, puckX, puckY, robotX, robotY):
  global puckCoordX, puckCoordY, robotCoordX, robotCoordY, com_pos_x, com_pos_y, max_speed, max_accel, robotCoordX, robotCoordY
  puckCoordX, puckCoordY, robotCoordX, robotCoordY = puckX, puckY, robotX, robotY
	# Puck detection and trayectory prediction
  cameraProcess(dt)
	# Strategy based on puck prediction
  newDataStrategy()
  robotStrategy()
  #print(puckSpeedX)
  #conn.sendall(data)
  # print('Policy: ', com_pos_x, com_pos_y, max_speed, max_accel, robotCoordX, robotCoordY)
  return com_pos_x, com_pos_y, max_speed, max_accel, robotCoordX, robotCoordY
