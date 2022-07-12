// JJROBOTS AHR AIR HOCKEY ROBOT EVO PROJECT

// UDP packets decoding...

uint16_t extractParamInt(uint8_t pos) {
  union {
    unsigned char Buff[2];
    uint16_t d;
  }
  u;

  u.Buff[0] = (unsigned char)SBuffer[pos];
  u.Buff[1] = (unsigned char)SBuffer[pos + 1];
  return (u.d);
}

// Read UDP packets from remote device
// Packet structure:
// Packet 1: Camera packet:
// sync bytes[3]+payload[12]: "mm1"[2bytes] + cam_timestamp[UINT16] + puckCoordX_mm[UINT16] + puckCoordY_mm[UINT16] + robotCoordX_mm[UINT16] + robotCoordY_mm[UINT16] + payload(0xFFFF)[UINT16]
// Packet 2: Set manual robot position packet: (include robot position read from camera)
// sync bytes[3]+payload[12]: "mm2"[2bytes] + target_robot_posX_mm[UINT16] + target_robot_posY_mm[UINT16] + speed[UINT16] + accel[UINT16] + robotCoordX_mm[UINT16] + robotCoordY_mm[UINT16]
// Packet 3: Config packet:
// sync bytes[3]+payload[12]: "mmc"[2bytes] + max_speed[UINT16] + max_accel[UINT16] + robot_defense_position[UINT16] + robot_attack_position[UINT16] + payload(0xFFFF)[UINT16] + payload(0xFFFF)[UINT16]
void packetRead()
{
  unsigned char i;
  uint16_t value;

  if (Serial.available() > 0) {
    // we use direct character manipulation because itÃ‚Â´s fast and we had some problems with arduino String functions...
    // We rotate the Buffer (we could implement a ring buffer in future...)
    for (i = (PACKET_SIZE - 1); i > 0; i--)
      SBuffer[i] = SBuffer[i - 1];
    SBuffer[0] = Serial.read();
    //Serial.print(SBuffer[0]);
    // We look for a  message sync start (mm1, mm2 or mm3)
    if ((SBuffer[2] == 'm') && (SBuffer[1] == 'm'))
    {
      if (readStatus != 0)
        Serial.println("P ERR");
      switch (SBuffer[0]) {
        case '1':
          readStatus = 1;     // PACKET TYPE 1 (camera)
          break;
        case '2':
          readStatus = 2;     // PACKET TYPE 2 (manual)
          break;
        case 'c':
          readStatus = 3;     // PACKET TYPE 3 (config)
          break;
      }
      readCounter = PACKET_SIZE;
      return;
    }

    if (readStatus == 1)
    {
      readCounter--;   // Until we complete the packet
      if (readCounter <= 0) // packet complete!!
      {
        // Extract payload from packet
        cam_timestamp = extractParamInt(10);
        puckOldCoordX = puckCoordX;
        puckOldCoordY = puckCoordY;
        puckCoordX = extractParamInt(8);
        puckCoordY = extractParamInt(6);
        robotCoordX = extractParamInt(4);
        robotCoordY = extractParamInt(2);
        readStatus = 0;
        // Parameters check:
        if ((puckCoordX > TABLE_WIDTH) || (puckCoordY > TABLE_LENGTH) || (robotCoordX > TABLE_WIDTH) || (robotCoordY > ROBOT_CENTER_Y)) {
          //Serial.print("P ERR99!");
          //Serial.print(puckCoordX);
          //Serial.print(",");
          //Serial.print(puckCoordY);
          //Serial.print(";");
          //Serial.print(robotCoordX);
          //Serial.print(",");
          //Serial.println(robotCoordY);
          newPacket = 99; // No valid packet!
          robot_status = 0;
        }
        else
          newPacket = 1;
      }
    }
    else if (readStatus == 2)
    {
      readCounter--;   // Until we complete the packet
      if (readCounter <= 0) // packet complete!!
      {
        // CHECK MAXIMUNS
        user_target_x = constrain(extractParamInt(10), 0, TABLE_WIDTH);
        user_target_y = constrain(extractParamInt(8), 0, TABLE_LENGTH / 2);
        user_target_speed = constrain(extractParamInt(6), MIN_SPEED, MAX_SPEED);
        user_target_accel = constrain(extractParamInt(4), MIN_ACCEL, MAX_ACCEL);
        robotCoordX = extractParamInt(2);
        robotCoordY = extractParamInt(0);
        newPacket = 2;
        readStatus = 0;
      }
    }
    else if (readStatus == 3)
    {
      readCounter--;   // Until we complete the packet
      if (readCounter <= 0) // packet complete!!
      {
        if (extractParamInt(0) != 0xFFFF) // Check final payload (as a quality control)
        {
          //Serial.println("mmc PACKET ERROR!");
          readStatus = 0;
          return;
        }
        // CHECK MAXIMUNS
        user_max_speed = constrain(extractParamInt(10), MIN_SPEED, MAX_SPEED);
        user_max_accel = constrain(extractParamInt(8), MIN_ACCEL, MAX_ACCEL);
        user_robot_defense_position = constrain(extractParamInt(6), ROBOT_DEFENSE_POSITION_MIN, ROBOT_DEFENSE_POSITION_MAX);
        user_robot_defense_attack_position = constrain(extractParamInt(4), ROBOT_DEFENSE_ATTACK_POSITION_MIN, ROBOT_DEFENSE_ATTACK_POSITION_MAX);
        defense_position = user_robot_defense_position;
        attack_position = user_robot_defense_attack_position;
        newPacket = 3;
        readStatus = 0;
      }
    }
  }
}
