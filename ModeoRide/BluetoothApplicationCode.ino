void performBluetoothSend() {
  
  boolean hasSentValue = false;
  
  for (byte i = 0; i < NUM_SENSORS; i++) {
    
    uint16_t value = properties[sensors[i].propertyAddress].value;
    if (i == SENSOR_FILTERED_RIDER_EFFORT + FIRST_SENSOR_IDENTIFIER) {
      Serial.print("value = ");
      Serial.println(value);
    }
    
    if ( value > 0 && sensors[i].isFresh ) {
      BLEMini.write(sensors[i].dataIdentifier);
      BLEMini.write(sensors[i].value);
      BLEMini.write(sensors[i].value >> 8);
      
      sensors[i].isFresh = false;
      
      hasSentValue = true;
    }
  }
  
  digitalWrite(INDICATOR_LED_PIN, hasSentValue);
}

/*
void performBluetoothReceive() {
  
  if (BLEMini.available() > 0) {
    
    Serial.print("Available: ");
    Serial.println(BLEMini.available());
    
    if ( BLEMini.available() % 3 == 0)
    {
      byte identifier = BLEMini.read();
      byte data1 = BLEMini.read();
      byte data2 = BLEMini.read();
      
      uint16_t value = (data2 << 8) + data1;
      
      Serial.print("Identifier: ");
      Serial.println(identifier);
      Serial.print("Value: ");
      Serial.println(value);
      
      if (identifier >= FIRST_COMMAND_IDENTIFIER) {
        byte commandIdentifier = identifier - FIRST_COMMAND_IDENTIFIER;
        
        if (commandIdentifier == REQUEST_CONNECT) {
          performConnect();
        }
        else if (commandIdentifier == REQUEST_DISCONNECT) {
          performDisconnect();
        }
        
        return;
      }
      
      byte propertyIdentifier = identifier - FIRST_PROPERTY_IDENTIFIER;
      if (properties[propertyIdentifier].value != value) {
        properties[propertyIdentifier].value = value;
        properties[propertyIdentifier].pendingSave = true;
      }
      
      switch(identifier) {
        case PROPERTY_STRAIN_DAMPING_CONTROL1_X:
        case PROPERTY_STRAIN_DAMPING_CONTROL1_Y:
        case PROPERTY_STRAIN_DAMPING_CONTROL2_X:
        case PROPERTY_STRAIN_DAMPING_CONTROL2_Y:
          buildDampingCurve();
          break;
        case PROPERTY_ASSIST_1_X:
        case PROPERTY_ASSIST_1_Y:
        case PROPERTY_ASSIST_2_X:
        case PROPERTY_ASSIST_2_Y:
          buildAssistCurve();
          break;
        case PROPERTY_REGEN_1_X:
        case PROPERTY_REGEN_1_Y:
        case PROPERTY_REGEN_2_X:
        case PROPERTY_REGEN_2_Y:
          buildRegenCurve();
          break;
        case PROPERTY_SENSITIVITY_1_X:
        case PROPERTY_SENSITIVITY_1_Y:
        case PROPERTY_SENSITIVITY_2_X:
        case PROPERTY_SENSITIVITY_2_Y:
          buildSensitivityCurve();
          break;
      }
      
      performPropertySync(identifier, value);
    }
    else {
      clearBLEBuffer();
    }
  }
}
*/

void performBluetoothReceive() {
  if (BLEMini.available() > 0) {
    Serial.print("available: ");
    Serial.println(BLEMini.available());
    
    byte identifier = BLEMini.read();
    byte messageIdentifier;
    
    Serial.print("identifier: ");
    Serial.println(identifier);
    
    if (identifier >= FIRST_BEZIER_IDENTIFIER) {
      messageIdentifier = identifier - FIRST_BEZIER_IDENTIFIER;
      handleBezier(messageIdentifier);
    }
    else if (identifier >= FIRST_COMMAND_IDENTIFIER) {
      messageIdentifier = identifier - FIRST_COMMAND_IDENTIFIER;
      handleCommand(messageIdentifier);
    }
    else if (identifier >= FIRST_EVENT_IDENTIFIER) {
      //This doesn't do anything yet. Maybe not ever.
    }
    else if (identifier >= FIRST_SENSOR_IDENTIFIER) {
      //This doesn't do anything. Proably ever.
    }
    else if (identifier >= FIRST_PROPERTY_IDENTIFIER) {
      messageIdentifier = identifier - FIRST_PROPERTY_IDENTIFIER;
      handleProperty(messageIdentifier);
    }
  }
}

void handleBezier(byte identifier) {
  Bezier bezier;
  byte headerSize = 4;
  byte header[headerSize];
  
  for (int i = 0; i < headerSize; i++) {
    header[i] = BLEMini.read();
    Serial.print("header[");
    Serial.print(i);
    Serial.print("] = ");
    Serial.println(header[i]);
  }
  
  bezier.type = header[0];
  bezier.maxX = header[1];
  bezier.maxY = header[2];
  bezier.numPoints = header[3];
  bezier.cacheIsValid = false;
  
  byte bodySize = header[3] * 2;
  byte body[bodySize];
  
  Serial.print("bodySize = ");
  Serial.println(bodySize);
  
  if (BLEMini.available() >= bodySize) {
    
    for (byte i = 0; i < bodySize; i += 2) {
      byte pointX = BLEMini.read();
      byte pointY = BLEMini.read();
      
      body[i] = pointX;
      body[i + 1] = pointY;
      
      bezier.points[i / 2].x = pointX;
      bezier.points[i / 2].y = pointY;
      
      Serial.print("point[");
      Serial.print(i / 2);
      Serial.print("] = ");
      Serial.print(bezier.points[i / 2].x);
      Serial.print(", ");
      Serial.println(bezier.points[i / 2].y);
    }
    
    switch(bezier.type) {
      case CURVE_TYPE_ASSIST:
        assist = bezier;
        Serial.println("assist");
        break;
      case CURVE_TYPE_DAMPING:
        damping = bezier;
        Serial.println("damping");
        break;
      case CURVE_TYPE_REGEN:
        regen = bezier;
        Serial.println("regen");
        break;
      default:
        Serial.println("Unrecognized Curve Identifier");
    }
    
    byte messageData[headerSize + bodySize];
    for (byte i = 0; i < headerSize; i++) {
      messageData[i] = header[i];
    }
    
    for (byte i = 0; i < bodySize; i++) {
      messageData[i + headerSize] = body[i];
    }
    
    BLEMini.write(identifier + FIRST_BEZIER_IDENTIFIER);
    Serial.print(identifier + FIRST_BEZIER_IDENTIFIER);
    Serial.print(", ");
    for (byte i = 0; i < headerSize + bodySize; i++) {
      BLEMini.write(messageData[i]);
      Serial.print(messageData[i]);
      Serial.print(", ");
    }
    
    Serial.println("");
  }
  else {
    clearBLEBuffer();
  }
}

void handleProperty(byte identifier) {
  if ( BLEMini.available() >= 2) {
    
    byte data1 = BLEMini.read();
    byte data2 = BLEMini.read();
    uint16_t value = (data2 << 8) + data1;
    
    if (properties[identifier].value != value) {
      properties[identifier].value = value;
      properties[identifier].pendingSave = true;
    }
    
    BLEMini.write(identifier + FIRST_PROPERTY_IDENTIFIER);
    BLEMini.write(data1);
    BLEMini.write(data2);
  }
  else {
    clearBLEBuffer();
  }
}

void clearBLEBuffer() {
  while(BLEMini.available() > 0) {
    BLEMini.read();
  }
}

void handleCommand(byte identifier) {
  if (identifier == REQUEST_CONNECT) {
    performConnect();
  }
  else if (identifier == REQUEST_DISCONNECT) {
    performDisconnect();
  }
}

void performSync(byte identifier, byte data[], byte length) {
  digitalWrite(INDICATOR_LED_PIN, HIGH);
  
  BLEMini.write(identifier);
  
  for (int i = 0; i < length; i++) {
    BLEMini.write(data[i]);
  }
  
}

void performConnect() {
  digitalWrite(INDICATOR_LED_PIN, HIGH);
  
  /*
  Serial.println(micros());
  Serial.println("   SYNC START");
  Serial.println(identifier);
  */
  
  stopSensorUpdates();
  
  for (byte i = 0; i < NUM_PROPERTIES; i++) {
    BLEMini.write(i + FIRST_PROPERTY_IDENTIFIER);
    BLEMini.write(properties[i].value);
    BLEMini.write(properties[i].value >> 8);
    Serial.print("i: ");
    Serial.println(i);
    delay(25);
  }
  
  /*
  Serial.println(micros());
  Serial.println("  SYNC END");
  */
}

void performDisconnect() {
  stopSensorUpdates();
  storeCalibrations();
}

void stopSensorUpdates() {
  for (byte i = 0; i < NUM_SENSORS; i++) {
    properties[sensors[i].propertyAddress].value = false; //when we're performing a sync then we reset the sensors.
  }
}

void constructBLESensors() {
  
  for (byte i = 0; i < NUM_SENSORS; i++) {
    sensors[i].dataIdentifier = i + FIRST_SENSOR_IDENTIFIER;
    sensors[i].value = 0;
    sensors[i].isFresh = false;
  }
  
  sensors[SENSOR_RIDER_EFFORT].propertyAddress = PROPERTY_SENSOR_RIDER_EFFORT_STATE;
  sensors[SENSOR_CURRENT_STRAIN].propertyAddress = PROPERTY_SENSOR_CURRENT_STRAIN_STATE;
  sensors[SENSOR_SPEED].propertyAddress = PROPERTY_SENSOR_SPEED_STATE;
  sensors[SENSOR_RAW_STRAIN].propertyAddress = PROPERTY_SENSOR_RAW_STRAIN_STATE;
  sensors[SENSOR_TORQUE_APPLIED].propertyAddress = PROPERTY_SENSOR_TORQUE_APPLIED_STATE;
  sensors[SENSOR_MOTOR_TEMP].propertyAddress = PROPERTY_SENSOR_MOTOR_TEMP_STATE;
  sensors[SENSOR_BATTERY_VOLTAGE].propertyAddress = PROPERTY_SENSOR_BATTERY_VOLTAGE_STATE;
  sensors[SENSOR_POWER_OUTPUT].propertyAddress = PROPERTY_SENSOR_POWER_OUTPUT_STATE;
  sensors[SENSOR_STROKE_LENGTH].propertyAddress = PROPERTY_SENSOR_STROKE_LENGTH_STATE;
  sensors[SENSOR_FILTERED_RIDER_EFFORT].propertyAddress = PROPERTY_SENSOR_FILTERED_RIDER_EFFORT_STATE;
}

void constructBLEProperties() {
  
  for (byte i = 0; i < NUM_PROPERTIES; i++) {
    properties[i].value = 0;
    properties[i].eepromSave = true;
  }

  properties[PROPERTY_SENSOR_RIDER_EFFORT_STATE].eepromSave = false;
  properties[PROPERTY_SENSOR_SPEED_STATE].eepromSave = false;
  properties[PROPERTY_SENSOR_RAW_STRAIN_STATE].eepromSave = false;
  properties[PROPERTY_SENSOR_TORQUE_APPLIED_STATE].eepromSave = false;
  properties[PROPERTY_SENSOR_MOTOR_TEMP_STATE].eepromSave = false;
  properties[PROPERTY_SENSOR_BATTERY_VOLTAGE_STATE].eepromSave = false;
  properties[PROPERTY_SENSOR_POWER_OUTPUT_STATE].eepromSave = false;
  properties[PROPERTY_SENSOR_STROKE_LENGTH_STATE].eepromSave = false;
  properties[PROPERTY_SENSOR_FILTERED_RIDER_EFFORT_STATE].eepromSave = false;
  properties[PROPERTY_SENSOR_CURRENT_STRAIN_STATE].eepromSave = false;
  properties[PROPERTY_TORQUE_MULTIPLIER].eepromSave = false;
}



