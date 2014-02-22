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

void performBluetoothReceive() {
  
  /*if (BLEMini.available() > 0) {
    Serial.print("Available: ");
    Serial.println(BLEMini.available());
  }*/
  
  if ( BLEMini.available() == 3 )
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
}

void performPropertySync(byte identifier, uint16_t value) {
  digitalWrite(INDICATOR_LED_PIN, HIGH);
  
  BLEMini.write(identifier);
  BLEMini.write(value);
  BLEMini.write(value >> 8);
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
    delay(50);
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



