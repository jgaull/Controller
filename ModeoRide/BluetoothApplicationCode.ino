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
  while ( BLEMini.available() > 0) {
    
    //Serial.print("available: ");
    //Serial.println(BLEMini.available());
    
    byte identifier = BLEMini.read();
    byte messageIdentifier;
    
    //Serial.print("identifier: ");
    //Serial.println(identifier);
    
    switch(identifier) {
      case REQUEST_CONNECT:
        Serial.println("connect");
        performConnect();
        break;
        
      case REQUEST_DISCONNECT:
        Serial.println("disconnect");
        performDisconnect();
        break;
        
      case REQUEST_GET_PROPERTY_VALUE:
        Serial.println("send property");
        getPropertyValue();
        break;
        
      case REQUEST_SET_PROPERTY_VALUE:
        Serial.println("set property");
        setPropertyValue();
        break;
        
      case REQUEST_ADD_BEZIER:
        Serial.println("bezier");
        addBezier();
        break;
        
      case REQUEST_GET_SENSOR_VALUE:
        Serial.println("get sensor value");
        getSensorValue();
        break;
        
      case REQUEST_WRITE_PROPERTY:
        Serial.println("write property");
        writeProperty();
        break;
        
      default:
        Serial.print("Uknown command: ");
        Serial.println(identifier);
        clearBLEBuffer();
    }
  }
}

void performConnect() {
  stopSensorUpdates();
  BLEMini.write((byte)REQUEST_CONNECT);
  BLEMini.write(NUM_PROPERTIES);
}

void performDisconnect() {
  storeCalibrations();
  stopSensorUpdates();
  BLEMini.write(REQUEST_DISCONNECT);
  BLEMini.write(1);
}

void getPropertyValue() {
  if ( BLEMini.available() >= 1) {
    byte propertyIdentifier = BLEMini.read();
    syncProperty(REQUEST_GET_PROPERTY_VALUE, propertyIdentifier);
  }
  else {
    clearBLEBuffer();
  }
}

void setPropertyValue() {
  if ( BLEMini.available() >= 3) {
    
    byte propertyIdentifier = BLEMini.read();
    byte data1 = BLEMini.read();
    byte data2 = BLEMini.read();
    unsigned short value = (data2 << 8) + data1;
    
    Property newProperty = copyProperty(propertyIdentifier);
    newProperty.value = value;
    
    if (properties[propertyIdentifier].value != newProperty.value) {
      newProperty.pendingSave = true;
    }
    
    propertyPendingSave = newProperty;
    propertyIdentifierForPropertyPendingSave = propertyIdentifier;
    
    BLEMini.write(REQUEST_SET_PROPERTY_VALUE);
    BLEMini.write(propertyIdentifier);
    BLEMini.write(data1);
    BLEMini.write(data2);
  }
  else {
    clearBLEBuffer();
  }
}

void writeProperty() {
  if ( BLEMini.available() >= 1 ) {
    byte propertyIdentifier = BLEMini.read();
    
    boolean success = false;
    if ( propertyIdentifierForPropertyPendingSave == propertyIdentifier) {
      properties[propertyIdentifier].value = propertyPendingSave.value;
      properties[propertyIdentifier].pendingSave = propertyPendingSave.pendingSave;
      success = true;
      
      Serial.print("propertyPendingSave.value: ");
      Serial.println(propertyPendingSave.value);
      Serial.print("propertyPendingSave.pendingSave: ");
      Serial.println(propertyPendingSave.pendingSave);
      Serial.print("propertyIdentifier: ");
      Serial.println(propertyIdentifier);
    }
    else {
      Serial.println("missmatched property id's");
    }
    
    BLEMini.write(REQUEST_WRITE_PROPERTY);
    BLEMini.write(success);
  }
  else {
    clearBLEBuffer();
  }
}

Property copyProperty(byte propertyIdentifier) {
  Property newProperty;
  
  newProperty.eepromSave = properties[propertyIdentifier].eepromSave;
  newProperty.value = properties[propertyIdentifier].value;
  newProperty.pendingSave = properties[propertyIdentifier].pendingSave;
  
  return newProperty;
}

void syncProperty(byte commandIdentifier, byte propertyIdentifier) {
  BLEMini.write(commandIdentifier);
  BLEMini.write(propertyIdentifier);
  BLEMini.write(properties[propertyIdentifier].value);
  BLEMini.write(properties[propertyIdentifier].value >> 8);
}

void getSensorValue() {
  if (BLEMini.available() >= 1) {
    
    byte sensorIdentifier = BLEMini.read();
    
    unsigned short value = sensors[sensorIdentifier].value;
    
    BLEMini.write(REQUEST_GET_SENSOR_VALUE);
    BLEMini.write(sensorIdentifier);
    BLEMini.write(value);
    BLEMini.write(value >> 8);
  }
  else {
    clearBLEBuffer();
  }
}

void addBezier() {
  Bezier bezier;
  byte headerSize = 4;
  byte header[headerSize];
  
  for (int i = 0; i < headerSize; i++) {
    header[i] = BLEMini.read();
  }
  
  bezier.type = header[0];
  bezier.maxX = header[1];
  bezier.maxY = header[2];
  bezier.numPoints = header[3];
  bezier.cacheIsValid = false;
  
  byte bodySize = header[3] * 2;
  byte body[bodySize];
  
  if (BLEMini.available() >= bodySize) {
    
    for (byte i = 0; i < bodySize; i += 2) {
      byte pointX = BLEMini.read();
      byte pointY = BLEMini.read();
      
      body[i] = pointX;
      body[i + 1] = pointY;
      
      bezier.points[i / 2].x = pointX;
      bezier.points[i / 2].y = pointY;
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
    
    BLEMini.write(REQUEST_ADD_BEZIER);
    for (byte i = 0; i < headerSize + bodySize; i++) {
      BLEMini.write(messageData[i]);
    }
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



