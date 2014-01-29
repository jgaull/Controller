void performBluetoothSend(unsigned long now) {
   // manage BLE counter 
   // 50 ms fast, 200 ms med, 1000 ms slow
 
  // check for switch on and CAN rx code indicating ready to transmit
 
 
 if (EnableBluetoothTX)
  {
     if ((now >= bleFastStamp+BLE_FAST_DELTA) || ((now < bleFastStamp) && ((4294967295 - (bleFastStamp + now))>=BLE_FAST_DELTA))){

      writeBLEmsg(0x00, bleFastPointer);
      bleFastStamp = now;
      bleFastPointer++;
      if (bleFastPointer>=6){bleFastPointer=0;}
   }
     if ((now >= bleMedStamp+BLE_MED_DELTA) || ((now < bleMedStamp) && ((4294967295 - (bleMedStamp + now))>=BLE_MED_DELTA))){

      writeBLEmsg(0x00, bleMedPointer);
      bleMedStamp = now;
      bleMedPointer++;
      if (bleMedPointer>=6){bleMedPointer=0;}
   }   
       if ((now >= bleSlowStamp+BLE_SLOW_DELTA) || ((now < bleSlowStamp) && ((4294967295 - (bleSlowStamp + now))>=BLE_SLOW_DELTA))){

      writeBLEmsg(0x00, bleSlowPointer);
      bleSlowStamp = now;
      bleSlowPointer++;
      if (bleSlowPointer>=6){bleSlowPointer=0;}
   }
  
  }
}

void performBluetoothSend1() {
  
  boolean hasSentValue = false;
  for (byte i = 0; i < NUM_SENSORS; i++) {
    if ( sensors[i].state && sensors[i].isFresh ) {
      
      BLEMini.write(sensors[i].dataIdentifier);
      BLEMini.write(sensors[i].value);
      BLEMini.write(sensors[i].value >> 8);
      
      sensors[i].isFresh = false;
      
      hasSentValue = true;
    }
  }
  
  digitalWrite(INDICATOR_LED_PIN, hasSentValue);
}

void writeBLEmsg(byte msgID, byte arrayPointer){
  
  
      BLEMini.write(msgID);
    BLEMini.write(pgm_read_byte(&(bleArray[arrayPointer][1])));
    BLEMini.write(pgm_read_byte(&(bleArray[arrayPointer][1])));
    BLEMini.write(pgm_read_byte(&(bleArray[arrayPointer][2])));
    BLEMini.write(pgm_read_byte(&(bleArray[arrayPointer][3])));
    BLEMini.write(pgm_read_byte(&(bleArray[arrayPointer][4])));
    BLEMini.write(pgm_read_byte(&(bleArray[arrayPointer][5])));
    BLEMini.write(pgm_read_byte(&(bleArray[arrayPointer][6])));
    BLEMini.write(pgm_read_byte(&(bleArray[arrayPointer][7])));
    BLEMini.write(pgm_read_byte(&(bleArray[arrayPointer][8])));
    BLEMini.write(pgm_read_byte(&(bleArray[arrayPointer][9])));
    BLEMini.write(pgm_read_byte(&(bleArray[arrayPointer][10])));
    BLEMini.write(pgm_read_byte(&(bleArray[arrayPointer][11])));
    BLEMini.write(pgm_read_byte(&(bleArray[arrayPointer][12])));
    BLEMini.write(pgm_read_byte(&(bleArray[arrayPointer][13])));
    BLEMini.write(pgm_read_byte(&(bleArray[arrayPointer][14])));
    BLEMini.write(pgm_read_byte(&(bleArray[arrayPointer][15])));    
    BLEMini.write(pgm_read_byte(&(bleArray[arrayPointer][16])));
    BLEMini.write(pgm_read_byte(&(bleArray[arrayPointer][17])));
    BLEMini.write(pgm_read_byte(&(bleArray[arrayPointer][18])));
}




void peformBluetoothReceive() {
  
  if (BLEMini.available() > 0) {
    Serial.print("Available: ");
    Serial.println(BLEMini.available());
  }
  
  while ( BLEMini.available() )
  {
    byte identifier = BLEMini.read();
    byte data1 = BLEMini.read();
    byte data2 = BLEMini.read();
    
    uint16_t value = (data2 << 8) + data1;
    
    if (identifier == SEND_PARAMS_BYTE) {
        performBluetoothSync();
        Serial.println("send params");
        return;
    }
    
    Serial.print("identifier: ");
    Serial.println(identifier);
    Serial.print("value: ");
    Serial.println(value);
    
    //Check to see if the identifier is a sensor
    boolean isSensor = false;
    for(byte i = 0; i < NUM_SENSORS; i++) {
      if( identifier == sensors[i].stateIdentifier ) {
        if (value == 0) {
          sensors[i].state = false;
        }
        else if (value == 1) {
          sensors[i].state = true;
        }
        
        sensors[i].isFresh = false;
        isSensor = true;
        performPropertySync(identifier, value);
        break;
      }
    }
    
    //If it was a sensor then hack around the rest of the function
    if (isSensor) {
      return;
    }
    
    switch(identifier)
    {
      case SMOOTHING_MIN_BYTE:
      
        SMOOTHING_MIN = value;
        
        Serial.print("smoothingMin: ");
        Serial.println(SMOOTHING_MIN);
        break;
        
      case SMOOTHING_MAX_BYTE:
      
        SMOOTHING_MAX = value;
        
        Serial.print("smoothingMax: ");
        Serial.println(SMOOTHING_MAX);
        break;
      
      case MAX_OUTPUT_BYTE:
      
        MAX_OUTPUT = value;
        recalculateStrainDampingMultiplier();
        
        Serial.print("maxOutput: ");
        Serial.println(MAX_OUTPUT);
        break;
        
      case MAX_STRAIN_DAMPING_SPEED_BYTE:
      
        maxStrainDampingSpeed = value;
        recalculateStrainDampingMultiplier();
        
        Serial.print("maxInput: ");
        Serial.println(maxStrainDampingSpeed);
        break;
        
      case STRAIN_DAMPING_CURVE_BYTE:
      
        //STRAIN_DAMPING_CURVE = ((float)value / (float)UINT16_MAX) * 2;
        STRAIN_DAMPING_CURVE = value;
        recalculateStrainDampingMultiplier();
        
        Serial.print("strainDampingCurve: ");
        Serial.println(STRAIN_DAMPING_CURVE);
        break;
      
      case STROKE_TIMEOUT_CYCLES_BYTE:
      
        STROKE_TIMEOUT_CYCLES = value;
        
        Serial.print("strokeTimeoutCycles: ");
        Serial.println(STROKE_TIMEOUT_CYCLES);
        break;
        
      case MAX_EFFORT_BYTE:
      
        MAX_EFFORT = value;
        
        Serial.print("maxEffort: ");
        Serial.println(MAX_EFFORT);
        break;
        
      case TORQUE_MULTIPLIER_BYTE:
      
        torqueMultiplier = value;
        
        Serial.print("torqueMultiplier: ");
        Serial.println(torqueMultiplier);
        break;
        
      default:
        Serial.print("Unknown Identifier: ");
        Serial.print(identifier, HEX);
        Serial.print(", ");
        Serial.println(identifier);
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

void performBluetoothSync() {
  digitalWrite(INDICATOR_LED_PIN, HIGH);
  
  BLEMini.write(SMOOTHING_MIN_BYTE);
  BLEMini.write(SMOOTHING_MIN);
  BLEMini.write(SMOOTHING_MIN >> 8);
  
  BLEMini.write(SMOOTHING_MAX_BYTE);
  BLEMini.write(SMOOTHING_MAX);
  BLEMini.write(SMOOTHING_MAX >> 8);
  
  BLEMini.write(MAX_OUTPUT_BYTE);
  BLEMini.write(MAX_OUTPUT);
  BLEMini.write(MAX_OUTPUT >> 8);
  
  BLEMini.write(MAX_STRAIN_DAMPING_SPEED_BYTE);
  BLEMini.write(maxStrainDampingSpeed);
  BLEMini.write(maxStrainDampingSpeed >> 8);
  
  BLEMini.write(STRAIN_DAMPING_CURVE_BYTE);
  BLEMini.write(STRAIN_DAMPING_CURVE);
  BLEMini.write(STRAIN_DAMPING_CURVE >> 8);
  
  BLEMini.write(STROKE_TIMEOUT_CYCLES_BYTE);
  BLEMini.write(STROKE_TIMEOUT_CYCLES);
  BLEMini.write(STROKE_TIMEOUT_CYCLES >> 8);
  
  BLEMini.write(MAX_EFFORT_BYTE);
  BLEMini.write(MAX_EFFORT);
  BLEMini.write(MAX_EFFORT >> 8);
  
  BLEMini.write(TORQUE_MULTIPLIER_BYTE);
  BLEMini.write(torqueMultiplier);
  BLEMini.write(torqueMultiplier >> 8);
  
  for (byte i = 0; i < NUM_SENSORS; i++) {
    BLEMini.write(sensors[i].stateIdentifier);
    BLEMini.write(sensors[i].state);
    BLEMini.write(sensors[i].state >> 8);
  }
}

void constructBLESensors() {
  sensors[SENSOR_RIDER_EFFORT].dataIdentifier = RIDER_EFFORT_BYTE;
  sensors[SENSOR_RIDER_EFFORT].stateIdentifier = ENABLE_RIDER_EFFORT_UPDATES_BYTE;
  sensors[SENSOR_RIDER_EFFORT].value = 0;
  sensors[SENSOR_RIDER_EFFORT].state = false;
  sensors[SENSOR_RIDER_EFFORT].isFresh = false;
  
  sensors[SENSOR_CURRENT_STRAIN].dataIdentifier = 0x0B;
  sensors[SENSOR_CURRENT_STRAIN].stateIdentifier = ENABLE_CURRENT_STRAIN_UPATES_BYTE;
  sensors[SENSOR_CURRENT_STRAIN].value = 0;
  sensors[SENSOR_CURRENT_STRAIN].state = false;
  sensors[SENSOR_CURRENT_STRAIN].isFresh = false;
  
  sensors[SENSOR_SPEED].dataIdentifier = 0x0C;
  sensors[SENSOR_SPEED].stateIdentifier = ENABLE_SPEED_UPDATES_BYTE;
  sensors[SENSOR_SPEED].value = 0;
  sensors[SENSOR_SPEED].state = false;
  sensors[SENSOR_SPEED].isFresh = false;
  
  sensors[SENSOR_RAW_STRAIN].dataIdentifier = 0x0D;
  sensors[SENSOR_RAW_STRAIN].stateIdentifier = ENABLE_RAW_STRAIN_UPDATES_BYTE;
  sensors[SENSOR_RAW_STRAIN].value = 0;
  sensors[SENSOR_RAW_STRAIN].state = false;
  sensors[SENSOR_RAW_STRAIN].isFresh = false;
  
  sensors[SENSOR_TORQUE_APPLIED].dataIdentifier = 0x0E;
  sensors[SENSOR_TORQUE_APPLIED].stateIdentifier = ENABLE_TORQUE_APPLIED_UPDATES_BYTE;
  sensors[SENSOR_TORQUE_APPLIED].value = 0;
  sensors[SENSOR_TORQUE_APPLIED].state = false;
  sensors[SENSOR_TORQUE_APPLIED].isFresh = false;
  
  sensors[SENSOR_MOTOR_TEMP].dataIdentifier = 0x0F;
  sensors[SENSOR_MOTOR_TEMP].stateIdentifier = ENABLE_MOTOR_TEMP_UPDATES_BYTE;
  sensors[SENSOR_MOTOR_TEMP].value = 0;
  sensors[SENSOR_MOTOR_TEMP].state = false;
  sensors[SENSOR_MOTOR_TEMP].isFresh = false;
  
  sensors[SENSOR_BATTERY_VOLTAGE].dataIdentifier = 0xAA;
  sensors[SENSOR_BATTERY_VOLTAGE].stateIdentifier = ENABLE_BATTERY_VOLTAGE_BYTE;
  sensors[SENSOR_BATTERY_VOLTAGE].value = 0;
  sensors[SENSOR_BATTERY_VOLTAGE].state = false;
  sensors[SENSOR_BATTERY_VOLTAGE].isFresh = false;
}



