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
      
      Serial.println("Sending sensor value");
      
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
    
    if (identifier == SEND_PARAMS_BYTE) {
      for(byte i = 0; i < NUM_SENSORS; i++) {
      sensors[i].state = false;
      }
      performBluetoothSync();
        //Serial.println("send params");
        return;
    }
    
    /*Serial.print("identifier: ");
    Serial.println(identifier);
    Serial.print("value: ");
    Serial.println(value);*/
    
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
        
        Serial.print("setting sensor with state identifier ");
        Serial.print(sensors[i].stateIdentifier, HEX);
        Serial.print(" to value ");
        Serial.println(sensors[i].state);
        
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
    
    for (byte i = 0; i < NUM_PROPERTIES; i++) {
      if (identifier == properties[i].bleIdentifier) {
        properties[i].value = value;
        
        Serial.print("Property: ");
        Serial.println(i);
        Serial.print("Value: ");
        Serial.println(value);
        
        switch(i) {
          case PROPERTY_MAX_OUTPUT:
          case PROPERTY_STRAIN_DAMPING_CURVE:
          case PROPERTY_MAX_STRAIN_DAMPING_SPEED:
            recalculateStrainDampingMultiplier();
            break;
        }
        
        performPropertySync(identifier, value);
        break;
      }
    }
  }
}

void performPropertySync(byte identifier, uint16_t value) {
  digitalWrite(INDICATOR_LED_PIN, HIGH);
  
  BLEMini.write(identifier);
  BLEMini.write(value);
  BLEMini.write(value >> 8);
  Serial.print("   SYNCED: ");  
  Serial.println(identifier);  
}

void performBluetoothSync() {
  digitalWrite(INDICATOR_LED_PIN, HIGH);
  /*Serial.println(micros());
  Serial.println("   SYNC START");
  Serial.println(identifier);*/
  
  for (byte i = 0; i < NUM_PROPERTIES; i++) {
    BLEMini.write(properties[i].bleIdentifier);
    BLEMini.write(properties[i].value);
    BLEMini.write(properties[i].value >> 8);
    
    Serial.print("Sync identifier: ");
    Serial.println(properties[i].bleIdentifier);
    delay(1);
  }
  
  for (byte i = 0; i < NUM_SENSORS; i++) {
    sensors[i].state = false; //when we're performing a sync then we reset the sensors.
    
    BLEMini.write(sensors[i].stateIdentifier);
    BLEMini.write(sensors[i].state);
    BLEMini.write(sensors[i].state >> 8);
    
    Serial.print("Sync identifier: ");
    Serial.println(sensors[i].stateIdentifier);
    delay(1);
  }
  
  /*
  Serial.println(micros());
  Serial.println("  SYNC END");
  */
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

void constructBLEProperties() {
  properties[PROPERTY_SMOOTHING_MIN].value = 0;
  properties[PROPERTY_SMOOTHING_MIN].bleIdentifier = SMOOTHING_MIN_BYTE;
  properties[PROPERTY_SMOOTHING_MIN].eepMSB = EEP_SMOOTHING_MIN_MSB;
  properties[PROPERTY_SMOOTHING_MIN].eepLSB = EEP_SMOOTHING_MIN_LSB;
  
  properties[PROPERTY_SMOOTHING_MAX].value = 0;
  properties[PROPERTY_SMOOTHING_MAX].bleIdentifier = SMOOTHING_MAX_BYTE;
  properties[PROPERTY_SMOOTHING_MAX].eepMSB = EEP_SMOOTHING_MAX_MSB;
  properties[PROPERTY_SMOOTHING_MAX].eepLSB = EEP_SMOOTHING_MAX_LSB;
  
  properties[PROPERTY_MAX_OUTPUT].value = 0;
  properties[PROPERTY_MAX_OUTPUT].bleIdentifier = MAX_OUTPUT_BYTE;
  properties[PROPERTY_MAX_OUTPUT].eepMSB = EEP_MAX_OUTPUT_MSB;
  properties[PROPERTY_MAX_OUTPUT].eepLSB = EEP_MAX_OUTPUT_LSB;
  
  properties[PROPERTY_STRAIN_DAMPING_CURVE].value = 0;
  properties[PROPERTY_STRAIN_DAMPING_CURVE].bleIdentifier = STRAIN_DAMPING_CURVE_BYTE;
  properties[PROPERTY_STRAIN_DAMPING_CURVE].eepMSB = EEP_STRAINDAMPCURVE_MSB;
  properties[PROPERTY_STRAIN_DAMPING_CURVE].eepLSB = EEP_MAXSTRAINDAMP_LSB;
  
  properties[PROPERTY_STROKE_TIMEOUT_CYCLES].value = 0;
  properties[PROPERTY_STROKE_TIMEOUT_CYCLES].bleIdentifier = STROKE_TIMEOUT_CYCLES_BYTE;
  properties[PROPERTY_STROKE_TIMEOUT_CYCLES].eepMSB = EEP_STROKETIMOUTCYC_MSB;
  properties[PROPERTY_STROKE_TIMEOUT_CYCLES].eepLSB = EEP_STROKETIMOUTCYC_LSB;
  
  properties[PROPERTY_MAX_EFFORT].value = 0;
  properties[PROPERTY_MAX_EFFORT].bleIdentifier = MAX_EFFORT_BYTE;
  properties[PROPERTY_MAX_EFFORT].eepMSB = EEP_MAXEFFORT_MSB;
  properties[PROPERTY_MAX_EFFORT].eepLSB = EEP_MAXEFFORT_LSB;
  
  properties[PROPERTY_TORQUE_MULTIPLIER].value = 0;
  properties[PROPERTY_TORQUE_MULTIPLIER].bleIdentifier = TORQUE_MULTIPLIER_BYTE;
  properties[PROPERTY_TORQUE_MULTIPLIER].eepMSB = EEP_TRQ_MULT_MSB;
  properties[PROPERTY_TORQUE_MULTIPLIER].eepLSB = EEP_TRQ_MULT_LSB;
  
  properties[PROPERTY_MAX_STRAIN_DAMPING_SPEED].value = 0;
  properties[PROPERTY_MAX_STRAIN_DAMPING_SPEED].bleIdentifier = MAX_STRAIN_DAMPING_SPEED_BYTE;
  properties[PROPERTY_MAX_STRAIN_DAMPING_SPEED].eepMSB = EEP_MAXSTRAINDAMP_MSB;
  properties[PROPERTY_MAX_STRAIN_DAMPING_SPEED].eepLSB = EEP_MAXSTRAINDAMP_LSB;
}



