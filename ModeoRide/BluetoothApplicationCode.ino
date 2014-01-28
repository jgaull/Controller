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
 // check for switch on and CAN rx code indicating ready to transmit
 /*Serial.print("enableRiderEffortUpdates: ");
 Serial.println(enableRiderEffortUpdates);
 Serial.print("sendBleFlg: ");
 Serial.println(sendBleFlg);*/
 
//   if (enableRiderEffortUpdates && sendBleFlg)
 
  if (enableRiderEffortUpdates && sendBleFlg)
  {
    digitalWrite(INDICATOR_LED_PIN, HIGH);
    float convertedEffort = ((float)riderEffort / (float)MAX_EFFORT) * UINT16_MAX;
    uint16_t effortValue = constrain(convertedEffort, 0, UINT16_MAX);
    BLEMini.write(RIDER_EFFORT_BYTE);
    BLEMini.write(effortValue);
    BLEMini.write(effortValue >> 8);
    
    sendBleFlg = false;  // clear ready to transmit flag
  }
  else
  {
    digitalWrite(INDICATOR_LED_PIN, LOW);
  }
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
    
    /*Serial.print("data1: ");
    Serial.println(data1);
    Serial.print("data2: ");
    Serial.println(data2);*/
    
    //Serial.print("identifier: ");
    //Serial.println(identifier);
    
    uint16_t value = (data2 << 8) + data1;
    
    //Serial.print("value: ");
    //Serial.println(value);
    
    if (identifier == SEND_PARAMS_BYTE) {
        performBluetoothSync();
        Serial.println("send params");
        return;
    }
    
    boolean isValidIdentifier = true;
    switch(identifier)
    {
      case SMOOTHING_MIN_BYTE:
      
        SMOOTHING_MIN = value;
        
        //Serial.print("smoothingMin: ");
        //Serial.println(SMOOTHING_MIN);
        break;
        
      case SMOOTHING_MAX_BYTE:
      
        SMOOTHING_MAX = value;
        
        //Serial.print("smoothingMax: ");
        //Serial.println(SMOOTHING_MAX);
        break;
      
      case MAX_OUTPUT_BYTE:
      
        MAX_OUTPUT = value;
        recalculateStrainDampingMultiplier();
        
        //Serial.print("maxOutput: ");
        //Serial.println(MAX_OUTPUT);
        break;
        
      case MAX_STRAIN_DAMPING_SPEED_BYTE:
      
        maxStrainDampingSpeed = value;
        recalculateStrainDampingMultiplier();
        
        //Serial.print("maxInput: ");
        //Serial.println(maxStrainDampingSpeed);
        break;
        
      case STRAIN_DAMPING_CURVE_BYTE:
      
        //STRAIN_DAMPING_CURVE = ((float)value / (float)UINT16_MAX) * 2;
        STRAIN_DAMPING_CURVE = value;
        recalculateStrainDampingMultiplier();
        
        //Serial.print("strainDampingCurve: ");
        //Serial.println(STRAIN_DAMPING_CURVE);
        break;
      
      case STROKE_TIMEOUT_CYCLES_BYTE:
      
        STROKE_TIMEOUT_CYCLES = value;
        
        //Serial.print("strokeTimeoutCycles: ");
        //Serial.println(STROKE_TIMEOUT_CYCLES);
        break;
        
      case MAX_EFFORT_BYTE:
      
        MAX_EFFORT = value;
        
        //Serial.print("maxEffort: ");
        //Serial.println(MAX_EFFORT);
        break;
        
      case ENABLE_RIDER_EFFORT_UPDATES_BYTE:
        
        if (value == 0) {
          enableRiderEffortUpdates = false;
        }
        else if (value == 1) {
          enableRiderEffortUpdates = true;
        }
        
        //Serial.print("enableRiderEffortUpdates: ");
        //Serial.println(enableRiderEffortUpdates);
        break;
        
      case TORQUE_MULTIPLIER_BYTE:
      
        torqueMultiplier = value;
        
        //Serial.print("torqueMultiplier: ");
        //Serial.println(torqueMultiplier);
        break;
        
      case ENABLE_CURRENT_STRAIN_UPDATES_BYTE:
      
        if (value == 0) {
          enableCurrentStrainUpdates = false;
        }
        else if (value == 1) {
          enableCurrentStrainUpdates = true;
        }
        
        //Serial.print("currentStrainUpdates: ");
        //Serial.println(enableCurrentStrainUpdates);
        break;
        
      default:
        /*Serial.print("Unknown Identifier: ");
        Serial.print(identifier, HEX);
        Serial.print(", ");
        Serial.println(identifier);*/
        isValidIdentifier = false;
    }
    
    if (isValidIdentifier) {
      performPropertySync(identifier, value);
    }
  }
}

void performPropertySync(byte identifier, uint16_t value) {
  digitalWrite(INDICATOR_LED_PIN, HIGH);
  
  BLEMini.write(identifier);
  BLEMini.write(value);
  BLEMini.write(value >> 8);
  
  //Serial.print("Sync value: ");
  //Serial.println(value);
  //Serial.print("sync id: ");
  //Serial.println(identifier);
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
  
  BLEMini.write(ENABLE_RIDER_EFFORT_UPDATES_BYTE);
  BLEMini.write(enableRiderEffortUpdates);
  BLEMini.write(enableRiderEffortUpdates >> 8);
  
  BLEMini.write(TORQUE_MULTIPLIER_BYTE);
  BLEMini.write(torqueMultiplier);
  BLEMini.write(torqueMultiplier >> 8);
}



