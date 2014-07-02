void manageDataProcessing() {
  
  if (rxDataIsFresh[DAT_RID_TRQ]) {
    //handleStrainMessage(rxData[DAT_RID_TRQ]);
    handleStrainMessageLight(rxData[DAT_RID_TRQ]);
    rxDataIsFresh[DAT_RID_TRQ] = false;
    handleRideStart();
  }
  
  if (rxDataIsFresh[DAT_MTR_TMP]) {
    
    modeo.setValueForSensor(map(rxData[DAT_MTR_TMP], 0, 64, 0, UINT16_MAX), SENSOR_MOTOR_TEMP);
    rxDataIsFresh[DAT_MTR_TMP] = false;
    
  }
  
  if (rxDataIsFresh[DAT_BAT_VBAT]) {
    
    unsigned short mappedBatteryVoltage = constrain(rxData[DAT_BAT_VBAT], 170, 217);
    mappedBatteryVoltage = map(mappedBatteryVoltage, 170, 217, 0, UINT16_MAX);
    
    if (mappedBatteryVoltage != modeo.getValueForSensor(SENSOR_BATTERY_VOLTAGE)) {
      
      modeo.setValueForSensor(mappedBatteryVoltage, SENSOR_BATTERY_VOLTAGE);
      
    }
    
    rxDataIsFresh[DAT_BAT_VBAT] = false;
  }
  
  if (rxDataIsFresh[DAT_MTR_SPD]) {
    uint16_t mappedSpeed = map(rxData[DAT_MTR_SPD], 0, 64, 0, UINT16_MAX);
    
    if (modeo.getValueForSensor(SENSOR_SPEED) != mappedSpeed) {
      
      mapSpeedToDamping(rxData[DAT_MTR_SPD]);
      modeo.setValueForSensor(mappedSpeed, SENSOR_SPEED);
      handleRideStart();
    }
    
    rxDataIsFresh[DAT_MTR_SPD] = false;
  }
}

void handleRideStart() {
  /*
  Serial.print("startRideTimestamp = ");
  Serial.println(startRideTimestamp);
  
  Serial.print("filteredRiderEffort = ");
  Serial.println(filteredRiderEffort);
  
  Serial.print("rxData[DAT_MTR_SPD] = ");
  Serial.println(rxData[DAT_MTR_SPD]);
  //*/
  
  if (startRideTimestamp == 0 && filteredRiderEffort > 5 && rxData[DAT_MTR_SPD] > 4) {
    Serial.println("Started Ride!");
    startRideTimestamp = millis();
    createEvent(EVENT_START_RIDE);
  }
}

void createEvent(byte type) {
  byte headerSize = 1;
    byte datemSize = 2;
    byte totalSize = headerSize + (NUM_SENSORS - 1) * datemSize;
    
    byte event[17];
    event[0] = type;
    
    for (byte i = 0; i < NUM_SENSORS; i++) {
      
      if (i != SENSOR_HAS_EVENT) {
        unsigned short sensorValue = modeo.getValueForSensor(i);
        byte scaledValue = map(sensorValue, 0, UINT16_MAX, 0, BYTE_MAX);
        byte dataIndex = headerSize + datemSize * i;
        
        event[dataIndex] = i;
        event[dataIndex + 1] = scaledValue;
        /*
        Serial.print("sensorValue[");
        Serial.print(i);
        Serial.print("] = ");
        Serial.println(sensorValue);
        */
      }
    }
    
    /*
    for (byte i = 0; i < totalSize; i++) {
      Serial.print("event[");
      Serial.print(i);
      Serial.print("] = ");
      Serial.println(event[i]);
    }
    */
    
    modeo.setValueForProperty(PROPERTY_EVENT, event);
    modeo.setValueForSensor(1, SENSOR_HAS_EVENT);
}

//Bezier intersection functions
byte mapEffortToPower(float effort) {
  
  byte mappedEffort = constrain(effort, 0, assist.maxX);
  mappedEffort = map(mappedEffort, 0, assist.maxX, 0, 255);
  
  point effort1 = { mappedEffort, 0 };
  point effort2 = { mappedEffort, 255 };
  
  short powerOutput = 0;
  
  if ( !assist.cacheIsValid ) {
    rebuildBezierCache(assist);
  }
  
  for (int i = 0; i < RESOLUTION - 1; i++)
  {
    point curveSegment1 = assist.cache[i];
    point curveSegment2 = assist.cache[i + 1];
    
    point result;
    
    if (i == 0 && curveSegment1.x == effort1.x) {
      powerOutput = curveSegment1.y;
      return powerOutput;
    }
    
    if (intersectionOfLineFrom(curveSegment1, curveSegment2, effort1, effort2, result)) {
      powerOutput = result.y;
      powerOutput = map(powerOutput, 0, 255, 0, assist.maxY);
      return powerOutput;
    }
  }
  
  return 0;
}

float mapSpeedToDamping(byte motorSpeed) {
  
  if (motorSpeed == 0) {
    return 0;
  }
  
  unsigned short maxDampingSpeed = modeo.getUnsignedShortValueForProperty(PROPERTY_MAX_DAMPING_SPEED);
  float damping = (float)motorSpeed / (float)maxDampingSpeed;
  damping = constrain(damping, 0, 1);
  strainDampingMultiplier = damping;
  return damping;
}

//Bezier helpers
void rebuildBezierCache(Bezier &rebuildBezier) {
  /*
  Serial.print("rebuild bezier: ");
  Serial.println(rebuildBezier.type);
  
  Serial.print("maxX: ");
  Serial.println(rebuildBezier.maxX);
  Serial.print("maxY: ");
  Serial.println(rebuildBezier.maxY);
  */
  
  point startPoint = rebuildBezier.points[0];
  point control1 = rebuildBezier.points[1];
  point control2 = rebuildBezier.points[2];
  point endPoint = rebuildBezier.points[3];
  
  for (int i=0; i < RESOLUTION; ++i) {
    point p;
    float t = static_cast<float>(i)/(RESOLUTION - 1.0f);
    bezier(p, startPoint, control1, control2, endPoint, t);
    rebuildBezier.cache[i].x = p.x;
    rebuildBezier.cache[i].y = p.y;
    /*
    Serial.print(p.x);
    Serial.print(",");
    Serial.println(p.y);
    //*/
  }
  
  rebuildBezier.cacheIsValid = true;
}

// simple linear interpolation between two points
void lerp(point &dest, const point &a, const point &b, const float t) {
    dest.x = a.x + (b.x-a.x)*t;
    dest.y = a.y + (b.y-a.y)*t;
}

// evaluate a point on a bezier-curve. t goes from 0 to 1.0
void bezier(point &dest, const point& a, const point& b, const point& c, const point& d, const float t) {
    point ab,bc,cd,abbc,bccd;
    lerp(ab, a,b,t);           // point between a and b (green)
    lerp(bc, b,c,t);           // point between b and c (green)
    lerp(cd, c,d,t);           // point between c and d (green)
    lerp(abbc, ab,bc,t);       // point between ab and bc (blue)
    lerp(bccd, bc,cd,t);       // point between bc and cd (blue)
    lerp(dest, abbc,bccd,t);   // point on the bezier-curve (black)
}

boolean intersectionOfLineFrom(point &p1, point &p2, point &p3, point &p4, point &result) {
  float d = ((float)p2.x - (float)p1.x) * ((float)p4.y - (float)p3.y) - ((float)p2.y - (float)p1.y) * ((float)p4.x - (float)p3.x);
  
  if (d == 0) {
      return false; // parallel lines
  }
      
  float u = (((float)p3.x - (float)p1.x)*((float)p4.y - (float)p3.y) - ((float)p3.y - (float)p1.y)*((float)p4.x - (float)p3.x))/d;
  float v = (((float)p3.x - (float)p1.x)*((float)p2.y - (float)p1.y) - ((float)p3.y - (float)p1.y)*((float)p2.x - (float)p1.x))/d;
  
  if (u < 0.0 || u > 1.0) {
    return false; // intersection point not between p1 and p2
  }
  
  if (v < 0.0 || v > 1.0) {
    return false; // intersection point not between p3 and p4
  }
  
  result.x = (float)p1.x + u * ((float)p2.x - (float)p1.x);
  result.y = (float)p1.y + u * ((float)p2.y - (float)p1.y);
  
  return true;
}

//The core data processing
void handleStrainMessageLight(byte newStrain) {
  
  byte strainDelta = round(newStrain * strainDampingMultiplier);
  
  unsigned short smoothingMin = modeo.getUnsignedShortValueForProperty(PROPERTY_SMOOTHING_MIN);
  unsigned short smoothingMax = modeo.getUnsignedShortValueForProperty(PROPERTY_SMOOTHING_MAX);
  unsigned short riderEffortFilterStrength = modeo.getUnsignedShortValueForProperty(PROPERTY_RIDER_EFFORT_FILTER_STRENGTH);
  //unsigned short maxEffort = modeo.getValueForProperty(PROPERTY_MAX_EFFORT);
  unsigned short maxEffort = 64;
  
  float filterAmount = map(64 - strainDelta, 0, 64, smoothingMin, smoothingMax);
  filterAmount /= (float)UINT16_MAX; //because map only works with whole numbers.
  float secondaryFilterAmount = (float)riderEffortFilterStrength / (float)UINT16_MAX;
  
  riderEffort = smooth(strainDelta, riderEffort, filterAmount);
  filteredRiderEffort = smooth(riderEffort, filteredRiderEffort, secondaryFilterAmount);
  
  if (newStrain > 0) {
    cyclesSinceLastStroke = 0;
  }
  else {
    cyclesSinceLastStroke = min(cyclesSinceLastStroke + 1, 254);
  }
  
  unsigned short strokeTimeoutCycles = modeo.getUnsignedShortValueForProperty(PROPERTY_STROKE_TIMEOUT_CYCLES);
  if (cyclesSinceLastStroke > strokeTimeoutCycles) {
    riderEffort = 0;
    filteredRiderEffort = 0;
  }
  
  byte effortMappedToPower = mapEffortToPower(filteredRiderEffort);
  byte torque = map(effortMappedToPower, 0, BYTE_MAX, 0, 64);
  
  unsigned short torqueMultiplierValue = modeo.getUnsignedShortValueForProperty(PROPERTY_TORQUE_MULTIPLIER);
  
  float torqueMultiplier = ((float)torqueMultiplierValue / (float)UINT16_MAX) * 2;
  torque = constrain(round(torque * torqueMultiplier), 0, 64);
  Temp_Var_For_Fwd_Twrk_Msg = torque;
  
  float riderEffortSensorValue = constrain(riderEffort, 0, maxEffort);
  riderEffortSensorValue = map(riderEffortSensorValue, 0, maxEffort, 0, UINT16_MAX);
  riderEffortSensorValue = round(riderEffortSensorValue);
  modeo.setValueForSensor(riderEffortSensorValue, SENSOR_RIDER_EFFORT);
  
  float filteredRiderEffortSensorValue = constrain(filteredRiderEffort, 0, maxEffort);
  filteredRiderEffortSensorValue = map(filteredRiderEffortSensorValue, 0, maxEffort, 0, UINT16_MAX);
  modeo.setValueForSensor(filteredRiderEffortSensorValue, SENSOR_FILTERED_RIDER_EFFORT);
  
  unsigned short rawStrainSensorValue = map(newStrain, 0, 64, 0, UINT16_MAX);
  modeo.setValueForSensor(rawStrainSensorValue, SENSOR_RAW_STRAIN);
  
  unsigned short torqueAppliedSensorValue = map(torque, 0, 64, 0, UINT16_MAX);
  modeo.setValueForSensor(torqueAppliedSensorValue, SENSOR_TORQUE_APPLIED);
  
  /*
  Serial.print(riderEffort);
  Serial.print(",");
  Serial.print(filteredRiderEffort);
  Serial.print(",");
  Serial.print(strainDampingMultiplier);
  Serial.print(",");
  Serial.print(torque);
  Serial.print(",");
  Serial.print(newStrain);
  Serial.print(",");
  Serial.print(strainDelta);
  Serial.println("");
  //*/
}

//Simple smoothing function
float smooth(float newVal, float oldVal, float filterStrength) {
  
  filterStrength = constrain(filterStrength, 0, 1);
  
  oldVal = (newVal * (1 - filterStrength)) + (oldVal  *  filterStrength);

  return oldVal;
}
