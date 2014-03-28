void manageDataProcessing() {
  
  if (rxDataIsFresh[DAT_RID_TRQ]) {
    //handleStrainMessage(rxData[DAT_RID_TRQ]);
    handleStrainMessageLight(rxData[DAT_RID_TRQ]);
    rxDataIsFresh[DAT_RID_TRQ] = false;
  }
  
  if (rxDataIsFresh[DAT_MTR_TMP]) {
    sensors[SENSOR_MOTOR_TEMP].value = map(rxData[DAT_MTR_TMP], 0, 64, 0, UINT16_MAX);
    sensors[SENSOR_MOTOR_TEMP].isFresh = true;
    
    rxDataIsFresh[DAT_MTR_TMP] = false;
  }
  
  if (rxDataIsFresh[DAT_BAT_VBAT]) {
    //Serial.print("batteryVoltage: ");
    //Serial.println(rxData[DAT_BAT_VBAT]);
    sensors[SENSOR_BATTERY_VOLTAGE].value = map(rxData[DAT_BAT_VBAT], 0, 64, 0, UINT16_MAX);
    sensors[SENSOR_BATTERY_VOLTAGE].isFresh = true;
    
    rxDataIsFresh[DAT_BAT_VBAT] = false;
  }
  
  if (rxDataIsFresh[DAT_MTR_SPD]) {
    uint16_t mappedSpeed = map(rxData[DAT_MTR_SPD], 0, 64, 0, UINT16_MAX);
    
    if (sensors[SENSOR_SPEED].value != mappedSpeed) {
      mapSpeedToDamping(rxData[DAT_MTR_SPD]);
      //buildPowerOutputCurve();
      
      sensors[SENSOR_SPEED].value = mappedSpeed;
      sensors[SENSOR_SPEED].isFresh = true;
    }
    
    rxDataIsFresh[DAT_MTR_SPD] = false;
  }
 
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
  
  float maxMultiplier = 1;
  motorSpeed = constrain(motorSpeed, 0, damping.maxX);
  byte mappedSpeed = map(motorSpeed, 0, damping.maxX, 0, 255);
  
  point speed1 = { mappedSpeed, 0 };
  point speed2 = { mappedSpeed, 255 };
  
  if ( !damping.cacheIsValid ) {
    rebuildBezierCache(damping);
  }
  
  for (int i = 0; i < RESOLUTION - 1; i++)
  {
    point curveSegment1 = damping.cache[i];
    point curveSegment2 = damping.cache[i + 1];
    
    point result;
    
    if (intersectionOfLineFrom(curveSegment1, curveSegment2, speed1, speed2, result)) {
      strainDampingMultiplier = (float)result.y / (float)BYTE_MAX;
      return strainDampingMultiplier;
    }
  }
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
  
  float filterAmount = map(64 - strainDelta, 0, 64, properties[PROPERTY_SMOOTHING_MIN].value, properties[PROPERTY_SMOOTHING_MAX].value);
  filterAmount /= (float)UINT16_MAX; //because map only works with whole numbers.
  float secondaryFilterAmount = (float)properties[PROPERTY_RIDER_EFFORT_FILTER_STRENGTH].value / (float)UINT16_MAX;
  
  riderEffort = smooth(strainDelta, riderEffort, filterAmount);
  filteredRiderEffort = smooth(riderEffort, filteredRiderEffort, secondaryFilterAmount);
  
  if (newStrain > 0) {
    cyclesSinceLastStroke = 0;
  }
  else {
    cyclesSinceLastStroke = min(cyclesSinceLastStroke + 1, 254);
  }
  
  if (cyclesSinceLastStroke > properties[PROPERTY_STROKE_TIMEOUT_CYCLES].value) {
    riderEffort = 0;
    filteredRiderEffort = 0;
  }
  
  byte torque;
  
  if (properties[PROPERTY_FANCY_ASSIST_STATE].value == STANDARD_ASSIST) {
    torque = map(filteredRiderEffort, 0, properties[PROPERTY_MAX_EFFORT].value, 0, 64);
  }
  else if (properties[PROPERTY_FANCY_ASSIST_STATE].value == EFFORT_MAPPING) {
    byte effortMappedToPower = mapEffortToPower(filteredRiderEffort);
    torque = map(effortMappedToPower, 0, BYTE_MAX, 0, 64);
  }
  
  float torqueMultiplier = ((float)properties[PROPERTY_TORQUE_MULTIPLIER].value / (float)UINT16_MAX) * 2;
  torque = constrain(round(torque * torqueMultiplier), 0, 64);
  Temp_Var_For_Fwd_Twrk_Msg = torque;
  
  float riderEffortSensorValue = constrain(riderEffort, 0, properties[PROPERTY_MAX_EFFORT].value);
  riderEffortSensorValue = map(riderEffortSensorValue, 0, properties[PROPERTY_MAX_EFFORT].value, 0, UINT16_MAX);
  riderEffortSensorValue = round(riderEffortSensorValue);
  sensors[SENSOR_RIDER_EFFORT].value = riderEffortSensorValue;
  sensors[SENSOR_RIDER_EFFORT].isFresh = true;
  
  float filteredRiderEffortSensorValue = constrain(filteredRiderEffort, 0, properties[PROPERTY_MAX_EFFORT].value);
  filteredRiderEffortSensorValue = map(filteredRiderEffortSensorValue, 0, properties[PROPERTY_MAX_EFFORT].value, 0, UINT16_MAX);
  sensors[SENSOR_FILTERED_RIDER_EFFORT].value = filteredRiderEffortSensorValue;
  sensors[SENSOR_FILTERED_RIDER_EFFORT].isFresh = true;
  
  unsigned short rawStrainSensorValue = map(newStrain, 0, 64, 0, UINT16_MAX);
  sensors[SENSOR_RAW_STRAIN].value = rawStrainSensorValue;
  sensors[SENSOR_RAW_STRAIN].isFresh = true;
  
  unsigned short torqueAppliedSensorValue = map(torque, 0, 64, 0, UINT16_MAX);
  sensors[SENSOR_TORQUE_APPLIED].value = torqueAppliedSensorValue;
  sensors[SENSOR_TORQUE_APPLIED].isFresh = true;
  
  /*
  Serial.print(riderEffort);
  Serial.print(",");
  Serial.print(filteredRiderEffort);
  Serial.print(",");
  Serial.print(strainDampingMultiplier);
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
