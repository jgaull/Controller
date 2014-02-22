void manageDataProcessing() {
  
  if (rxDataIsFresh[DAT_RID_TRQ]) {
    handleStrainMessage(rxData[DAT_RID_TRQ]);
    rxDataIsFresh[DAT_RID_TRQ] = false;
  }
  
  if (rxDataIsFresh[DAT_MTR_TMP]) {
    sensors[SENSOR_MOTOR_TEMP].value = map(rxData[DAT_MTR_TMP], 0, 64, 0, UINT16_MAX);
    sensors[SENSOR_MOTOR_TEMP].isFresh = true;
    
    rxDataIsFresh[DAT_MTR_TMP] = false;
  }
  
  if (rxDataIsFresh[DAT_BAT_VBAT]) {
    sensors[SENSOR_BATTERY_VOLTAGE].value = map(rxData[DAT_BAT_VBAT], 0, 64, 0, UINT16_MAX);
    sensors[SENSOR_BATTERY_VOLTAGE].isFresh = true;
    
    rxDataIsFresh[DAT_BAT_VBAT] = false;
  }
  
  if (rxDataIsFresh[DAT_MTR_SPD]) {
    uint16_t mappedSpeed = map(rxData[DAT_MTR_SPD], 0, 64, 0, UINT16_MAX);
    
    if (sensors[SENSOR_SPEED].value != mappedSpeed) {
      recalculateBezierStrainDampingMultiplier();
      buildPowerOutputCurve();
      
      sensors[SENSOR_SPEED].value = mappedSpeed;
      sensors[SENSOR_SPEED].isFresh = true;
    }
    
    rxDataIsFresh[DAT_MTR_SPD] = false;
  }
 
}

/*void meadowsFilterAndTorque(byte newRiderTrq) {
  if (meadowsStrainBuffer < 10) {
    meadowsStrainBuffer = (meadowsStrainBuffer+ newRiderTrq) / 2;
  }
  else {
    meadowsStrainBuffer = ((meadowsStrainBuffer*2)+newRiderTrq)/3;
    meadowsStrainBuffer = ((meadowsStrainBuffer*2)+newRiderTrq)/3;
  }
  
  byte torque = map(meadowsStrainBuffer, 0, 40, 0, 64 );
  torque = constrain(torque, 0,64);

  Temp_Var_For_Fwd_Twrk_Msg = torque;
  
  Serial.print(meadowsStrainBuffer);
  Serial.print(",");
  Serial.print(torque);
  Serial.print(",");
  Serial.print(rxData[DAT_MTR_TRQ]);
  Serial.print(",");
  Serial.print(rxData[DAT_RID_TRQ]);
  Serial.print(",");
  Serial.print(rxData[DAT_MTR_SPD]);
  Serial.print(",");
  Serial.print(micros());
  Serial.println("");
}




void meadowsFilterAndTorqueAdvanced(byte newRiderTrq) {
  if (meadowsStrainBuffer < 10) {
    meadowsStrainBuffer = (meadowsStrainBuffer+ newRiderTrq) / 2;
  }
  else {
    meadowsStrainBuffer = ((meadowsStrainBuffer*2)+newRiderTrq)/3;
  }
  
  unsigned long now = micros();
 
  byte torque = map(meadowsStrainBuffer, 0, 40, 0, 64 );
  
  torque = constrain(torque, 0,64);
  Temp_Var_For_Fwd_Twrk_Msg = torque;
  
  if ((now >= TrqTimestamp + TRQ_UPDATE_DELTA) || ((now < TrqTimestamp) && ((4294967295 - (TrqTimestamp + now)) >= TRQ_UPDATE_DELTA))) {
    TrqTimestamp = now;
  }
  
  Serial.print(meadowsStrainBuffer);
  Serial.print(",");
  Serial.print(torque);
  Serial.print(",");
  Serial.print(rxData[DAT_MTR_TRQ]);
  Serial.print(",");
  Serial.print(rxData[DAT_RID_TRQ]);
  Serial.print(",");
  Serial.print(rxData[DAT_MTR_SPD]);
  Serial.print(",");
  Serial.print(micros());
  Serial.println("");
}*/

short calculatePowerOutput(float effort) {
  
  byte mappedEffort = constrain(effort, 0, properties[PROPERTY_MAX_EFFORT].value);
  mappedEffort = map(mappedEffort, 0, properties[PROPERTY_MAX_EFFORT].value, 0, 255);
  
  point effort1 = { mappedEffort, 0 };
  point effort2 = { mappedEffort, 255 };
  
  short powerOutput = 0;
  
  for (int i = 0; i < RESOLUTION - 1; i++)
  {
    point curveSegment1 = powerOutputCurve[i];
    point curveSegment2 = powerOutputCurve[i + 1];
    
    point result;
    
    if (i == 0 && curveSegment1.x == effort1.x) {
      powerOutput = curveSegment1.y - 127;
      return powerOutput;
    }
    
    if (intersectionOfLineFrom(curveSegment1, curveSegment2, effort1, effort2, result)) {
      powerOutput = result.y - 127;
      return powerOutput;
    }
  }
  
  return 0;
}

void recalculateBezierStrainDampingMultiplier() {
  
  unsigned long timestamp = micros();
  
  float maxMultiplier = 1;
  byte motorSpeed = constrain(rxData[DAT_MTR_SPD], 0, properties[PROPERTY_MAX_STRAIN_DAMPING_SPEED].value);
  byte mappedSpeed = map(motorSpeed, 0, properties[PROPERTY_MAX_STRAIN_DAMPING_SPEED].value, 0, 255);
  
  point speed1 = { mappedSpeed, 0 };
  point speed2 = { mappedSpeed, 255 };
  
  for (int i = 0; i < RESOLUTION - 1; i++)
  {
    point curveSegment1 = strainDampingCurve[i];
    point curveSegment2 = strainDampingCurve[i + 1];
    
    point result;
    
    if (intersectionOfLineFrom(curveSegment1, curveSegment2, speed1, speed2, result)) {
      strainDampingMultiplier = (float)result.y / 255.0f;
      break;
    }
  }
}

void buildPowerOutputCurve() {
  //unsigned long timestamp = micros();
  //Serial.println("power output");
  
  float maxMultiplier = 1;
  uint16_t maxSpeed = 30;
  byte motorSpeed = constrain(rxData[DAT_MTR_SPD], 0, maxSpeed);
  motorSpeed = map(motorSpeed, 0, maxSpeed, 0, 255);
  
  point speed1 = { motorSpeed, 0 };
  point speed2 = { motorSpeed, 255 };
  
  point assist;
  point regen;
  point sensitivity1;
  point sensitivity2;
  
  /*
  point assistCurve[RESOLUTION];
  point sensitivityCurve[RESOLUTION];
  point regenCurve[RESOLUTION];
  point powerOutputCurve[RESOLUTION];
  */
  
  for (int i = 0; i < RESOLUTION - 1; i++)
  {
    point curveSegment1 = assistCurve[i];
    point curveSegment2 = assistCurve[i + 1];
    
    point result;
    if (intersectionOfLineFrom(curveSegment1, curveSegment2, speed1, speed2, result)) {
      assist.x = 255;
      assist.y = map(result.y, 0, 255, 127, 255);
      break;
    }
  }
  
  for (int i = 0; i < RESOLUTION - 1; i++)
  {
    point curveSegment1 = regenCurve[i];
    point curveSegment2 = regenCurve[i + 1];
    
    point result;
    if (intersectionOfLineFrom(curveSegment1, curveSegment2, speed1, speed2, result)) {
      regen.x = 0;
      regen.y = map(result.y, 0, 255, 127, 0);
      break;
    }
  }
  
  for (int i = 0; i < RESOLUTION - 1; i++)
  {
    point curveSegment1 = sensitivityCurve[i];
    point curveSegment2 = sensitivityCurve[i + 1];
    
    point result;
    
    if (intersectionOfLineFrom(curveSegment1, curveSegment2, speed1, speed2, result)) {
      result.x = constrain(result.x, 1, 254);
      sensitivity1.x = result.x;
      sensitivity1.y = regen.y;
      
      sensitivity2.x = result.x;
      sensitivity2.y = assist.y;
      break;
    }
  }
  
  point startPoint = regen;
  point endPoint = assist;
  
  point control1 = sensitivity1;
  point control2 = sensitivity2;
  
  for (int i=0; i < RESOLUTION; ++i) {
    point p;
    float t = static_cast<float>(i)/(RESOLUTION - 1.0f);
    bezier(p, startPoint, control1, control2, endPoint, t);
    powerOutputCurve[i].x = p.x;
    powerOutputCurve[i].y = p.y;
  }
}

void buildDampingCurve() {
  point startPoint = { 0, 0 };
  point endPoint = { 255, 255 };
  
  byte firstPropertyIdentifier = PROPERTY_STRAIN_DAMPING_CONTROL1_X;
  
  point control1;
  control1.x = map(properties[firstPropertyIdentifier + 0].value, 0, UINT16_MAX, 0, 255);
  control1.y = map(properties[firstPropertyIdentifier + 1].value, 0, UINT16_MAX, 0, 255);
  
  point control2;
  control2.x = map(properties[firstPropertyIdentifier + 2].value, 0, UINT16_MAX, 0, 255);
  control2.y = map(properties[firstPropertyIdentifier + 3].value, 0, UINT16_MAX, 0, 255);
  
  for (int i=0; i < RESOLUTION; ++i) {
    point p;
    float t = static_cast<float>(i)/(RESOLUTION - 1.0f);
    bezier(p, startPoint, control1, control2, endPoint, t);
    strainDampingCurve[i].x = p.x;
    strainDampingCurve[i].y = p.y;
  }
}

void buildAssistCurve() {
  //Serial.println("Assist");
  point startPoint = { 0, 255 };
  point endPoint = { 255, 0 };
  
  byte firstPropertyIdentifier = PROPERTY_ASSIST_1_X;
  
  point control1;
  control1.x = map(properties[firstPropertyIdentifier + 0].value, 0, UINT16_MAX, 0, 255);
  control1.y = map(properties[firstPropertyIdentifier + 1].value, 0, UINT16_MAX, 0, 255);
  
  point control2;
  control2.x = map(properties[firstPropertyIdentifier + 2].value, 0, UINT16_MAX, 0, 255);
  control2.y = map(properties[firstPropertyIdentifier + 3].value, 0, UINT16_MAX, 0, 255);
  
  for (int i=0; i < RESOLUTION; ++i) {
    point p;
    float t = static_cast<float>(i)/(RESOLUTION - 1.0f);
    bezier(p, startPoint, control1, control2, endPoint, t);
    assistCurve[i].x = p.x;
    assistCurve[i].y = p.y;
    /*
    Serial.print(p.x);
    Serial.print(",");
    Serial.println(p.y);
    */
  }
}

void buildRegenCurve() {
  //Serial.println("Regen");
  point startPoint = { 0, 0 };
  point endPoint = { 255, 255 };
  
  byte firstPropertyIdentifier = PROPERTY_REGEN_1_X;
  
  point control1;
  control1.x = map(properties[firstPropertyIdentifier + 0].value, 0, UINT16_MAX, 0, 255);
  control1.y = map(properties[firstPropertyIdentifier + 1].value, 0, UINT16_MAX, 0, 255);
  
  point control2;
  control2.x = map(properties[firstPropertyIdentifier + 2].value, 0, UINT16_MAX, 0, 255);
  control2.y = map(properties[firstPropertyIdentifier + 3].value, 0, UINT16_MAX, 0, 255);
  
  for (int i=0; i < RESOLUTION; ++i) {
    point p;
    float t = static_cast<float>(i)/(RESOLUTION - 1.0f);
    bezier(p, startPoint, control1, control2, endPoint, t);
    regenCurve[i].x = p.x;
    regenCurve[i].y = p.y;
    
    /*
    Serial.print(p.x);
    Serial.print(",");
    Serial.println(p.y);
    */
  }
}

void buildSensitivityCurve() {
  //Serial.println("Sensitivity");
  point startPoint = { 0, 255 };
  point endPoint = { 255, 0 };
  
  byte firstPropertyIdentifier = PROPERTY_SENSITIVITY_1_X;
  
  point control1;
  control1.x = map(properties[firstPropertyIdentifier + 0].value, 0, UINT16_MAX, 0, 255);
  control1.y = map(properties[firstPropertyIdentifier + 1].value, 0, UINT16_MAX, 0, 255);
  
  point control2;
  control2.x = map(properties[firstPropertyIdentifier + 2].value, 0, UINT16_MAX, 0, 255);
  control2.y = map(properties[firstPropertyIdentifier + 3].value, 0, UINT16_MAX, 0, 255);
  
  for (int i=0; i < RESOLUTION; ++i) {
    point p;
    float t = static_cast<float>(i)/(RESOLUTION - 1.0f);
    bezier(p, startPoint, control1, control2, endPoint, t);
    sensitivityCurve[i].x = p.x;
    sensitivityCurve[i].y = p.y;
    /*
    Serial.print(p.x);
    Serial.print(",");
    Serial.println(p.y);
    */
  }
}

/*void buildBezierCurve(byte firstPropertyIdentifier, point startPoint, point endPoint, point &curve) {
  
  Serial.println(firstPropertyIdentifier);
  
  point control1;
  control1.x = map(properties[firstPropertyIdentifier + 0].value, 0, UINT16_MAX, 0, 255);
  control1.y = map(properties[firstPropertyIdentifier + 1].value, 0, UINT16_MAX, 0, 255);
  
  point control2;
  control2.x = map(properties[firstPropertyIdentifier + 2].value, 0, UINT16_MAX, 0, 255);
  control2.y = map(properties[firstPropertyIdentifier + 3].value, 0, UINT16_MAX, 0, 255);
  
  for (int i=0; i < RESOLUTION; ++i) {
    point p;
    float t = static_cast<float>(i)/(RESOLUTION - 1.0f);
    bezier(p, startPoint, control1, control2, endPoint, t);
    curve[i].x = p.x;
    curve[i].y = p.y;
    Serial.print(p.x);
    Serial.print(",");
    Serial.print(p.y);
  }
}*/

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


void addNewStroke(PedalStroke stroke) {
  byte newLength = min(strokesLength + 1, MAX_STORED_STROKES);
  
  for (int i = strokesLength - 1; i >= 0; i--) {
    if (i + 1 < newLength) {
      strokes[i + 1] = strokes[i];
    }
  }
  
  strokes[0] = stroke;
  strokesLength = newLength;
}

/*PedalStroke copyStroke(PedalStroke stroke) {
  PedalStroke copy;
  copy.length = stroke.length;
  copy.index = stroke.index;
  copy.runs = stroke.runs;
  
  for (byte i = 0; i < stroke.length; i++) {
    copy.data[i] = stroke.data[i];
  }
  
  return copy;
}*/

void handleStrainMessage(byte newStrain) {
  //strain values come in as positive deltas
  byte strainDelta = newStrain;
  
  //If there was no change in strain
  if ((strainDelta == 0 && currentStrain > 0) || (currentPedalStrokeLength >= MAX_STROKE_LENGTH && strainDelta == 0)) {
    
    //Reset any strokes that have finished playing back. It's time for a new stroke.
    for (byte i = 0; i < strokesLength; i++) {
      if (strokes[i].index == strokes[i].length) {
        strokes[i].index = 0;
        strokes[i].runs++;
        
        if (strokes[i].runs >= NUM_AVERAGED_STROKES) {
          removeStrokeAtIndex(i);
          i--;
        }
      }
    }
    
    //Create a new stroke
    PedalStroke completedStroke;
    
    byte startIndex = 0;
    if (currentPedalStrokeLength >= MAX_STROKE_LENGTH) {
      startIndex = (currentPedalStrokeLength - MAX_STROKE_LENGTH) % MAX_STROKE_LENGTH;
    }
    
    byte length = min(currentPedalStrokeLength, MAX_STROKE_LENGTH);
    
    for (byte i = 0; i < length; i++) {
      completedStroke.data[i] = currentPedalStroke[(i + startIndex) % MAX_STROKE_LENGTH];
    }
    
    sensors[SENSOR_STROKE_LENGTH].value = map(currentPedalStrokeLength, 0, MAX_STROKE_LENGTH, 0, UINT16_MAX);
    sensors[SENSOR_STROKE_LENGTH].isFresh = true;
    
    completedStroke.length = min(currentPedalStrokeLength, MAX_STROKE_LENGTH);
    completedStroke.index = 0;
    completedStroke.runs = 0;
    completedStroke.strokeId = strokeId;
    addNewStroke(completedStroke);
    
    strokeId++;
    
    //Reset our stroke variables
    currentStrain = 0;
    currentPedalStrokeLength = 0;
  }
  
  //We just got our first value of the new stroke
  if (strainDelta > 0 && currentStrain == 0) {
    for (byte i = 0; i < strokesLength; i++) {
      //If there are any strokes still playing out from before, we reset them to the current index.
      if (strokes[i].index > currentPedalStrokeLength && strokes[i].index != strokes[i].length) {
        strokes[i].index = currentPedalStrokeLength;
        strokes[i].runs++;
        
        //If the stroke is played out then remove it.
        if (strokes[i].runs >= NUM_AVERAGED_STROKES) {
          removeStrokeAtIndex(i);
          i--;
        }
      }
    }
  }
  
  //since they are deltas we add them all together.
  currentStrain += strainDelta;
  
  if (currentStrain > 0) {
    cyclesSinceLastStroke = 0;
  }
  else {
    cyclesSinceLastStroke = min(cyclesSinceLastStroke + 1, 255);
  }
  
  int expectedStrain = 0;
  for (byte i = 0; i < strokesLength; i++) {
    
    byte index = strokes[i].index;
    
    if (index < strokes[i].length) {
      
      for (byte j = 0; j <= index; j++) {
        expectedStrain += strokes[i].data[j];
      }
      
      strokes[i].index++;
    }
    else if (index > currentPedalStrokeLength) {
      strokes[i].index = currentPedalStrokeLength;
      strokes[i].runs++;
      
      if (strokes[i].runs >= NUM_AVERAGED_STROKES) {
        removeStrokeAtIndex(i);
        i--;
      }
    }
    
    /*
    Serial.print(strokes[i].strokeId);
    Serial.print(",");
    
    if (index < strokes[i].length) {
      Serial.print(strokes[i].data[index]);
    }
    else {
      Serial.print(0);
    }
    
    Serial.print(",");
    Serial.print(strokes[i].length);
    Serial.print(",");
    Serial.print(index);
    Serial.print(",");
    Serial.print(strokes[i].runs);
    Serial.print(",");
    //*/
  }
  
  //Serial.println();
  
  if (expectedStrain > 0) {
    expectedStrain /= strokesLength;
  }
  
  currentPedalStroke[currentPedalStrokeLength % MAX_STROKE_LENGTH] = strainDelta;
  currentPedalStrokeLength++;
  
  float strainDiff = currentStrain - expectedStrain;
  strainDiff = abs(strainDiff);
  
  //Map will allow values above and below the max. I'm currently leaving them unconstrained.
  float filterAmount = map(properties[PROPERTY_MAX_OUTPUT].value - strainDiff, 0, properties[PROPERTY_MAX_OUTPUT].value, properties[PROPERTY_SMOOTHING_MIN].value, properties[PROPERTY_SMOOTHING_MAX].value);
  filterAmount /= (float)UINT16_MAX; //because map only works with round numbers.
  
  byte filterMultiplier = 1;
  if (cyclesSinceLastStroke > properties[PROPERTY_STROKE_TIMEOUT_CYCLES].value) {
    filterMultiplier = 0;
  }
  
  filterAmount *= filterMultiplier;
  
  float dampenedStrain = currentStrain * strainDampingMultiplier;
  riderEffort = smooth(dampenedStrain, riderEffort, filterAmount);
  
  float secondaryFilterAmount = (float)properties[PROPERTY_RIDER_EFFORT_FILTER_STRENGTH].value / (float)UINT16_MAX;
  filteredRiderEffort = smooth(riderEffort, filteredRiderEffort, secondaryFilterAmount);
  
  float multiplier = ((float)properties[PROPERTY_TORQUE_MULTIPLIER].value / (float)UINT16_MAX) * 2;
  //float multipliedEffort = multiplier * riderEffort;
  //multipliedEffort = round(constrain(multipliedEffort, 0, properties[PROPERTY_MAX_EFFORT].value));
  
  short powerOutput = calculatePowerOutput(riderEffort);
  powerOutput = map(powerOutput, -127, 128, -63, 64);
  short powerOutputSensorValue = powerOutput;
  powerOutput *= multiplier;
  byte torque = constrain(powerOutput, 0, 64);
  
  //A bunch of shit for sensor managers.
  sensors[SENSOR_POWER_OUTPUT].value = map(powerOutputSensorValue, -127, 128, 0, UINT16_MAX);
  sensors[SENSOR_POWER_OUTPUT].isFresh = true;
  
  float riderEffortSensorValue = constrain(riderEffort, 0, properties[PROPERTY_MAX_EFFORT].value);
  riderEffortSensorValue = map(riderEffortSensorValue, 0, properties[PROPERTY_MAX_EFFORT].value, 0, UINT16_MAX);
  sensors[SENSOR_RIDER_EFFORT].value = riderEffortSensorValue;
  sensors[SENSOR_RIDER_EFFORT].isFresh = true;
  
  float filteredRiderEffortSensorValue = constrain(filteredRiderEffort, 0, properties[PROPERTY_MAX_EFFORT].value);
  filteredRiderEffortSensorValue = map(filteredRiderEffortSensorValue, 0, properties[PROPERTY_MAX_EFFORT].value, 0, UINT16_MAX);
  sensors[SENSOR_FILTERED_RIDER_EFFORT].value = filteredRiderEffortSensorValue;
  sensors[SENSOR_FILTERED_RIDER_EFFORT].isFresh = true;
  
  float currentStrainSensorValue = constrain(currentStrain, 0, properties[PROPERTY_MAX_OUTPUT].value);
  currentStrainSensorValue = map(currentStrainSensorValue, 0, properties[PROPERTY_MAX_OUTPUT].value, 0, UINT16_MAX);
  sensors[SENSOR_CURRENT_STRAIN].value = currentStrainSensorValue;
  sensors[SENSOR_CURRENT_STRAIN].isFresh = true;
  
  unsigned short rawStrainSensorValue = map(strainDelta, 0, 64, 0, UINT16_MAX);
  sensors[SENSOR_RAW_STRAIN].value = rawStrainSensorValue;
  sensors[SENSOR_RAW_STRAIN].isFresh = true;
  
  unsigned short torqueAppliedSensorValue = map(torque, 0, 64, 0, UINT16_MAX);
  sensors[SENSOR_TORQUE_APPLIED].value = torqueAppliedSensorValue;
  sensors[SENSOR_TORQUE_APPLIED].isFresh = true;
  //end a bunch of shit for sensor managers
  
  Temp_Var_For_Fwd_Twrk_Msg = torque;
  
 // hasTorqueMessage = true;  //  NOW ALTERNATELY HANDLED BY MEDIUM MESSAGE RX TIMER 
  //for debug.
  /*
  Serial.print(riderEffort);
  Serial.print(",");
  Serial.print(filteredRiderEffort);
  Serial.print(",");
  Serial.print(micros());
  Serial.println("");
  //*/
}

float smooth(float newVal, float oldVal, float filterStrength) {
  
  filterStrength = constrain(filterStrength, 0, 1);
  
  oldVal = (newVal * (1 - filterStrength)) + (oldVal  *  filterStrength);

  return oldVal;
}

void removeStrokeAtIndex(byte index) {
  if (strokesLength > 0 && index < strokesLength && index >= 0) {
    for (int i = index; i < strokesLength - 1; i++) {
      if (i + 1 < strokesLength) {
        strokes[i] = strokes[i + 1];
      }
    }
    
    strokesLength--;
  }
}
