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
      
      sensors[SENSOR_SPEED].value = mappedSpeed;
      sensors[SENSOR_SPEED].isFresh = true;
    }
    
    rxDataIsFresh[DAT_MTR_SPD] = false;
  }
 
}
 

void meadowsFilterAndTorque(byte newRiderTrq)
{
if (meadowsStrainBuffer < 10)
{
  meadowsStrainBuffer = (meadowsStrainBuffer+ newRiderTrq) / 2;
}
else{
  meadowsStrainBuffer = ((meadowsStrainBuffer*2)+newRiderTrq)/3;
  meadowsStrainBuffer = ((meadowsStrainBuffer*2)+newRiderTrq)/3;
}
 
 
  byte torque = map(meadowsStrainBuffer, 0, 40, 0, 64 );
  
  torque = constrain(torque, 0,64);

  Temp_Var_For_Fwd_Twrk_Msg = torque;
 
 /*
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
 */
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

void rebuildStrainDampingCurve() {
  
  point strainDamping1 = { 0, 0 };
  point strainDamping2 = { 0, 0 };
  point strainDamping3 = { 255, 255 };
  point strainDamping4 = { 255, 255 };

  for (int i=0; i < RESOLUTION; ++i) {
    point p;
    float t = static_cast<float>(i)/(RESOLUTION - 1.0f);
    bezier(p, strainDamping1, strainDamping2, strainDamping3, strainDamping4, t);
    strainDampingCurve[i] = p;
  }
}

// simple linear interpolation between two points
void lerp(point &dest, const point &a, const point &b, const float t)
{
    dest.x = a.x + (b.x-a.x)*t;
    dest.y = a.y + (b.y-a.y)*t;
}

// evaluate a point on a bezier-curve. t goes from 0 to 1.0
void bezier(point &dest, const point& a, const point& b, const point& c, const point& d, const float t)
{
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
  filterAmount /= (float)SMOOTHING_DIVISOR; //because map only works with round numbers.
  
  byte filterMultiplier = 1;
  if (cyclesSinceLastStroke > properties[PROPERTY_STROKE_TIMEOUT_CYCLES].value) {
    filterMultiplier = 0;
  }
  
  filterAmount *= filterMultiplier;
  
  float dampenedStrain = currentStrain * strainDampingMultiplier;
  riderEffort = smooth(dampenedStrain, riderEffort, filterAmount);
  
  float multiplier = ((float)properties[PROPERTY_TORQUE_MULTIPLIER].value / (float)UINT16_MAX) * 2;
  float multipliedEffort = multiplier * riderEffort;
  multipliedEffort = round(constrain(multipliedEffort, 0, properties[PROPERTY_MAX_EFFORT].value));
  byte torque = map(multipliedEffort, 0, properties[PROPERTY_MAX_EFFORT].value, 0, 64);
  
  //A bunch of shit for sensor managers.
  float riderEffortSensorValue = constrain(riderEffort, 0, properties[PROPERTY_MAX_EFFORT].value);
  uint16_t riderEffortValue = map(riderEffortSensorValue, 0, properties[PROPERTY_MAX_EFFORT].value, 0, UINT16_MAX);
  sensors[SENSOR_RIDER_EFFORT].value = riderEffortValue;
  sensors[SENSOR_RIDER_EFFORT].isFresh = true;
  
  float currentStrainSensorValue = constrain(currentStrain, 0, properties[PROPERTY_MAX_OUTPUT].value);
  currentStrainSensorValue = map(currentStrainSensorValue, 0, properties[PROPERTY_MAX_OUTPUT].value, 0, UINT16_MAX);
  sensors[SENSOR_CURRENT_STRAIN].value = currentStrainSensorValue;
  sensors[SENSOR_CURRENT_STRAIN].isFresh = true;
  
  byte rawStrainSensorValue = map(strainDelta, 0, 64, 0, UINT16_MAX);
  sensors[SENSOR_RAW_STRAIN].value = 0;
  sensors[SENSOR_RAW_STRAIN].isFresh = true;
  
  byte torqueAppliedSensorValue = map(torque, 0, 64, 0, UINT16_MAX);
  sensors[SENSOR_TORQUE_APPLIED].value = 0;
  sensors[SENSOR_TORQUE_APPLIED].isFresh = true;
  //end a bunch of shit for sensor managers
  
  Temp_Var_For_Fwd_Twrk_Msg = torque;
  
 // hasTorqueMessage = true;  //  NOW ALTERNATELY HANDLED BY MEDIUM MESSAGE RX TIMER 
  //for debug.
  /*
  Serial.print(dampenedStrain);
  Serial.print(",");
  Serial.print(currentStrain);
  Serial.print(",");
  Serial.print(strainDamping);
  Serial.print(",");
  Serial.print(riderEffort);
  Serial.print(",");
  Serial.print(filterAmount);
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
  //*/
}

/*byte calculateTorque()
{
  if(trqAssistState == TRQ_ASSIST_OFF && realSpeed > REAL_SPEED_THRESH && vBatt > VBATT_THRESH) {
      trqAssistState = TRQ_ASSIST_ON;
  }
  
  if(trqAssistState == TRQ_ASSIST_ON && (realSpeed < REAL_SPEED_THRESH || vBatt < VBATT_THRESH)) {
      trqAssistState = TRQ_ASSIST_OFF;
  }
  
  if (trqAssistState == TRQ_ASSIST_ON){
   
  // return 60;
    
    return min((int)(40.0 *((27.0-(float)realSpeed)/(27.0)) * ((float)filteredPedalTorque/15.0)),60);
  }
  else {
    return 0;
  }

}

byte filter(byte newVal, byte oldVal, byte number){
  if(newVal>1) {
    return (int)(((float)newVal*75 + (float)(number-75)*((float)oldVal))/(float)number);
  }
  else {
    return (int)(((float)newVal + (float)(number-1)*(float)oldVal)/(float)number);
  }
}*/

float smooth(byte newVal, float oldVal, float filterStrength) {
  
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
