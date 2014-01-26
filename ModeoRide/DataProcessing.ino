void manageDataProcessing(){
 if(rxDataIsFresh[DAT_RID_TRQ]==1){
   handleStrainMessage(rxData[DAT_RID_TRQ]);
   //meadowsFilterAndTorqueBasic(rxData[DAT_RID_TRQ]);
   //meadowsFilterAndTorqueAdvanced(rxData[DAT_RID_TRQ]);
   sendBleFlg = true;
   rxDataIsFresh[DAT_RID_TRQ]=0;
   
  }
 
}
 
 
void meadowsFilterAndTorqueBasic(byte newRiderTrq)
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




void meadowsFilterAndTorqueAdvanced(byte newRiderTrq)
{
if (meadowsStrainBuffer < 10)
{
  meadowsStrainBuffer = (meadowsStrainBuffer+ newRiderTrq) / 2;
}
else{
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
 
}


 
 
 
 
  

void recalculateStrainDampingMultiplier() {
  strainDampingMultiplier = MAX_DAMPING_MULTIPLIER / sqrt(pow(maxStrainDampingSpeed, ((float)STRAIN_DAMPING_CURVE/(float)UINT16_MAX) * 2));
  Serial.print("strainDampingMultiplier: ");
  Serial.println(strainDampingMultiplier, 5);
}


void addNewStroke(PedalStroke stroke) {
  for (int i = strokesLength - 1; i >= 0; i--) {
    if (i + 1 < strokesLength + 1) {
      strokes[i + 1] = strokes[i];
    }
  }
  
  strokes[0] = stroke;
  
  strokesLength = min(strokesLength + 1, MAX_STORED_STROKES);
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
      expectedStrain += strokes[i].data[index];
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
  
  currentPedalStroke[currentPedalStrokeLength % MAX_STROKE_LENGTH] = currentStrain;
  currentPedalStrokeLength++;
  
  float strainDiff = currentStrain - expectedStrain;
  strainDiff = abs(strainDiff);
  
  //Map will allow values above and below the max. I'm currently leaving them unconstrained.
  float filterAmount = map(MAX_OUTPUT - strainDiff, 0, MAX_OUTPUT, SMOOTHING_MIN, SMOOTHING_MAX);
  filterAmount /= (float)SMOOTHING_DIVISOR; //because map only works with round numbers.
  
  byte filterMultiplier = 1;
  if (cyclesSinceLastStroke > STROKE_TIMEOUT_CYCLES) {
    filterMultiplier = 0;
  }
  
  filterAmount *= filterMultiplier;
  
  float strainDampingExponent = ((float)STRAIN_DAMPING_CURVE / (float)UINT16_MAX) * 2;
  float strainDamping = sqrt(pow(rxData[DAT_MTR_SPD], strainDampingExponent)) * strainDampingMultiplier;
  
  strainDamping = constrain(strainDamping, 0, MAX_DAMPING_MULTIPLIER);
  float dampenedStrain = currentStrain * strainDamping;
  riderEffort = smooth(dampenedStrain, riderEffort, filterAmount);
  
  float multiplier = ((float)torqueMultiplier / (float)UINT16_MAX) * 2;
  float multipliedEffort = multiplier * riderEffort;
  float constrainedEffort = round(constrain(multipliedEffort, 0, MAX_EFFORT));
  byte torque = map(constrainedEffort, 0, MAX_EFFORT, 0, 64);
  
  Temp_Var_For_Fwd_Twrk_Msg = torque;
 // hasTorqueMessage = true;  //  NOW ALTERNATELY HANDLED BY MEDIUM MESSAGE RX TIMER 
  //for debug.
  ///*
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
