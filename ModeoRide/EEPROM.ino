void retrieveCalibrations(){

   Serial.print(micros());
 Serial.println("   Started cal restore");

  
  SMOOTHING_MIN = (EEPROM.read(EEP_SMOOTHING_MIN_MSB) << 8)+ EEPROM.read(EEP_SMOOTHING_MIN_LSB);
  
  SMOOTHING_MAX =  (EEPROM.read(EEP_SMOOTHING_MAX_MSB) << 8)+ EEPROM.read(EEP_SMOOTHING_MAX_LSB);
  
  MAX_OUTPUT =  (EEPROM.read(EEP_MAX_OUTPUT_MSB) << 8)+ EEPROM.read(EEP_MAX_OUTPUT_LSB);
  
  maxStrainDampingSpeed =  (EEPROM.read(EEP_MAXSTRAINDAMP_MSB) << 8)+ EEPROM.read(EEP_MAXSTRAINDAMP_LSB);
  
  STRAIN_DAMPING_CURVE = (EEPROM.read(EEP_STRAINDAMPCURVE_MSB) << 8)+ EEPROM.read(EEP_STRAINDAMPCURVE_LSB);
  
  STROKE_TIMEOUT_CYCLES =  (EEPROM.read(EEP_STROKETIMOUTCYC_MSB) << 8)+ EEPROM.read(EEP_STROKETIMOUTCYC_LSB);
  
  MAX_EFFORT =  (EEPROM.read(EEP_MAXEFFORT_MSB) << 8)+ EEPROM.read(EEP_MAXEFFORT_LSB);
  
  torqueMultiplier =  (EEPROM.read(EEP_TRQ_MULT_MSB) << 8)+ EEPROM.read(EEP_TRQ_MULT_LSB);

 Serial.print(micros());
 Serial.println("   Finishe  cal restore");

}


void storeCalibrations(){

 Serial.print(micros());
 Serial.println("   Started cal save");
  
  EEPROM.write(EEP_SMOOTHING_MIN_MSB, (SMOOTHING_MIN >> 8));
  EEPROM.write(EEP_SMOOTHING_MIN_LSB, (SMOOTHING_MIN));

  EEPROM.write(EEP_SMOOTHING_MAX_MSB, (SMOOTHING_MAX >> 8));
  EEPROM.write(EEP_SMOOTHING_MAX_LSB, (SMOOTHING_MAX));
  
  EEPROM.write(EEP_MAX_OUTPUT_MSB, (MAX_OUTPUT >> 8));
  EEPROM.write(EEP_MAX_OUTPUT_LSB, (MAX_OUTPUT));
  
  EEPROM.write(EEP_MAXSTRAINDAMP_MSB, (maxStrainDampingSpeed >> 8)); 
  EEPROM.write(EEP_MAXSTRAINDAMP_LSB, (maxStrainDampingSpeed));

  EEPROM.write(EEP_STRAINDAMPCURVE_MSB, (STRAIN_DAMPING_CURVE >> 8)); 
  EEPROM.write(EEP_STRAINDAMPCURVE_LSB, (STRAIN_DAMPING_CURVE));
  
  EEPROM.write(EEP_STROKETIMOUTCYC_MSB,(STROKE_TIMEOUT_CYCLES >> 8));
  EEPROM.write(EEP_STROKETIMOUTCYC_LSB, (STROKE_TIMEOUT_CYCLES));
  
  EEPROM.write(EEP_MAXEFFORT_MSB, (MAX_EFFORT >> 8));
  EEPROM.write(EEP_MAXEFFORT_LSB, (MAX_EFFORT));

  EEPROM.write(EEP_TRQ_MULT_MSB, (torqueMultiplier >> 8));
  EEPROM.write(EEP_TRQ_MULT_LSB, (torqueMultiplier));

 Serial.print(micros());
 Serial.println("   Finished cal save");


}



