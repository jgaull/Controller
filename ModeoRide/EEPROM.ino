void retrieveCalibrations(){
  
  //Serial.println("Started property restore");
 
  byte propertyCount = 0;
  for (byte i = 0; i < NUM_PROPERTIES; i++) {
  
    if (properties[i].eepromSave) {
      int lsb = i * 2 + 1;
      int msb = i * 2;
      properties[i].value = (EEPROM.read(msb) << 8) + EEPROM.read(lsb);
      propertyCount++;
    }
  }
  
  /*
  Serial.print("Restored ");
  Serial.print(propertyCount);
  Serial.println(" properties.");
  */
}


void storeCalibrations() {
  //Serial.println("Started property save");
  
  byte propertyCount = 0;
  for (byte i = 0; i < NUM_PROPERTIES; i++) {
    
    if (properties[i].eepromSave && properties[i].pendingSave) {
      int lsb = i * 2 + 1;
      int msb = i * 2;
      EEPROM.write(msb, properties[i].value >> 8);
      EEPROM.write(lsb, properties[i].value);
      propertyCount++;
    }
  }
  
  /*
  Serial.print("Saved ");
  Serial.print(propertyCount);
  Serial.println(" properties.");
  */
}



