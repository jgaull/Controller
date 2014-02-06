void retrieveCalibrations(){

 Serial.print(micros());
 Serial.println("   Started cal restore");
 
 for (byte i = 0; i < NUM_PROPERTIES; i++) {
   
   if (properties[i].eepromSave) {
     int lsb = i * 2 + 1;
     int msb = i * 2;
     properties[i].value = (EEPROM.read(msb) << 8) + EEPROM.read(lsb);
   }
 }

 Serial.print(micros());
 Serial.println("Finished property restore");

}


void storeCalibrations(){
  Serial.print(micros());
  Serial.println("   Started cal save");
  
  for (byte i = 0; i < NUM_PROPERTIES; i++) {
    
    if (properties[i].eepromSave) {
      int lsb = i * 2 + 1;
      int msb = i * 2;
      EEPROM.write(msb, properties[i].value >> 8);
      EEPROM.write(lsb, properties[i].value);
    }
  }

 Serial.print(micros());
 Serial.println("Finished property save");


}



