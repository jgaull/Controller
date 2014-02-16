void activateBionx(){
     //digitalWrite(WAKE_RELAY_PIN,HIGH);
     digitalWrite(INDICATOR_LED_PIN, HIGH);
     
     //delay(1000);
     retrieveCalibrations();
     
     //digitalWrite(WAKE_RELAY_PIN,LOW);
     digitalWrite(INDICATOR_LED_PIN, LOW);
     
    
    txBuf[1]= 0x00;
    txBuf[1]= 0x20;
    txBuf[2]= 0x00;
    txBuf[3]= 0x00;
    CAN.sendMsgBuf(0x10, 0, 0x4, txBuf); 
    
    delay(50) ;
     
    txBuf[1]= 0x00;
    txBuf[1]= 0x21;
    txBuf[2]= 0x00;
    txBuf[3]= 0x01;
    CAN.sendMsgBuf(0x10, 0, 0x4, txBuf); 

    delay(50) ;
     
    txBuf[1]= 0x00;
    txBuf[1]= 0x3B;
    CAN.sendMsgBuf(0x10, 0, 0x2, txBuf); 

    delay(50) ;
     
    txBuf[1]= 0x00;
    txBuf[1]= 0x3C;
    CAN.sendMsgBuf(0x10, 0, 0x2, txBuf);     
     
    delay(50) ;
     
    txBuf[1]= 0x00;
    txBuf[1]= 0x3D;
    CAN.sendMsgBuf(0x10, 0, 0x2, txBuf); 
     
     delay(50) ;
     
    txBuf[1]= 0x00;
    txBuf[1]= 0x22;
    txBuf[2]= 0x00;
    txBuf[3]= 0x00;
    CAN.sendMsgBuf(0x10, 0, 0x4, txBuf); 
    
     delay(50) ;
     
    txBuf[1]= 0x00;
    txBuf[1]= 0x41;
    txBuf[2]= 0x00;
    txBuf[3]= 0x01;
    CAN.sendMsgBuf(0x20, 0, 0x4, txBuf); 
    
    delay(50) ;
     
    txBuf[1]= 0x00;
    txBuf[1]= 0x42;
    txBuf[2]= 0x00;
    txBuf[3]= 0x01;
    CAN.sendMsgBuf(0x20, 0, 0x4, txBuf); 

    delay(50) ;
     
    txBuf[1]= 0x00;
    txBuf[1]= 0x20;
    CAN.sendMsgBuf(0x20, 0, 0x2, txBuf); 
    
    delay(50) ;
     
    txBuf[1]= 0x00;
    txBuf[1]= 0x6C;
    CAN.sendMsgBuf(0x20, 0, 0x2, txBuf); 

    delay(10) ;
     
    txBuf[1]= 0x00;
    txBuf[1]= 0xF9;
    CAN.sendMsgBuf(0x10, 0, 0x2, txBuf);     

    delay(10); 
     
    txBuf[1]= 0x00;
    txBuf[1]= 0xFA;
    CAN.sendMsgBuf(0x10, 0, 0x2, txBuf);     
    
    delay(10); 
     
    txBuf[1]= 0x00;
    txBuf[1]= 0xFB;
    CAN.sendMsgBuf(0x10, 0, 0x2, txBuf);     
    
    delay(10) ;
     
    txBuf[1]= 0x00;
    txBuf[1]= 0xFC;
    CAN.sendMsgBuf(0x10, 0, 0x2, txBuf);
    
    buildDampingCurve();
    buildAssistCurve();
    buildRegenCurve();
    buildSensitivityCurve();
    buildPowerOutputCurve();
}


void shutdownBionx(){
  
  storeCalibrations();
  
//  JIC code:  SEND TWO zero-speed commands, SEND 0A command, SET DIRECTION FORWARD
    txBuf[1]= 0x00;
    txBuf[1]= CMD_MTR_TRQ_ID;
    txBuf[2]= 0x00;
    txBuf[3]= 0x00;
    
    CAN.sendMsgBuf(0x20, 0, 0x4, txBuf); 

txBuf[1]= 0x00;
    txBuf[1]= CMD_MTR_TRQ_ID;
    txBuf[2]= 0x00;
    txBuf[3]= 0x00;
    
    CAN.sendMsgBuf(0x20, 0, 0x4, txBuf); 

txBuf[1]= 0x00;
    txBuf[1]= 0x0A;
    txBuf[2]= 0x00;
    txBuf[3]= 0x00;
    
    CAN.sendMsgBuf(0x20, 0, 0x4, txBuf); 

txBuf[1]= 0x00;
    txBuf[1]= 0x41;
    txBuf[2]= 0x00;
    txBuf[3]= 0x01;
    
    CAN.sendMsgBuf(0x20, 0, 0x4, txBuf); 

// END JIC CODE

// SEND OFF COMMAND TO MOTOR
    txBuf[1]= 0x00;
    txBuf[1]= 0x42;
    txBuf[2]= 0x00;
    txBuf[3]= 0x01;
    
    CAN.sendMsgBuf(0x20, 0, 0x4, txBuf); 

// SEND OFF COMMAND TO BATTERY
    txBuf[0]= 0x00;
    txBuf[1]= 0x25;
    txBuf[2]= 0x00;
    txBuf[3]= 0x01;
    
    CAN.sendMsgBuf(0x10, 0, 0x4, txBuf);
}


