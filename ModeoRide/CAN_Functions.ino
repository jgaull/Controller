
void performCANRX() {
  // READ A MESSAGE EVERY EXECUTION CYCLE, IF AVAILABLE
  if (!digitalRead(CAN_READY_PIN)) {                       // If pin 2 is low, read receive buffer
    
    byte rxBuf[4];
    byte rxLen = 0;
    
    CAN.readMsgBuf(&rxLen, rxBuf);              // Read data: len = data length, buf = data byte(s)
    //rxId = CAN.getCanId();                    // Get message ID

    if (rxLen == 4)
    {
      for (int rxPointer = 0; rxPointer < 32; rxPointer++) {
        if (pgm_read_byte(&(RX_IDS[rxPointer])) == rxBuf[1]) {
          rxData[rxPointer] = rxBuf[3];
          rxDataIsFresh[rxPointer] = 1;
          break;
        }
      }
    }


    /* switch (rxBuf[1]){
    case MTR_SPEED_ID:
      handleSpeedMessage(rxBuf[3]);
      break;
    case MTR_TRQ_ID:
      realTorque = rxBuf[3];
      break;
    case BATT_V_ID:
      vBatt = rxBuf[3];
      break;
    case MTR_PEDSTRAIN_ID:
      handleStrainMessage(rxBuf[3]);
      sendBleFlg = true;
      break;
      */

  }
}

void manageTxTimers(unsigned long now) {
  if ((now >= mediumTxStamp + MEDIUM_TX_DELTA) || ((now < mediumTxStamp) && ((4294967295 - (mediumTxStamp + now)) >= MEDIUM_TX_DELTA))) {
    mediumTxStamp = now;
    mediumTxFlag = 1;
  }

  if ((now >= slowTxStamp + SLOW_TX_DELTA) || ((now < slowTxStamp) && ((4294967295 - (slowTxStamp + now)) >= SLOW_TX_DELTA))) {
    slowTxStamp = now;
    slowTxFlag = 1;
  }

  if ((now >= fastTxStamp + FAST_TX_DELTA) || ((now < fastTxStamp) && ((4294967295 - (fastTxStamp + now)) >= FAST_TX_DELTA))) {
    fastTxStamp = now;
    fastTxFlag = 1;
  }
  
  if ((now >= torqueTxStamp + TORQUE_TX_DELTA) || ((now < torqueTxStamp) && ((4294967295 - (torqueTxStamp + now)) >= TORQUE_TX_DELTA))) {
    torqueTxStamp = now;
    trqCmdTxFlag = 1;
  }
}





void performPeriodicMessageSend(unsigned long now) {
  /*Serial.print("EnableCANTX = ");
  Serial.println(EnableCANTX);
  Serial.print("fastTxFlag = ");
  Serial.println(fastTxFlag);
  Serial.print("mediumTxFlag = ");
  Serial.println(mediumTxFlag);*/
  
  if (vehicleState == VEHICLE_ON) {
    
    if (trqCmdTxFlag) {
      unsigned char txBuf[4] = {0, CMD_MTR_TRQ_ID, Temp_Var_For_Fwd_Twrk_UpperByte, Temp_Var_For_Fwd_Twrk_Msg};
      CAN.sendMsgBuf(0x20, 0, 0x04, txBuf);
      delay(1);
      trqCmdTxFlag = false;
    }
    
    if (fastTxFlag) {
      if (fastTxPointer < (sizeof(fastTxMsgs) / sizeof(fastTxMsgs[0]))) {
        unsigned char txBuf[2] = { 0, pgm_read_byte(&(fastTxMsgs[fastTxPointer][2])) };
        CAN.sendMsgBuf(pgm_read_byte(&(fastTxMsgs[fastTxPointer][0])), 0, pgm_read_byte(&(fastTxMsgs[fastTxPointer][1])), txBuf);
        delay(1);
        fastTxPointer++;
      }
      else {
        fastTxPointer = 0;
        fastTxFlag = 0;
      }
    }
    
    if (mediumTxFlag) {
      if (mediumTxPointer < (sizeof(mediumTxMsgs) / sizeof(mediumTxMsgs[0]))) {
        unsigned char txBuf[2] = {0, pgm_read_byte(&(mediumTxMsgs[mediumTxPointer][2]))};
        CAN.sendMsgBuf(pgm_read_byte(&(mediumTxMsgs[mediumTxPointer][0])), 0, pgm_read_byte(&(mediumTxMsgs[mediumTxPointer][1])), txBuf);
        delay(1);
        mediumTxPointer++;
      }
      else {
        mediumTxPointer = 0;
        mediumTxFlag = 0;
      }
    }
    
    if (slowTxFlag) {
      if (slowTxPointer < (sizeof(slowTxMsgs) / sizeof(slowTxMsgs[0]))) {
        unsigned char txBuf[2] = {0, pgm_read_byte(&(slowTxMsgs[slowTxPointer][2]))};
        CAN.sendMsgBuf(pgm_read_byte(&(slowTxMsgs[slowTxPointer][0])), 0, pgm_read_byte(&(slowTxMsgs[slowTxPointer][1])), txBuf);
        delay(1);
        slowTxPointer++;
      }
      else {
        slowTxPointer = 0;
        slowTxFlag = 0;
      }
    }
  }

  
  //canMsgTxStamp = now;
  //}
}


