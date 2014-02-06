///////////////////////////////////////////////////////////////////////////////////////////////////
/*
ModeoRide Mk3
Jon Gaull and Isaac Meadows
January 2012

Notes:
Mk1- Ended with basic strain guage filtering, assist application, on/off commands according to switch, non trq messages sent by array
Trq cmd sent on every strain message reciept

Mk2a - Reorganized code.  Added periodic message queues

Mk3 - Full time based CAN message management:  3 tx queues @ configurable rates



*/
//////////////////////////////////////////////////////////////////////////////////////////////////

//  LIBRARIES
#include <SPI.h>                        //  For sparkfun CAN board communications
#include <boards.h>
#include <services.h>                 //  boards.h and services.h are old libraries for the fullsize BLE board
#include <mcp_can.h>                    //  Awesome library for CAN board, specifically traceivers 
//#include <MemoryFree.h>
#include <AltSoftSerial.h>    			//  Serial comms library for communication with RedBear BLE Mini
#include <avr/pgmspace.h>
// END LIBRARIES
//#include <ble_mini.h>

#include "ModeoRide.h"                  //  Local headers, need to move constants here
#include "CAN_Definitions.h"
#include "BLE_Definitions.h"
#include "DataProcessing_Definitions.h"
#include "EEPROM_Definitions.h"

#include <EEPROM.h>

//VARIABLE DECLARATIONS FOLLOW

PROGMEM prog_uchar bleArray [6] [19] = {
  {0xA0, 0x11, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x0F},
  {0xB0, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x0E},
  {0xC0, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0xD0},
  {0xD0, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x0C},
  {0xE0, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x0B},
  {0xF0, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x0A}, // 6 row, 19 col
};

byte bleFastPointer = 0;
byte bleMedPointer = 0;
byte bleSlowPointer = 0;

//byte bleSlowBuff[] =  {0x0,0x10,0x20,0x30,0x40,0x50,0x60,0x70,0x80,0x90,0x11,0x21,0x31,0x41,0x51,0xF1,0xF2,0xF3,0xF3,0xF5};

PROGMEM prog_uchar RX_IDS[32] = {0x11, 0x14, 0x16, 0x21, 0x70, 0x72, 0x32, 0xAA, 0x9A, 0x12, 0x20, 0x92, 0x6C, 0x30, 0x31, 0x33, 0x61, 0x80, 0x1D, 0x3B, 0x3C, 0x3D, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xF0, 0xF9, 0xFA, 0xFB, 0xFC};
byte rxData[32];
bool rxDataIsFresh[32] = {0};

byte Temp_Var_For_Fwd_Twrk_Msg;
byte Temp_Var_For_Fwd_Twrk_UpperByte;

unsigned long bleTxStamp = 0;
unsigned long bleFastStamp = 0;
unsigned long bleMedStamp = 0;
unsigned long bleSlowStamp = 0;


byte rxLen = 0;
byte rxBuf[8];

byte txLen = 0;
byte txBuf[8];

// Hard  code message length of commands vs queries if needed, and percentage of cycles where TX attempts
unsigned char CMD_TX_LEN = 4;
unsigned char QUERY_TX_LEN = 2;
byte CYCLE_TX_RATE = 200;
byte CycleTxCnt = 0;

byte CYCLE_DEBUG_RATE = 100;
byte CycleDebugCnt = 0;

unsigned int CYCLE_BLETX_Counter = 1;
unsigned int CYCLE_ACTION_COUNTER = 10000; // 0 -65535
unsigned int CycleActionCnt = 0;


// State management variables
bool readyToStart = 1;
bool EnableCANTX = 0;
bool EnableBluetoothTX = 0;
bool EnableBluetoothRX = 0;
byte trqAssistState = TRQ_ASSIST_OFF;

long unsigned int rxId;
long unsigned int txId;

PROGMEM prog_uchar fastTxMsgs[][3] = {  // ID, DLC, sigID
  {0x20, 0x02, 0x11},
  {0x20, 0x04, 0x0A}

};
byte fastTxData [(sizeof(fastTxMsgs) / sizeof(fastTxMsgs[0]))] = {0};

PROGMEM prog_uchar mediumTxMsgs[][3] = {  // ID, DLC, sigID
  {0x58, 0x02, 0x9C},
  {0x10, 0x02, 0x32},
  {0x10, 0x02, 0x9A},
  {0x10, 0x02, 0xAA},
  {0x20, 0x02, 0x14},
  {0x20, 0x02, 0x21},
  {0x20, 0x02, 0x70},
  {0x20, 0x02, 0x72},
};

PROGMEM prog_uchar slowTxMsgs[][3] = {
  {0x10, 0x02, 0x30},
  {0x10, 0x02, 0x31},
  {0x10, 0x02, 0x33},
  {0x10, 0x02, 0x61},
  {0x10, 0x02, 0x80},
  {0x10, 0x02, 0x1D},
  {0x10, 0x02, 0xA1},
  {0x10, 0x02, 0xA2},
  {0x10, 0x02, 0xA3},
  {0x10, 0x02, 0xA4},
  {0x10, 0x02, 0xA5},
  {0x20, 0x02, 0x12},
  {0x20, 0x02, 0x16},
  {0x20, 0x02, 0x92},

};



canMsg rxMsgs[1];
byte RXno = 0;

byte txPointer = 0;

byte realTorque;
byte vBatt;

//This needs to change to a PedalStroke
int currentStrain = 0;
int currentPedalStroke[MAX_STROKE_LENGTH];
byte currentPedalStrokeLength = 0;

//Expected Strain Finding
PedalStroke strokes[MAX_STORED_STROKES];
byte strokesLength = 0;
byte strokeId = 0;
byte cyclesSinceLastStroke = 0;

float riderEffort = 0;

float strainDampingMultiplier = 0.0f;
point strainDampingCurve[RESOLUTION];

boolean trqCmdTxFlag = false;

//*****TrqMgmt Variables


unsigned long fastTxStamp = 0;
unsigned long mediumTxStamp = 0;
unsigned long slowTxStamp = 0;




unsigned long canMsgTxStamp = 0;

bool fastTxFlag = 0;
bool mediumTxFlag = 0;
bool slowTxFlag = 0;

byte mediumTxPointer = 0;
byte slowTxPointer = 0;
byte fastTxPointer = 0;

byte REAL_SPEED_THRESH = 0x04;
byte VBATT_THRESH = 0xA0;

Sensor sensors[NUM_SENSORS];
Property properties[NUM_PROPERTIES];

AltSoftSerial BLEMini;
//#define BLEMini Serial


//  setup() is called at startup
void setup()
{

  Serial.begin(9600);    // Enable serial debug

  pinMode(ON_OFF_SWITCH_PIN, INPUT);
  pinMode(INDICATOR_LED_PIN, OUTPUT);
  pinMode(WAKE_RELAY_PIN, OUTPUT);
  pinMode(CAN_READY_PIN, INPUT);
  pinMode(MARKER_PIN, INPUT);

  // Default to internally pull high, change it if you need
  //digitalWrite(DIGITAL_IN_PIN, HIGH);
  //digitalWrite(DIGITAL_IN_PIN, LOW);

  CAN.begin(CAN_125KBPS);   //125kbps CAN is default for the Bionx system

  BLEMini.begin(57600);  //  BLE serial.  Lower speeds cause chaos, this speed gets noise due to interrupt issues
  
  constructBLESensors();
  constructBLEProperties();
  rebuildStrainDampingCurve();
  
  Serial.println("SETUP COMPLETE");
}


// MAIN LOOP
void loop()
{
  
  manageVehicleState(digitalRead(ON_OFF_SWITCH_PIN));  // Get the state of the switch every cycle. This function can be slowed down if it turns out to take serious time

  managePhysicalIO();

  performCANRX();

  unsigned long now = micros();

  // performBluetoothSend(now);

  performBluetoothSend1();

  performBluetoothReceive();
  
  performPeriodicMessageSend(now);

  manageTxTimers(now);
  
  manageDataProcessing();

  //performSerialDebugging();

  manageActionCounter();
  
  /*
  Serial.print("freeMemory() = ");
  Serial.println(freeMemory());
  */
}

void performSerialDebugging() {
  if (CycleActionCnt % CYCLE_DEBUG_RATE == 0) {
    Serial.print(rxData[DAT_MTR_SPD]);
    Serial.print(",TrqUpper: ");
    Serial.print(Temp_Var_For_Fwd_Twrk_Msg);
    Serial.print(",TrqLower");
    Serial.print(Temp_Var_For_Fwd_Twrk_Msg);
    Serial.print(",Effort: ");
    Serial.print(riderEffort);
    Serial.print(",Curr Strn:");
    Serial.print(currentStrain);
    Serial.print(",PedStrkLen: ");
    Serial.print(currentPedalStrokeLength);
    Serial.print(",StrokesLen:");
    Serial.print(strokesLength);
    Serial.println(" ");

  }
}

void manageActionCounter() {
  if ( CycleActionCnt == CYCLE_ACTION_COUNTER)
  {
    CycleActionCnt = 0;
  }
  else if (CycleActionCnt > CYCLE_ACTION_COUNTER)
  {
    CycleActionCnt = 0;    // This should never happen
  }
  else
  {
    CycleActionCnt++;
  }
}

void manageVehicleState(bool switchValue) {
  if (readyToStart == 1 && switchValue == HIGH)
  {
    activateBionx();  // Fire the relay for a few seconds if we are off (ready to start) and the switch is ON
    readyToStart = 0;
    EnableCANTX = 1;
    EnableBluetoothTX = 1;
    EnableBluetoothRX = 1;
    Serial.println("ACTIVATE BIONX COMPLETE");
  }
  else if (readyToStart == 0 && switchValue == LOW)
  {
    shutdownBionx();   // send the stop cmds to the battery and motor inverter if the switch is off while we were running
    readyToStart = 1;
    EnableCANTX = 0;
    EnableBluetoothTX = 0;
    EnableBluetoothRX = 0;
     Serial.println("SHURTDOWN BIONX COMPLETE");
  }
}

