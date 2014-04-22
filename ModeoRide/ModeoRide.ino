///////////////////////////////////////////////////////////////////////////////////////////////////
/*
ModeoRide Mk3
Jon Gaull and Isaac Meadows
January 2013

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
#include <EEPROM.h>
#include <ModeoBLE.h>
// END LIBRARIES
//#include <ble_mini.h>

#include "ModeoRide.h"                  //  Local headers, need to move constants here
#include "CAN_Definitions.h"

//VARIABLE DECLARATIONS FOLLOW

PROGMEM prog_uchar RX_IDS[32] = {0x11, 0x14, 0x16, 0x21, 0x70, 0x72, 0x32, 0xAA, 0x9A, 0x12, 0x20, 0x92, 0x6C, 0x30, 0x31, 0x33, 0x61, 0x80, 0x1D, 0x3B, 0x3C, 0x3D, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xF0, 0xF9, 0xFA, 0xFB, 0xFC};
byte rxData[32];
bool rxDataIsFresh[32] = {0};

byte Temp_Var_For_Fwd_Twrk_Msg;
byte Temp_Var_For_Fwd_Twrk_UpperByte;

byte rxLen = 0;
byte rxBuf[8];

byte txLen = 0;
byte txBuf[8];

byte vehicleState = VEHICLE_OFF;

long unsigned int rxId;
long unsigned int txId;

byte cyclesSinceLastStroke = 0;

float riderEffort = 0;
float filteredRiderEffort = 0;

float strainDampingMultiplier = 0.0f;

boolean trqCmdTxFlag = false;

//*****TrqMgmt Variables

boolean lastButtonState = false;
  
ModeoBLE modeo = ModeoBLE(NUM_PROPERTIES, NUM_SENSORS, NUM_BEZIERS);

//  setup() is called at startup
void setup()
{
  Serial.begin(9600);    // Enable serial debug

  pinMode(ON_OFF_SWITCH_PIN, INPUT_PULLUP);
  pinMode(INDICATOR_LED_PIN, OUTPUT);
  pinMode(CAN_READY_PIN, INPUT);
  pinMode(SWITCH_LED_PIN, OUTPUT);
  pinMode(BLE_POWER_PIN, OUTPUT);
  
  digitalWrite(BLE_POWER_PIN, HIGH);

  CAN.begin(CAN_125KBPS);   //125kbps CAN is default for the Bionx syste;
  
  constructBLESensors();
  constructBLEProperties();
  constructBLEBeziers();
  
  lastButtonState = digitalRead(ON_OFF_SWITCH_PIN);
  
  activateBionx();
  
  modeo.startup();
}


// MAIN LOOP
void loop()
{
  unsigned long now = micros();
  
  manageVehicleState(digitalRead(ON_OFF_SWITCH_PIN));  // Get the state of the switch every cycle. This function can be slowed down if it turns out to take serious time
  
  performCANRX();
  
  modeo.update();
  
  performPeriodicMessageSend(now);
  
  manageTxTimers(now);
  
  manageDataProcessing();
  
  //Serial.print("freeMemory() = ");
  //Serial.println(freeMemory());
  
}

void manageVehicleState(bool switchValue) {
  if (lastButtonState != switchValue) {
    
    if (vehicleState == VEHICLE_OFF && switchValue == HIGH) {
      activateBionx(); // Fire the relay for a few seconds if we are off (ready to start) and the switch is ON
      modeo.startup();
      Serial.println("ACTIVATE BIONX COMPLETE");
    }
    else if (vehicleState == VEHICLE_ON && switchValue == HIGH) {
      shutdownBionx(); // send the stop cmds to the battery and motor inverter if the switch is off while we were running
      modeo.shutdown();
      Serial.println("SHUTDOWN BIONX COMPLETE");
    }
    
    lastButtonState = switchValue;
    delay(1); //debounce
  }
  
  digitalWrite(SWITCH_LED_PIN, vehicleState == VEHICLE_ON);
}

void constructBLEBeziers() {
  modeo.registerBezier(CURVE_TYPE_ASSIST);
  modeo.registerBezier(CURVE_TYPE_DAMPING);
}

void constructBLESensors() {
  
  for (byte i = 0; i < NUM_SENSORS; i++) {
    modeo.registerSensor(i);
  }
}

void constructBLEProperties() {
  
  byte doNotSaveListLength = 9;
  byte doNotSaveList[9] = { PROPERTY_SENSOR_RIDER_EFFORT_STATE,
                            PROPERTY_SENSOR_SPEED_STATE,
                            PROPERTY_SENSOR_RAW_STRAIN_STATE,
                            PROPERTY_SENSOR_TORQUE_APPLIED_STATE,
                            PROPERTY_SENSOR_MOTOR_TEMP_STATE,
                            PROPERTY_SENSOR_BATTERY_VOLTAGE_STATE,
                            PROPERTY_SENSOR_FILTERED_RIDER_EFFORT_STATE,
                            PROPERTY_SENSOR_CURRENT_STRAIN_STATE,
                            PROPERTY_TORQUE_MULTIPLIER };
  
  for (byte i = 0; i < NUM_PROPERTIES; i++) {
    
    boolean eepromSave = true;
    for (byte j = 0; j < doNotSaveListLength; j++) {
      if (doNotSaveList[j] == i) {
        eepromSave = false;
        break;
      }
    }
    
    modeo.registerProperty(i, eepromSave);
  }
}

