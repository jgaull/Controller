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
//byte rxLen = 0;
//byte rxBuf[8];
//byte txLen = 0;
//byte txBuf[8];
//long unsigned int rxId;
//long unsigned int txId;
boolean trqCmdTxFlag = false;

byte vehicleState = VEHICLE_OFF;
unsigned long startRideTimestamp = 0;
unsigned long pendingShutdownTimestamp = 0;

byte cyclesSinceLastStroke = 0;

float riderEffort = 0;
float filteredRiderEffort = 0;
float strainDampingMultiplier = 0.0f;

float batteryPercentage = 0;

boolean lastButtonState = false;

Bezier assist;

ModeoBLE modeo = ModeoBLE();

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
  
  //constructBLESensors();
  //constructBLEProperties();
  
  lastButtonState = digitalRead(ON_OFF_SWITCH_PIN);
  
  activateBionx();
  
  modeo.startup(&propertyDidChange);
  
  /*
  byte length;
  byte value[2];
  modeo.setUnsignedShortValueForProperty(1000, PROPERTY_FANCY_ASSIST_STATE);
  modeo.getValueForProperty(PROPERTY_FANCY_ASSIST_STATE, &length, value);
  
  Serial.print("length = ");
  Serial.println(length);
  Serial.print("value = ");
  Serial.println(modeo.getUnsignedShortValueForProperty(PROPERTY_FANCY_ASSIST_STATE));
  for (byte i = 0; i < length; i++) {
    Serial.print("value[");
    Serial.print(i);
    Serial.print("] = ");
    Serial.println(value[i]);
  }
  //*/
}


// MAIN LOOP
void loop()
{
  modeo.update();
  unsigned long now = micros();
  manageVehicleState(digitalRead(ON_OFF_SWITCH_PIN));  // Get the state of the switch every cycle. This function can be slowed down if it turns out to take serious time
  performCANRX();
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
      modeo.startup(&propertyDidChange);
      digitalWrite(BLE_POWER_PIN, HIGH);
      Serial.println("ACTIVATE BIONX COMPLETE");
    }
    else if (vehicleState == VEHICLE_ON && switchValue == HIGH) {
      if (startRideTimestamp > 0) {
        createEvent(EVENT_END_RIDE);
        pendingShutdownTimestamp = millis();
        vehicleState = VEHICLE_SHUTDOWN_PENDING;
        Serial.println("Shutdown Pending");
      }
      else {
        Serial.println("Immediate shutdown!");
        completeShutdown();
      }
    }
    
    lastButtonState = switchValue;
    delay(1); //debounce
  }
  
  if (vehicleState == VEHICLE_SHUTDOWN_PENDING && millis() - pendingShutdownTimestamp > 10000) {
    completeShutdown();
  }
  
  digitalWrite(SWITCH_LED_PIN, vehicleState == VEHICLE_ON);
}

void completeShutdown() {
  pendingShutdownTimestamp = 0;
  startRideTimestamp = 0;
  shutdownBionx(); // send the stop cmds to the battery and motor inverter if the switch is off while we were running
  modeo.shutdown();
  digitalWrite(BLE_POWER_PIN, LOW);
  Serial.println("SHUTDOWN BIONX COMPLETE");
}

/*
void constructBLESensors() {
  
  for (byte i = 0; i < NUM_SENSORS; i++) {
    modeo.registerSensor(i);
  }
}

void constructBLEProperties() {
  modeo.registerProperty(PROPERTY_SMOOTHING_MIN, 2, true);
  modeo.registerProperty(PROPERTY_SMOOTHING_MAX, 2, true);
  modeo.registerProperty(PROPERTY_STROKE_TIMEOUT_CYCLES, 2, true);
  modeo.registerProperty(PROPERTY_TORQUE_MULTIPLIER, 2, true);
  modeo.registerProperty(PROPERTY_RIDER_EFFORT_FILTER_STRENGTH, 2, true);
  modeo.registerProperty(PROPERTY_NUM_PROPERTIES, 2, true);
  modeo.registerProperty(PROPERTY_MAX_DAMPING_SPEED, 2, true);
  
  modeo.registerPropertyWithCallback(PROPERTY_ASSIST, 12, false, &assistDidChange);
  //modeo.registerPropertyWithCallback(PROPERTY_DAMPING, 12, false, &dampingDidChange);
  modeo.registerPropertyWithCallback(PROPERTY_EVENT, 17, false, &eventDidChange);
}
*/

void propertyDidChange(byte identifier, byte length, byte value[]) {
  switch(identifier) {
    case PROPERTY_ASSIST:
      assistDidChange(length, value);
      break;
    
    case PROPERTY_EVENT:
      eventDidChange(length, value);
      break;
      
    default:
      Serial.print("wtf?");
  }
}

void eventDidChange(byte length, byte value[]) {
  if (modeo.getByteValueForProperty(PROPERTY_EVENT) == EVENT_NO_EVENT) {
    if (vehicleState == VEHICLE_SHUTDOWN_PENDING) {
      completeShutdown();
    }
    
    Serial.println("No more event!");
    modeo.setValueForSensor(0, SENSOR_HAS_EVENT);
  }
}

void assistDidChange(byte length, byte value[]) {
  Serial.println("Assist Did Change!");
  
  point bottomLeft;
  bottomLeft.x = value[0];
  bottomLeft.y = value[1];
  
  point topRight;
  topRight.x = value[2];
  topRight.y = value[3];
  
  point point0;
  point0.x = value[4];
  point0.y = value[5];
  
  point point1;
  point1.x = value[6];
  point1.y = value[7];
  
  point point2;
  point2.x = value[8];
  point2.y = value[9];
  
  point point3;
  point3.x = value[10];
  point3.y = value[11];
  
  assist.points[0] = point0;
  assist.points[1] = point1;
  assist.points[2] = point2;
  assist.points[3] = point3;
  
  assist.maxX = topRight.x;
  assist.maxY = topRight.y;
  
  assist.cacheIsValid = false;
}

/*
void dampingDidChange(byte length, byte value[]) {
  Serial.println("Damping Did Change!");
  
  point bottomLeft;
  bottomLeft.x = value[0];
  bottomLeft.y = value[1];
  
  point topRight;
  topRight.x = value[2];
  topRight.y = value[3];
  
  point point0;
  point0.x = value[4];
  point0.y = value[5];
  
  point point1;
  point1.x = value[6];
  point1.y = value[7];
  
  point point2;
  point2.x = value[8];
  point2.y = value[9];
  
  point point3;
  point3.x = value[10];
  point3.y = value[11];
  
  damping.points[0] = point0;
  damping.points[1] = point1;
  damping.points[2] = point2;
  damping.points[3] = point3;
  
  damping.maxX = topRight.x;
  damping.maxY = topRight.y;
  
  damping.cacheIsValid = false;
} */

