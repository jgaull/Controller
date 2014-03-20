//   IO PIN DEFINITIONS
#define CAN_READY_PIN 2
#define SWITCH_LED_PIN 4
#define ON_OFF_SWITCH_PIN 5
#define INDICATOR_LED_PIN 6
#define BLE_POWER_PIN 7

//	State definitions
#define TRQ_ASSIST_OFF 0
#define TRQ_ASSIST_ON 1

#define VEHICLE_OFF 0
#define VEHICLE_ON 1

//  Constants for configuration of the strain characterization algorithm
#define NUM_AVERAGED_STROKES 3
#define MAX_STORED_STROKES 3
#define MAX_DAMPING_MULTIPLIER 1
#define MAX_STROKE_LENGTH 30
#define RESOLUTION 10

//***Time Management Variables - ALL VALUES IN MICROSECONDS
#define BLE_TX_DELTA 10000
#define BLE_FAST_DELTA 50000
#define BLE_MED_DELTA 200000
#define BLE_SLOW_DELTA 1000000


#define MEDIUM_TX_LISTLEN 10
#define SLOW_TX_LISTLEN  10
//




struct canMsg {
  byte ID;
  byte DLC;
  byte data[4];
  byte freq;
  bool isPeriodic;
  bool dataReady;
};

struct PedalStroke {
  byte length;
  byte index;
  byte runs;
  byte strokeId;
  byte data[MAX_STROKE_LENGTH];
};

struct Property {
  uint16_t value;
  boolean eepromSave;
  boolean pendingSave;
};

struct Sensor {
  byte dataIdentifier;
  uint16_t value;
  byte propertyAddress;
  boolean isFresh;
};

 struct point
{
    byte x;
    byte y;
};

//Helpful
#define UINT16_MAX 65535
