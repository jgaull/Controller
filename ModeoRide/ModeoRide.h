//   IO PIN DEFINITIONS
#define CAN_READY_PIN 2
#define ON_OFF_SWITCH_PIN 5
#define INDICATOR_LED_PIN 6
#define WAKE_RELAY_PIN 7
#define SWITCH_LED_PIN 4

//	State definitions
#define TRQ_ASSIST_OFF 0
#define TRQ_ASSIST_ON 1

#define VEHICLE_OFF 0
#define VEHICLE_ON 1

//  Constants for configuration of the strain characterization algorithm
#define MAX_DAMPING_MULTIPLIER 1
#define RESOLUTION 10
#define STANDARD_ASSIST 0
#define THREE_DIMENSIONAL_MAPPING 1
#define EFFORT_MAPPING 2

//***Time Management Variables - ALL VALUES IN MICROSECONDS
#define BLE_TX_DELTA 10000
#define BLE_FAST_DELTA 50000
#define BLE_MED_DELTA 200000
#define BLE_SLOW_DELTA 1000000


#define MEDIUM_TX_LISTLEN 10
#define SLOW_TX_LISTLEN  10

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
