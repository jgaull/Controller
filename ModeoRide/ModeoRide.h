//IO PIN DEFINITIONS
#define CAN_READY_PIN 2
#define SWITCH_LED_PIN 4
#define ON_OFF_SWITCH_PIN 5
#define INDICATOR_LED_PIN 6
#define BLE_POWER_PIN 7

//Vehicle State
#define VEHICLE_OFF 0
#define VEHICLE_ON 1

//Constants for configuration of the strain characterization algorithm
#define MAX_DAMPING_MULTIPLIER 1
#define RESOLUTION 10
#define STANDARD_ASSIST 0
#define THREE_DIMENSIONAL_MAPPING 1
#define EFFORT_MAPPING 2

//Bezier curve types
#define CURVE_TYPE_ASSIST 0
#define CURVE_TYPE_DAMPING 1
#define CURVE_TYPE_REGEN 2

/*
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
*/

/*
struct point {
    byte x;
    byte y;
};
*/

/*
struct Bezier {
  byte type;
  byte numPoints;
  point points[4];
  point cache[RESOLUTION];
  boolean cacheIsValid;
  byte maxX;
  byte maxY;
};*/

//Helpful
#define UINT16_MAX 65535
#define BYTE_MAX 255
