

//   IO PIN DEFINITIONS
#define CAN_READY_PIN 2
#define ON_OFF_SWITCH_PIN 5
#define INDICATOR_LED_PIN 6
#define WAKE_RELAY_PIN 7
#define SWITCH_LED_PIN 4
//   END IO PINS

#define UINT16_MAX 65535


#define MAX_STROKE_LENGTH 30





//	State definitions
#define TRQ_ASSIST_OFF 0
#define TRQ_ASSIST_ON 1
//	End state definitions

//  Constants for configuration of the strain characterization algorithm
#define NUM_AVERAGED_STROKES 3
#define MAX_STORED_STROKES 3
#define MAX_DAMPING_MULTIPLIER 1
#define SMOOTHING_DIVISOR 65535
#define RESOLUTION 10
//   END Constants for configuration of the strain characterization algorithm


#define CMD_MTR_0x02_ID         0x02    //  unknown cmd sent at start
#define CMD_MTR_TRQ_ID          0x09    //  torque cmd
#define MTR_SPEED_ID 		0x11    //  motor speed data req
#define MTR_0x12_ID             0x12    //  unknown data req, sent at ~1sec
#define MTR_TRQ_ID              0x14    //  real torque reported by motor
#define MTR_TEMP_ID             0x16    //  unknown data req, sent at ~1sec
#define MTR_0x20_ID     	0x20    //  unknown data req, sent at startup, always 0x64 resp
#define MTR_PEDSTRAIN_ID 	0x21    //  strain from the bionx torque sensor
#define CMD_MTR_0x22_ID 	0x22    //  unknown cmd, set at start with 00 00 byte2-3
#define CMD_BATT_SHTDWN_ID      0x25    //  the shutdown command to the battery
#define BATT_V_ID               0x32    //  
#define CMD_0x41_ID             0x41
#define CMD_0x42_ID             0x42






//  END  MESSAGE IDS


// CAN message lists
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
