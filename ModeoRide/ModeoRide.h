

//   IO PIN DEFINITIONS
#define CAN_READY_PIN 2
#define ON_OFF_SWITCH_PIN 5
#define INDICATOR_LED_PIN 6
#define WAKE_RELAY_PIN 7
#define MARKER_PIN 4
//   END IO PINS

#define UINT16_MAX 65535


#define MAX_STROKE_LENGTH 30





//	State definitions
#define TRQ_ASSIST_OFF 0
#define TRQ_ASSIST_ON 1
//	End state definitions

//  Constants for configuration of the strain characterization algorithm
#define NUM_AVERAGED_STROKES 3
#define MAX_STORED_STROKES 6
#define MAX_DAMPING_MULTIPLIER 1
#define SMOOTHING_DIVISOR 65535
#define MAX_STRAIN_DAMPING_CURVE 10
//   END Constants for configuration of the strain characterization algorithm

//Constants for BLE properties
#define NUM_AVERAGED_STROKES_BYTE 0x1A
#define MAX_STORED_STROKES_BYTE 0x1B
#define MAX_STRAIN_DAMPING_SPEED_BYTE 0x1C
#define MAX_OUTPUT_BYTE 0x1D
#define STRAIN_DAMPING_CURVE_BYTE 0x1E
#define STROKE_TIMEOUT_CYCLES_BYTE 0x1F
#define SMOOTHING_MIN_BYTE 0x2A
#define SMOOTHING_MAX_BYTE 0x2B
#define MAX_EFFORT_BYTE 0x2C
#define ENABLE_RIDER_EFFORT_UPDATES_BYTE 0x2D
#define TORQUE_MULTIPLIER_BYTE 0x2E
#define ENABLE_CURRENT_STRAIN_UPDATES_BYTE 0x2F

//Values sent from sensors to iOS
#define RIDER_EFFORT_BYTE 0x0A
#define CURRENT_STRAIN_BYTE 0x0B

//Used to sync iOS to Arduino
#define SEND_PARAMS_BYTE 0xEE



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
  int data[MAX_STROKE_LENGTH];
};


