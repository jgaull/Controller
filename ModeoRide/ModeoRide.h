

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
#define MAX_STORED_STROKES 3
#define MAX_DAMPING_MULTIPLIER 1
#define SMOOTHING_DIVISOR 65535
#define RESOLUTION 100
//   END Constants for configuration of the strain characterization algorithm




/*

//   MESSAGE IDS
//  Commands are 4 bytes sent by controller, data requests are 2 bytes sent, 4 recd

//  CMDS+DATA to/from BATTERY
//
#define CMD_BATT_SHTDWN_ID      0x25 

9A
AA
32


61
61
30
33

~1S



#define SIG_BATT_0x3C_ID        0x25   
#define CMD_BATT_SHTDWN_ID      0x25 

21
22
25
30
31
32
33
61
80
1D
3B





//  CMDS+DATA

*/


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

#define DAT_MTR_SPD     0
#define DAT_MTR_TRQ     1
#define DAT_MTR_TMP     2
#define DAT_RID_TRQ     3
#define DAT_MTR_0x70    4
#define DAT_MTR_0x72    5
#define DAT_BAT_VBAT    6
#define DAT_BAT_0xAA    7
#define DAT_BAT_0x9A    8
#define DAT_INV_0x12    9
#define DAT_INV_0x20    10
#define DAT_INV_0x92    11
#define DAT_INV_0x6C    12
#define DAT_BAT_0x30    13
#define DAT_BAT_0x31    14
#define DAT_BAT_0x33    15
#define DAT_BAT_0x61    16
#define DAT_BAT_0x80    17
#define DAT_BAT_0x1D    18
#define DAT_BAT_0x3B    19
#define DAT_BAT_0x3C    20
#define DAT_BAT_0x3D    21
#define DAT_BAT_0xA1    22
#define DAT_BAT_0xA2    23
#define DAT_BAT_0xA3    24
#define DAT_BAT_0xA4    25
#define DAT_BAT_0xA5    26
#define DAT_BAT_0xF0    27
#define DAT_BAT_0xF9    28
#define DAT_BAT_0xFA    29
#define DAT_BAT_0xFB    30
#define DAT_BAT_0xFC    31





//  END  MESSAGE IDS


// CAN message lists
//***Time Management Variables - ALL VALUES IN MICROSECONDS
#define CANMSG_TX_DELTA 2000

#define FAST_TX_DELTA  10000
#define MEDIUM_TX_DELTA 50000
#define SLOW_TX_DELTA 1000000
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

struct Sensor {
  byte dataIdentifier;
  byte stateIdentifier;
  uint16_t value;
  boolean state;
  boolean isFresh;
};

struct Property {
  uint16_t value;
  byte bleIdentifier;
  byte eepLSB;
  byte eepMSB;
};

 struct point
{
    float x;
    float y;
};
