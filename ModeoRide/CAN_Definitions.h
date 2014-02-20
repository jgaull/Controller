
unsigned long fastTxStamp = 0;
unsigned long mediumTxStamp = 0;
unsigned long slowTxStamp = 0;
unsigned long torqueTxStamp = 0;

#define CANMSG_TX_DELTA   3000
#define FAST_TX_DELTA     10000
#define MEDIUM_TX_DELTA   50000
#define SLOW_TX_DELTA     1000000
#define TORQUE_TX_DELTA   10000

unsigned long canMsgTxStamp = 0;

bool fastTxFlag = 0;
bool mediumTxFlag = 0;
bool slowTxFlag = 0;

byte mediumTxPointer = 0;
byte slowTxPointer = 0;
byte fastTxPointer = 0;

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

PROGMEM prog_uchar fastTxMsgs[][3] = {  // ID, DLC, sigID
  {0x20, 0x02, 0x11},
  {0x20, 0x02, 0x0A},
  {0x20, 0x02, 0x21},
};

byte fastTxData [(sizeof(fastTxMsgs) / sizeof(fastTxMsgs[0]))] = {0};

PROGMEM prog_uchar mediumTxMsgs[][3] = {  // ID, DLC, sigID
//  {0x20, 0x02, 0x21},
  {0x58, 0x02, 0x9C},
  {0x10, 0x02, 0x32},
  {0x10, 0x02, 0x9A},
  {0x10, 0x02, 0xAA},
  {0x20, 0x02, 0x14},
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




