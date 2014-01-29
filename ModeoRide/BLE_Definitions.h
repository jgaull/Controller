#define SENSOR_RIDER_EFFORT 0
#define SENSOR_CURRENT_STRAIN 1
#define SENSOR_SPEED 2
#define SENSOR_RAW_STRAIN 3
#define SENSOR_TORQUE_APPLIED 4
#define SENSOR_MOTOR_TEMP 5
#define SENSOR_BATTERY_VOLTAGE 6

#define NUM_SENSORS 7

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
#define TORQUE_MULTIPLIER_BYTE 0x2E

#define ENABLE_RIDER_EFFORT_UPDATES_BYTE 0x2D
#define ENABLE_CURRENT_STRAIN_UPATES_BYTE 0x2F
#define ENABLE_SPEED_UPDATES_BYTE 0x3A
#define ENABLE_RAW_STRAIN_UPDATES_BYTE 0x3B
#define ENABLE_TORQUE_APPLIED_UPDATES_BYTE 0x3C
#define ENABLE_MOTOR_TEMP_UPDATES_BYTE 0x3D
#define ENABLE_BATTERY_VOLTAGE_BYTE 0x3E

//Values sent from sensors to iOS
#define RIDER_EFFORT_BYTE 0x0A
#define CURRENT_STRAIN_BYTE 0x0B
#define SPEED_BYTE 0x0C
#define RAW_STRAIN_BYTE 0x0D
#define TORQUE_APPLIED_BYTE 0x0E
#define MOTOR_TEMP_BYTE 0x0F
#define BATTERY_VOLTAGE_BYTE 0xAA

//Used to sync iOS to Arduino
#define SEND_PARAMS_BYTE 0xEE
