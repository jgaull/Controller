//IO PIN DEFINITIONS
#define CAN_READY_PIN 2
#define SWITCH_LED_PIN 4
#define ON_OFF_SWITCH_PIN 5
#define INDICATOR_LED_PIN 6
#define BLE_POWER_PIN 7

//Vehicle State
#define VEHICLE_OFF 0
#define VEHICLE_ON 1
#define VEHICLE_SHUTDOWN_PENDING 2

//Constants for configuration of the strain characterization algorithm
#define STANDARD_ASSIST 0
#define THREE_DIMENSIONAL_MAPPING 1
#define EFFORT_MAPPING 2

//Bezier Curves
#define CURVE_TYPE_ASSIST 0
#define CURVE_TYPE_DAMPING 1

//#define NUM_BEZIERS 2

//Sensors
#define SENSOR_RIDER_EFFORT 0
#define SENSOR_CURRENT_STRAIN 1
#define SENSOR_SPEED 2
#define SENSOR_RAW_STRAIN 3
#define SENSOR_TORQUE_APPLIED 4
#define SENSOR_MOTOR_TEMP 5
#define SENSOR_BATTERY_VOLTAGE 6
#define SENSOR_FILTERED_RIDER_EFFORT 7
#define SENSOR_HAS_EVENT 8
#define SENSOR_BATTERY_PERCENTAGE 9

#define NUM_SENSORS 9 //Is there a better way to do this?

//Properties
#define PROPERTY_SMOOTHING_MIN 0
#define PROPERTY_SMOOTHING_MAX 1
#define PROPERTY_STROKE_TIMEOUT_CYCLES 2
#define PROPERTY_TORQUE_MULTIPLIER 4
#define PROPERTY_RIDER_EFFORT_FILTER_STRENGTH 12
#define PROPERTY_ASSIST 16
#define PROPERTY_MAX_DAMPING_SPEED 18
#define PROPERTY_EVENT 19

//#define NUM_PROPERTIES 9 //Is there a better way to do this?

//event identifiers
#define EVENT_NO_EVENT 0
#define EVENT_START_RIDE 1
#define EVENT_END_RIDE 2

struct point {
    byte x;
    byte y;
};

#define RESOLUTION 4

struct Bezier {
    point points[4];
    point cache[RESOLUTION];
    boolean cacheIsValid;
    byte maxX;
    byte maxY;
};

//Helpful
#define UINT16_MAX 65535
#define BYTE_MAX 255
