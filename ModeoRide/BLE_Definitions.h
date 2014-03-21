//Sensors
#define SENSOR_RIDER_EFFORT 0
#define SENSOR_CURRENT_STRAIN 1
#define SENSOR_SPEED 2
#define SENSOR_RAW_STRAIN 3
#define SENSOR_TORQUE_APPLIED 4
#define SENSOR_MOTOR_TEMP 5
#define SENSOR_BATTERY_VOLTAGE 6
#define SENSOR_POWER_OUTPUT 7
#define SENSOR_STROKE_LENGTH 8
#define SENSOR_FILTERED_RIDER_EFFORT 9

#define NUM_SENSORS 10 //Is there a better way to do this?

//Properties
#define PROPERTY_SMOOTHING_MIN 0
#define PROPERTY_SMOOTHING_MAX 1
#define PROPERTY_MAX_OUTPUT 2
#define PROPERTY_STROKE_TIMEOUT_CYCLES 3
#define PROPERTY_MAX_EFFORT 4
#define PROPERTY_TORQUE_MULTIPLIER 5
#define PROPERTY_MAX_STRAIN_DAMPING_SPEED 6
#define PROPERTY_STRAIN_DAMPING_CONTROL1_X 7
#define PROPERTY_STRAIN_DAMPING_CONTROL1_Y 8
#define PROPERTY_STRAIN_DAMPING_CONTROL2_X 9
#define PROPERTY_STRAIN_DAMPING_CONTROL2_Y 10
#define PROPERTY_SENSOR_RIDER_EFFORT_STATE 11
#define PROPERTY_SENSOR_CURRENT_STRAIN_STATE 12
#define PROPERTY_SENSOR_SPEED_STATE 13
#define PROPERTY_SENSOR_RAW_STRAIN_STATE 14
#define PROPERTY_SENSOR_TORQUE_APPLIED_STATE 15
#define PROPERTY_SENSOR_MOTOR_TEMP_STATE 16
#define PROPERTY_SENSOR_BATTERY_VOLTAGE_STATE 17
#define PROPERTY_ASSIST_1_X 18
#define PROPERTY_ASSIST_1_Y 19
#define PROPERTY_ASSIST_2_X 20
#define PROPERTY_ASSIST_2_Y 21
#define PROPERTY_REGEN_1_X 22
#define PROPERTY_REGEN_1_Y 23
#define PROPERTY_REGEN_2_X 24
#define PROPERTY_REGEN_2_Y 25
#define PROPERTY_SENSITIVITY_1_X 26
#define PROPERTY_SENSITIVITY_1_Y 27
#define PROPERTY_SENSITIVITY_2_X 28
#define PROPERTY_SENSITIVITY_2_Y 29
#define PROPERTY_MAX_SPEED 30
#define PROPERTY_SENSOR_POWER_OUTPUT_STATE 31
#define PROPERTY_SENSOR_STROKE_LENGTH_STATE 32
#define PROPERTY_RIDER_EFFORT_FILTER_STRENGTH 33
#define PROPERTY_SENSOR_FILTERED_RIDER_EFFORT_STATE 34
#define PROPERTY_FANCY_ASSIST_STATE 35

#define NUM_PROPERTIES 36 //Is there a better way to do this?

//Used to sync iOS to Arduino
#define REQUEST_CONNECT 0
#define REQUEST_DISCONNECT 1
#define REQUEST_GET_PROPERTY_VALUE 2
#define REQUEST_SET_PROPERTY_VALUE 3
#define REQUEST_ADD_BEZIER 4
#define REQUEST_GET_SENSOR_VALUE 5

//Bezier Calls

#define FIRST_PROPERTY_IDENTIFIER 0
#define FIRST_SENSOR_IDENTIFIER 150
#define FIRST_EVENT_IDENTIFIER 175
