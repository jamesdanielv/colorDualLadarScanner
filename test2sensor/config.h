//this is used to configure device and pins
//designed by james villeneuve for motor postitioning system that uses little memory.
//this version does not yet have a bresanham interpolation algorithim for interpolating moves, but it does have a planner
//you can know exact position and go back to it every time.
#define leftSensor_disable_pin A6 //both sensors share same address
#define rightSensor_disable_pin A7 //we can initialize both at same time, but disable one at a time
#define max_steps_per_second 82000 //this is used to change calc timings in moves per second in mm. this is based on current rates
#define mm_p_secondx_max_1_x //these numbers are turned into raw calc steps and delays further down in file
#define mm_p_secondx_ma_w_acc_x 1 //these numbers are turned into raw calc steps and delays further down in file
#define mm_p_secondx_max_1_y //these numbers are turned into raw calc steps and delays further down in file
#define mm_p_secondx_ma_w_acc_y 1 //these numbers are turned into raw calc steps and delays further down in file
#define mm_p_secondx_max 1_z //these numbers are turned into raw calc steps and delays further down in file
#define mm_p_secondx_ma_w_acc_z 1 //these numbers are turned into raw calc steps and delays further down in file
//xmotor
#define X_STEP_BIT 12 //12 //if this is set to -1 the entire x axis is not setup
#define X_ENABLE_BIT 10
#define X_DIRECTION_BIT 11 //this is pin for direction control of axis
#define ENDSTOPX_PIN -1 //if this is set to -1 homing will do nothing and no homing pin will be used
#define X_STEP_DIR      1 //1 or 0 this controlls direction of motor in case it is backwards
