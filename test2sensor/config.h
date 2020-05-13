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

//ymotor
#define Y_STEP_BIT  -1  
#define Y_ENABLE_BIT  6
#define Y_DIRECTION_BIT    7 //this is pin for direction control of axis
#define ENDSTOPY_PIN  9 //this makes sense for x axis as it is a rotating table more than likely.
#define Y_STEP_DIR      0 //1 or 0 this controlls direction of motor in case it is backwards

//zmotor
#define Z_STEP_BIT      4  
#define Z_ENABLE_BIT 2
#define Z_DIRECTION_BIT   3  
#define ENDSTOPZ_PIN 5 //if no end stop on an axis set to -1. homing will do nothing and leave axis in its current position
#define Z_STEP_DIR      0 //1 or 0 this controlls direction of motor in case it is backwards

//stepper logic enable and directional settings
#define BIT_DIR_FOR_STEP 0 //0 start low pulse, end high pulse, 1 start high pulse, end low pulse
#define BIT_ENABLE 0 //0 is low, 1 is high, usually a low will activate stepper driver

//other limits and move above homing switch if driver endstop is set
#define Z_STEPS_PER_MM float(141.732283465*16) // . 200/(25.4/18) steps_per_turn/(inch_to_mm/18turns per inch_to_mm ) using 5/16 rod 18tpi
#define Y_STEPS_PER_DEG float((200*16)/360)    // 200STEPS PER ROTATION/360 DEG IN A CIRCLE X16 MICROSTEPS
#define X_STEPS_PER_DEG float((200*16*26.85)/360)    // 200STEPS PER ROTATION/360 DEG IN A CIRCLE X16 MICROSTEPS gear reduction 26.85. it is possible that this is a geared motor
//soft limits 
#define ZMAX_HIEGH_MM 120 //this changes depending on design
#define YMAX_WIDTH_DEG 260 //this changes depending on design
#define XMAX_WIDTH_DEG 180 //this changes depending on design


//this moves away set distance from homing switch after a home is initiated if stepper endstop is defined, otherwise it just moves distance
// if negative number we go foward that much and then back the same amount. for now only an option on x axis because it has gearbox
#define Z_START_ABOVE_HOME_MM 2 //
#define Y_START_ABOVE_HOME_MM 90 //should start up on ladar sensor 
#define X_START_ABOVE_HOME_MM -20 //we move x axis to show it is working. if it is working correctly it will rotate counter clockwise

//acceleration settings
#define maxspeednoacellerationZ 72 // this is speed motor can stop and start 
#define maxspeednoacellerationY 1000// it is shorter for faster moves
#define maxspeednoacellerationX 1000 //we keep low for testing, but might speed it up later on

//this is max speed with speed ramp
#define SpeedwithaccelerationZ 50
#define SpeedwithaccelerationY 800
#define SpeedwithaccelerationX 100

//rate of change for ramping. larger number is faster
#define accerate_rateZ 1 //rate of change for ramping from 1 to 10 for example
#define accerate_rateY 1 //0 will not use acelleration
#define accerate_rateX 1 //0 will not use acelleration


//this is how many steps towards end of motion of a stepper to start slow down.
//this is only to reduce wear on parts and keep z axis from shaking appart. this makes stops and direction change deaccelerate first before pausing for jerk time in Milliseconds.
#define deacceleration_stepsX 180 //this is steps remaining that a slow down starts. this is delay in Microseconds per step added at rate of accerate_rate
#define deacceleration_stepsY 10 //this is steps remaining that a slow down starts
#define deacceleration_stepsZ 600 //this is steps remaining that a slow down starts
//this is noise reduction it vaires speed up and down a little to prevent resonance and reduce vibrations.
#define phase 4 //this is how far in and out frequency gets to reduce noise
#define phase_speed 2;//this is how fast phase changes
#define backlashXmm 3 //we will have backlash on x to remove
#define backlashYmm 0
#define backlashZmm 0

#define powerdownX_when_not_moving true //this axis is geared and uses a lot of power. disabling is ok as it should hold in place it is a stage with wieght on it
#define powerdownY_when_not_moving  false //y is direct and is best if powered continuously as it moves a lot
#define powerdownZ_when_not_moving  true // this is threaded rod, and is used enough that it should only power down when device is not used.
//this is delay motor stops before switching durection. it prevents an inerta force from overpowering the motor when switching directions
//this is helpful for moves that are long, followed by a short move where deaccelleration ramp can not complete
#define jerkZ 5 //time still at direction change in ms. higher time needed for faster speed, or greater mass on an axis.
#define jerkY 30 //time still at direction change in ms
#define jerkX 5 //time still at direction change in ms
#define resDetail 0.01 //we use this as a guide for res detail that will need to be caluclated per axis



//sensor settings

#define RED_LASER_PIN 9
#define GREEN_LASER_PIN 8
#define BLUE_LASER_PIN  7  //note most blue lasers also have some red in them, this needs to be accounted for in measurement
#define photon_cell_analog_pin1 A0 //we need to use an analog pin for this. also each color is sensative a little differently. also there is responce time to consider. should be about 10k, maybe adjustable?
#define photon_cell_analog_pin2 A1 
//time of flight sensor pins
//we dont set pins as it is i2c. so A4, A5 pins scl, sca





//we need to dedein resolution of motors. this only needs to be changed if stepper has different resolution than default 200steps per revolution. 
//for example this is 200steps x16 microsteps
#define MotordegreesX 1600
#define MotordegreesY 1600
#define MotordegreesZ 1600

//this defines position of lasers and sensors on y axis
#define RED_LASER_POS 33
#define GREEN_LASER_POS 10
#define BLUE_LASER_POS  60  
#define ladar_pos 91



//calculated static values that never change during execution. we do here to save memory or increase speed or both
#define max_size_z (Z_STEPS_PER_MM *ZMAX_HIEGH_MM)
#define max_size_y (Y_STEPS_PER_DEG * YMAX_WIDTH_DEG)
#define max_size_x (X_STEPS_PER_DEG * XMAX_WIDTH_DEG)
//we make things more human for planner
#define Z_axis Z_STEP_BIT
#define Y_axis Y_STEP_BIT
#define X_axis X_STEP_BIT 
