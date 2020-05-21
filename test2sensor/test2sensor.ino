//designed by james villeneuve March 2020  
//uses software library to address issues with single i2c address for ladar parts as we can have 2 channels for soft i2c
//frustrated with optical scanners that wont work reliably all the time, this one will even work outside!
//this tests that motors are enabled and change direction, and verifies that evereything seems to work
//this program will allow talking over serial, and home motors, and do pixel color measurements in r,g,b, and show 0.1resolution distance from 2 time of flight chip
//important for custom boards. whatever board is used needs some analog pins, and needs to be arduino based
// tested on arduino uno, nano
//pins used for driving motors
//if a motor driver is not to be used for example a different design, or testing then set stepper pin for motor to pin -1
//Z AXIS IS SCALLED. MOST LIKELY BY A 5/16 ROD WITH 18 TURNS PER INCH OR 25.4/18 OR 1.411MM PER TURN

#include "config.h"
#include "scanner_driver.h" //we want to keep things clean so we seperate out processes

//thes are required for sensor read



float step_count =0;//this is what we use to test x axis
void setup() {
    Serial.flush();
Serial.begin(112500);
 

for (int i=0;i<128;i++){Serial.println();}//like a clear screen
// wait for serial port to open on native usb devices
Serial.println("initializing both sensors");

 
 delay(100);
  while (!Serial) {
    delay(1);
  }
  
  Serial.println("Adafruit VL6180x test!");
#if disable_scanner_for_troubleshooting != true
  if (sensor1.VL6180xInit() != 0){
    Serial.println("Failed to find sensor");
    while (1);
    


  }
#else
    Serial.print("sensor disabled from config.h");
  
#endif
  delay(100);
#if disable_scanner_for_troubleshooting != true
sensor1.VL6180xDefautSettings(); //Load default settings to get started.
#endif
  delay(100);
  Serial.println("Adafruit VL6180x test!");
#if disable_scanner_for_troubleshooting != true
  if (sensor2.VL6180xInit() != 0){
    Serial.println("Failed to find sensor");
    while (1);

  }
#else
    Serial.print("sensor disabled from config.h");
#endif
  
#if disable_scanner_for_troubleshooting != true
  Serial.println("Sensor found!");
delay(100);
sensor2.VL6180xDefautSettings(); //Load default settings to get started.
 delay(100);
#endif
init_lasers_and_photo_sensor();

steppers_init();//we initalize outputs and pins
//we enable steppers

enable_stepper(X_ENABLE_BIT);//ENABLE //disabels

enable_stepper(Y_ENABLE_BIT);//ENABLE //disabels

enable_stepper(Z_ENABLE_BIT);//ENABLE //disabels




//we want to move z to zero, we assume -ZMAX_HIEGH_MM*2 is the value assigned once homed if homeing is true
//move_axis(Z_STEP_BIT,Z_STEP_DIR,-(ZMAX_HIEGH_MM*2),maxspeednoacellerationZ,SpeedwithaccelerationZ,accerate_rateZ,deacceleration_stepsZ,true,ENDSTOPZ_PIN);
//for example homing Z is move_axis(0.0,72,maxspeednoacellerationZ,SpeedwithaccelerationZ,accerate_rateZ,true);
//while(1);//we wait

//we know home by sending 'h'
//run_homing_check_X();//we want a  position for x. since no homeing we set at zero
//run_homing_check_Y();//we want y to be homed
//run_homing_check_Z();//we want z to be homed

//test a port

 // float lux = vl.readLux(VL6180X_ALS_GAIN_5);//we set gain on ladar
}

void loop() {
 
  delay(100);
check_serial();

  delay(100);
  
 // Serial.print("cds_photo_cell_value");Serial.println(analogRead(photon_cell_analog_pin));
 // move_Axis(Z_axis, ZMAX_HIEGH_MM);//we move z axis up full distance
 //  step_count+=10;//we move 10 degrees each cycle
  // move_Axis(X_axis, step_count);
  delay(100);
  check_serial();
  
  delay(100);
//  move_Axis(Z_axis, Z_START_ABOVE_HOME_MM);//we move z axis up full distance
//  step_count+=10;//we move 10 degrees each cycle
 //  move_Axis(X_axis, step_count);
}
