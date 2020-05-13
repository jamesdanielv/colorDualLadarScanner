#include "config.h"
#include "variables.h"
#include "SparkFun_VL6180X.h"

#define VL6180X_ADDRESS 0x29
#define VL6180X_ADDRESS2 0x29

//VL6180xIdentification identification;

VL6180x1 sensor1(VL6180X_ADDRESS,A4,A5);  // create both sensors as 
VL6180x2 sensor2(VL6180X_ADDRESS,A2,A3);  // create both sensors

void disable_stepper(int stepper_accessed){
digitalWrite(stepper_accessed,!BIT_ENABLE);
  Serial.println("stepper should be disabled!");
}
void enable_stepper(int stepper_accessed){
digitalWrite(stepper_accessed,BIT_ENABLE);
 Serial.print (stepper_accessed);
  Serial.println("stepper should be enabled!");
}








steppers_init(){
//if step bit is -1 we dont init that driver
#if Z_STEP_BIT != -1
pinMode(ENDSTOPZ_PIN,INPUT);
digitalWrite(ENDSTOPZ_PIN, HIGH);       // turn on pullup resistors
pinMode( Z_DIRECTION_BIT,OUTPUT);
pinMode( Z_STEP_BIT,OUTPUT);
pinMode(Z_ENABLE_BIT, OUTPUT); 
 Serial.println("Z stepper should be initialized");
 Serial.print("Z_STEP_BIT="); Serial.println(Z_STEP_BIT);
  Serial.print("ENDSTOPZ_PIN="); Serial.println(ENDSTOPZ_PIN);
   Serial.print("Z_DIRECTION_BIT="); Serial.println(Z_DIRECTION_BIT);
   Serial.print("Z_STEP_BIT="); Serial.println(Z_STEP_BIT);
    Serial.print("BIT_ENABLE="); Serial.println(BIT_ENABLE);
   Serial.print("Z_ENABLE_BIT="); Serial.println(Z_ENABLE_BIT);  

#endif


#if Y_STEP_BIT != -1
pinMode(ENDSTOPY_PIN,INPUT);
digitalWrite(ENDSTOPY_PIN, HIGH);       // turn on pullup resistors
pinMode( Y_DIRECTION_BIT,OUTPUT);
pinMode( Y_STEP_BIT,OUTPUT);
pinMode(Y_ENABLE_BIT, OUTPUT); 
Serial.println("Y stepper should be initialized");
 Serial.print("Y_STEP_BIT="); Serial.println(Y_STEP_BIT);
  Serial.print("ENDSTOPY_PIN="); Serial.println(ENDSTOPY_PIN);
   Serial.print("Y_DIRECTION_BIT="); Serial.println(Y_DIRECTION_BIT);
   Serial.print("Y_STEP_BIT="); Serial.println(Y_STEP_BIT);
    Serial.print("BIT_ENABLE="); Serial.println(BIT_ENABLE);
   Serial.print("Y_ENABLE_BIT="); Serial.println(Y_ENABLE_BIT);
Serial.print("max_size_y"); Serial.println(max_size_y);
#endif

#if  X_STEP_BIT != -1
pinMode(ENDSTOPX_PIN,INPUT);
digitalWrite(ENDSTOPX_PIN, HIGH);       // turn on pullup resistors
pinMode( X_DIRECTION_BIT,OUTPUT);
pinMode( X_STEP_BIT,OUTPUT);
pinMode(X_ENABLE_BIT, OUTPUT); 
Serial.println("X stepper should be initialized");
 Serial.print("X_STEP_BIT="); Serial.println(X_STEP_BIT);
  Serial.print("ENDSTOPX_PIN="); Serial.println(ENDSTOPX_PIN);
   Serial.print("X_DIRECTION_BIT="); Serial.println(X_DIRECTION_BIT);
   Serial.print("X_STEP_BIT="); Serial.println(X_STEP_BIT);
    Serial.print("BIT_ENABLE="); Serial.println(BIT_ENABLE);
   Serial.print("X_ENABLE_BIT="); Serial.println(X_ENABLE_BIT);

  #endif

   


}



void run_homing_check_Y(){
 
int  accellerate=maxspeednoacellerationY-SpeedwithaccelerationY;
  // put your main code here, to run repeatedly:
 digitalWrite( Y_DIRECTION_BIT,!Y_STEP_DIR);
delayMicroseconds(10);
  for (float i=0;i<max_size_y*2;i++){
    if (accellerate !=0){   accellerate-=accerate_rateY;}
delayMicroseconds(maxspeednoacellerationY+phaseControl*4);


if (phasePolarity){phaseControl=phaseControl+phase_speed;if (phaseControl>phase){phasePolarity=0;};}
if (!phasePolarity){phaseControl=phaseControl-phase_speed;if (phaseControl<-phase){phasePolarity=1;};}

digitalWrite(Y_STEP_BIT,BIT_DIR_FOR_STEP);
delayMicroseconds(maxspeednoacellerationY+phaseControl*4);

digitalWrite(Y_STEP_BIT,!BIT_DIR_FOR_STEP);

if (digitalRead(ENDSTOPY_PIN)==HIGH){break;}//we stop loop

  }


 digitalWrite( Y_DIRECTION_BIT,Y_STEP_DIR);
delayMicroseconds(10);
  for (float i=0;i<Y_STEPS_PER_DEG *Y_START_ABOVE_HOME_MM;i++){//we go set amount above y
    if (accellerate !=0){   accellerate-=accerate_rateY;}
delayMicroseconds(maxspeednoacellerationY);

digitalWrite(Y_STEP_BIT,BIT_DIR_FOR_STEP);
delayMicroseconds(maxspeednoacellerationY);

digitalWrite(Y_STEP_BIT,!BIT_DIR_FOR_STEP);

  }
//direction change

 delay(jerkY);
//we now need to set planner at current assumed position
y_pos=Y_START_ABOVE_HOME_MM*1048576;//planer is current number float x 1048576 for float precision in an int number
  
}
void run_homing_check_X(){
  if  (powerdownX_when_not_moving){digitalWrite(X_ENABLE_BIT,BIT_ENABLE);}
  float tempdistance=X_START_ABOVE_HOME_MM;
  if (X_START_ABOVE_HOME_MM<0){tempdistance=-X_START_ABOVE_HOME_MM;}//we make sure number used used is posative, because it is possible for it to be negative for forward and reverse motion at start
;
 digitalWrite( X_DIRECTION_BIT,X_STEP_DIR);
delayMicroseconds(10);
  for (float i=0;i<X_STEPS_PER_DEG *tempdistance;i++){//we go set amount above y
    
delayMicroseconds(maxspeednoacellerationX);

digitalWrite(X_STEP_BIT,BIT_DIR_FOR_STEP);
delayMicroseconds(maxspeednoacellerationX);

digitalWrite(X_STEP_BIT,!BIT_DIR_FOR_STEP);

  }

   delay(jerkX);

   if(backlashXmm>0){//we run this because we are changind direction
//we dont use acceleration for backlash
 digitalWrite( X_DIRECTION_BIT,!X_STEP_DIR);
delayMicroseconds(10);
  for (float i=0;i<X_STEPS_PER_DEG *backlashXmm;i++){//we go set amount above y

delayMicroseconds(maxspeednoacellerationX);

digitalWrite(X_STEP_BIT,BIT_DIR_FOR_STEP);
delayMicroseconds(maxspeednoacellerationX);

digitalWrite(X_STEP_BIT,!BIT_DIR_FOR_STEP);

  }

   delay(jerkX); 
   }
if (X_START_ABOVE_HOME_MM<0){

 digitalWrite( X_DIRECTION_BIT,!X_STEP_DIR);
delayMicroseconds(10);
  for (float i=0;i<X_STEPS_PER_DEG *tempdistance;i++){//we go set amount above y
  
delayMicroseconds(maxspeednoacellerationX);

digitalWrite(X_STEP_BIT,BIT_DIR_FOR_STEP);
delayMicroseconds(maxspeednoacellerationX);

digitalWrite(X_STEP_BIT,!BIT_DIR_FOR_STEP);

  }
 delay(jerkX);;//if we move back we still need a stop pause
 if  (powerdownX_when_not_moving){digitalWrite(X_ENABLE_BIT,!BIT_ENABLE);}
}
  

   
//we now need to set planner at current assumed position
x_pos=X_START_ABOVE_HOME_MM*1048576;//planer is current number float x 16384 for float precision in an int number
 
}

void run_homing_check_Z(){
 if  (powerdownZ_when_not_moving){digitalWrite(Z_ENABLE_BIT,BIT_ENABLE);}
int  accellerate=maxspeednoacellerationZ-SpeedwithaccelerationZ;
  // put your main code here, to run repeatedly:
 digitalWrite( Z_DIRECTION_BIT,!Z_STEP_DIR);
delayMicroseconds(10);
  for (float i=0;i<max_size_z*2;i++){
    if (accellerate !=0){   accellerate-=accerate_rateZ;}
delayMicroseconds(maxspeednoacellerationZ+phaseControl*4);


if (phasePolarity){phaseControl=phaseControl+phase_speed;if (phaseControl>phase){phasePolarity=0;};}
if (!phasePolarity){phaseControl=phaseControl-phase_speed;if (phaseControl<-phase){phasePolarity=1;};}
digitalWrite(Z_STEP_BIT,BIT_DIR_FOR_STEP);

delayMicroseconds(maxspeednoacellerationZ+phaseControl*4);
digitalWrite(Z_STEP_BIT,!BIT_DIR_FOR_STEP);


if (digitalRead(ENDSTOPZ_PIN)==HIGH){break;}//we stop loop

  }


 digitalWrite( Z_DIRECTION_BIT,Z_STEP_DIR);
delayMicroseconds(10);
  for (float i=0;i<Z_STEPS_PER_MM *Z_START_ABOVE_HOME_MM;i++){//we go set amount above z
    if (accellerate !=0){   accellerate-=accerate_rateZ;}
delayMicroseconds(maxspeednoacellerationZ);
digitalWrite(Z_STEP_BIT,BIT_DIR_FOR_STEP);

delayMicroseconds(maxspeednoacellerationZ);
digitalWrite(Z_STEP_BIT,!BIT_DIR_FOR_STEP);


  }
//direction change

 delay(jerkZ);
//we now need to set planner at current assumed position
z_pos=Z_START_ABOVE_HOME_MM*1048576;//planer is current number float x 16384 for float precision in an int number
  
 if  (powerdownZ_when_not_moving){digitalWrite(Z_ENABLE_BIT,!BIT_ENABLE);} 
}



move(int axis,float steps_in_distance_mm){
//we start with known planner values
//int32_t x_pos;//we use longs and divide by 1048576 for positions so we have precision down beyond decimal
//int32_t y_pos;
//int32_t z_pos; 

int  accellerate=maxspeednoacellerationZ-SpeedwithaccelerationZ;
  // put your main code here, to run repeatedly:
 digitalWrite( Z_DIRECTION_BIT,Z_STEP_DIR);
 
delayMicroseconds(10);
int accelrate=accerate_rateZ;

  for (float i=0;i<max_size_z;i++){
 
    if (accellerate !=0){   accellerate-=accerate_rateZ;}
//delayMicroseconds(SpeedwithaccelerationZ+accellerate);
digitalWrite(Z_STEP_BIT,BIT_DIR_FOR_STEP);
if (max_size_z-i <deacceleration_stepsZ){accellerate +=accerate_rateZ+accerate_rateZ;}//this causes slow down at end


delayMicroseconds(SpeedwithaccelerationZ+accellerate+phaseControl);

if (phasePolarity){phaseControl=phaseControl+phase_speed;if (phaseControl>phase){phasePolarity=0;};}
if (!phasePolarity){phaseControl=phaseControl-phase_speed;if (phaseControl<-phase){phasePolarity=1;};}
digitalWrite(Z_STEP_BIT,!BIT_DIR_FOR_STEP);

  }
//direction change

 delay(jerkZ);
 accellerate=maxspeednoacellerationZ-SpeedwithaccelerationZ;
    digitalWrite( Z_DIRECTION_BIT,!Z_STEP_DIR);
   
delayMicroseconds(10);

 accelrate=accerate_rateZ;

  for (float i=0;i<max_size_z;i++){

if (accellerate !=0){   accellerate-=accerate_rateZ;}
//delayMicroseconds(SpeedwithaccelerationZ+accellerate);
digitalWrite(Z_STEP_BIT,BIT_DIR_FOR_STEP);
if (max_size_z-i <deacceleration_stepsZ){accellerate +=accerate_rateZ+accerate_rateZ;}//this causes slow down at end

delayMicroseconds(SpeedwithaccelerationZ+accellerate+phaseControl);
if (phasePolarity){phaseControl=phaseControl+phase_speed;if (phaseControl>phase){phasePolarity=0;};}
if (!phasePolarity){phaseControl=phaseControl-phase_speed;if (phaseControl<-phase){phasePolarity=1;};}
digitalWrite(Z_STEP_BIT,!BIT_DIR_FOR_STEP);

  }
   delay(jerkZ);
 }

//this is the planner of system
//we take each value and mutliply it by 1048576  so it can be stored as an int32 with decimal precision, we need to calculate and store errors below motion limit
//so we will define motion limit resolution
#define raw_stepsX1048576forAxisZ (Z_STEPS_PER_MM *1048576)
#define raw_stepsX1048576forAxisY (Y_STEPS_PER_DEG *1048576)
#define raw_stepsX1048576forAxisX (X_STEPS_PER_DEG *1048576)


 move_Axis(int axis, float distance_in_mm)
 {//we have an inteligent planner that when we are told what axis, it automatically loads info on that axis
//these are the values for axis  

//int32_t x_pos;//we use longs and divide by 65536 for positions so we have precision down beyond decimal
//int32_t y_pos;
//int32_t z_pos; 

int maxspeednoacelleration;
int Speedwithacceleration;
int accerate_rate;
int deacceleration_steps;
int jerkSettings;
long distance_per_mm_steps;
long planner_position_for_axis=distance_in_mm*1048576;//we convert from float to a value that is comparable for use
long current_postion_stored=0;
long travel_amount;
long distance_to_move;
//stepper control pins mapping
int step_pin;
int dir_pin;
int enable_pin;
bool dir_inverted_or_not_mode;
bool powerdown;
//first we load axis information and specs
if (axis ==Z_axis){current_postion_stored=z_pos;maxspeednoacelleration=maxspeednoacellerationZ;Speedwithacceleration=SpeedwithaccelerationZ;accerate_rate=accerate_rateZ;deacceleration_steps=deacceleration_stepsZ,jerkSettings=jerkZ;}
if (axis ==Z_axis){step_pin=Z_STEP_BIT;dir_pin=Z_DIRECTION_BIT; dir_inverted_or_not_mode=Z_STEP_DIR;distance_per_mm_steps=Z_STEPS_PER_MM;enable_pin=Z_ENABLE_BIT;powerdown=powerdownZ_when_not_moving;}
if (axis ==Y_axis){current_postion_stored=y_pos;maxspeednoacelleration=maxspeednoacellerationY;Speedwithacceleration=SpeedwithaccelerationY;accerate_rate=accerate_rateY;deacceleration_steps=deacceleration_stepsY,jerkSettings=jerkY; }
if (axis ==Y_axis){step_pin=Y_STEP_BIT;dir_pin=Y_DIRECTION_BIT; dir_inverted_or_not_mode=Y_STEP_DIR;distance_per_mm_steps=Y_STEPS_PER_DEG;enable_pin=Y_ENABLE_BIT;powerdown=powerdownY_when_not_moving;}
if (axis ==X_axis){current_postion_stored=x_pos;maxspeednoacelleration=maxspeednoacellerationX;Speedwithacceleration=SpeedwithaccelerationX;accerate_rate=accerate_rateX;deacceleration_steps=deacceleration_stepsX,jerkSettings=jerkX; }
if (axis ==X_axis){step_pin=X_STEP_BIT;dir_pin=X_DIRECTION_BIT; dir_inverted_or_not_mode=X_STEP_DIR;distance_per_mm_steps=X_STEPS_PER_DEG;enable_pin=X_ENABLE_BIT;powerdown=powerdownX_when_not_moving;}
//we need to calcuate acceleration depending on axis and timing as well as slowing down
int  accellerate=maxspeednoacelleration-Speedwithacceleration;

//now that we have axis peramiters we need to determine direction motor needs to go from planner_position_for_axis
if (current_postion_stored !=planner_position_for_axis){
//first we find out if any difference. if so then we continue
if (current_postion_stored>planner_position_for_axis){
//we find out if move is greater or lesser than current position
travel_amount=current_postion_stored-planner_position_for_axis;
//move stuff
current_postion_stored=current_postion_stored-travel_amount;
//we set motor direction
digitalWrite(dir_pin,!dir_inverted_or_not_mode);//we go back

}else{travel_amount=planner_position_for_axis-current_postion_stored;
current_postion_stored=current_postion_stored+travel_amount;
//we set motor direction
digitalWrite(dir_pin,dir_inverted_or_not_mode);//we go foward
}
//we now step the steps required to make distance.
//we use travel steps to determine amount of moves
//we need to enable power if power management

if  (powerdown){digitalWrite(enable_pin,BIT_ENABLE);} 
long mathcachetravel=1048576/distance_per_mm_steps;
long deacceleration=mathcachetravel*deacceleration_steps;
delayMicroseconds(10);// generous time for dir pin to be reconized
while (travel_amount>0){
//we use accleration delay
if (accellerate !=0){   accellerate-=accerate_rate;}//we decrease delay per step by rate determined in config.h per axis
if (travel_amount <deacceleration){accellerate +=accerate_rate+accerate_rate;}//this causes slow down at end
digitalWrite(step_pin,BIT_DIR_FOR_STEP);


if (phasePolarity){phaseControl=phaseControl+phase_speed;if (phaseControl>phase){phasePolarity=0;};}
if (!phasePolarity){phaseControl=phaseControl-phase_speed;if (phaseControl<-phase){phasePolarity=1;};}



delayMicroseconds(Speedwithacceleration+accellerate+phaseControl);
 digitalWrite(step_pin,!BIT_DIR_FOR_STEP);
 travel_amount=travel_amount-mathcachetravel;//this is limit of resolution. any remainder will be added back to planner
 

 
}

//movement complete to device precision
//we need to account for error in substep here
if (travel_amount>0){//if any amount of steps is beyound resolution for an actual step, we need to account for it in planner
// we use memory of pin direction to determine math
if (dir_inverted_or_not_mode){current_postion_stored=current_postion_stored+travel_amount;}
else{current_postion_stored=current_postion_stored-travel_amount;}
  
}

  
}//find out if distance to move
//we update current axis position
if (axis ==Z_axis){z_pos=current_postion_stored;delay(jerkZ);}
if (axis ==Y_axis){y_pos=current_postion_stored;delay(jerkY);}
if (axis ==X_axis){x_pos=current_postion_stored;delay(jerkX);}
Serial.print("currentpos ");Serial.print(float(current_postion_stored)/1048576);
Serial.print(" move_to_pos ");Serial.print(float(planner_position_for_axis)/1048576);
Serial.println("");
if  (powerdown){digitalWrite(enable_pin,!BIT_ENABLE);} 

 }

init_lasers_and_photo_sensor(){
  pinMode(leftSensor_disable_pin,INPUT);//we pull down SHDN pin to zero volts from resistor. even a low on digital output could be seen as a high.we make output and high when we use this sensor

   pinMode(rightSensor_disable_pin,INPUT);//we pull down SHDN pin to zero volts from resistor. even a low on digital output could be seen as a high.we make output and high when we use this sensor
 
 // pinMode(rightSensor_disable_pin,INPUT);//we pull down SHDN pin to zero volts from resistor. even a low on digital output could be seen as a high.we make output and high when we use this sensor
 //vl.begin();
//vl.changeAddress(byte address);
//Adafruit_VL6180X::change_READ_Address(byte address);
   delay(100);

  //we enable analog input on phot sensor
pinMode( photon_cell_analog_pin1,INPUT);
pinMode( photon_cell_analog_pin2,INPUT);
//we enable laser outputs
pinMode( RED_LASER_PIN,OUTPUT);
pinMode( GREEN_LASER_PIN,OUTPUT);
pinMode( BLUE_LASER_PIN,OUTPUT);

}
float readladarsensor1(){
//  float lux =sensor1.getAmbientLight(vl6180x_als_gain VL6180X_ALS_GAIN); //vl.readLux(VL6180X_ALS_GAIN_5);

  //Serial.print("Lux: "); Serial.print(lux);Serial.print(" ");
  //we sample 10 times. with noise it should average outb
  
  uint16_t range2 = sensor1.getDistance();// range2 += sensor1.readRange();range2 += sensor1.readRange();
  //range2 += vl.readRange();range2 += vl.readRange();range2 += vl.readRange();range2 += vl.readRange();range2 += vl.readRange();range2 += vl.readRange();range2 += vl.readRange();

 // range2 += vl.readRange();range2 += vl.readRange();
 // range2 += vl.readRange();range2 += vl.readRange();range2 += vl.readRange();range2 += vl.readRange();range2 += vl.readRange();range2 += vl.readRange();range2 += vl.readRange();
float  range =range2*1;//we give it 0.1mm res
  //float range =vl.readRangeFullResolution();
  //uint8_t status = vl.readRangeStatus();

return range;
}

float readladarsensor2(){
//  float lux =sensor1.getAmbientLight(vl6180x_als_gain VL6180X_ALS_GAIN); //vl.readLux(VL6180X_ALS_GAIN_5);

  //Serial.print("Lux: "); Serial.print(lux);Serial.print(" ");
  //we sample 10 times. with noise it should average outb
  
  uint16_t range2 = sensor2.getDistance();// range2 += sensor1.readRange();range2 += sensor1.readRange();
  //range2 += vl.readRange();range2 += vl.readRange();range2 += vl.readRange();range2 += vl.readRange();range2 += vl.readRange();range2 += vl.readRange();range2 += vl.readRange();

 // range2 += vl.readRange();range2 += vl.readRange();
 // range2 += vl.readRange();range2 += vl.readRange();range2 += vl.readRange();range2 += vl.readRange();range2 += vl.readRange();range2 += vl.readRange();range2 += vl.readRange();
float  range =range2*1;//we give it 0.1mm res
  //float range =vl.readRangeFullResolution();
  //uint8_t status = vl.readRangeStatus();

return range;
}

int read_photo_cell1(){
return analogRead(photon_cell_analog_pin1);
}


int read_photo_cell2(){
return analogRead(photon_cell_analog_pin2);
}
bool check_serial(){

    while (Serial.available()>0){//this means data available while keeps going if only does 1 read
   
   char tempstring=Serial.read();//we read data

    //if a value we test but we turn of lasers before motion, then turn one laser back on

 
    if (tempstring==char('r')){digitalWrite(GREEN_LASER_PIN,LOW);digitalWrite(BLUE_LASER_PIN,LOW); if (Y_STEP_BIT>0){ move_Axis(Y_axis, RED_LASER_POS);};digitalWrite(RED_LASER_PIN,HIGH);}
    if (tempstring==char('g')){digitalWrite(BLUE_LASER_PIN,LOW);digitalWrite(RED_LASER_PIN,LOW);if (Y_STEP_BIT>0){move_Axis(Y_axis,GREEN_LASER_POS);};digitalWrite(GREEN_LASER_PIN,HIGH);}
    if (tempstring==char('b')){digitalWrite(RED_LASER_PIN,LOW);digitalWrite(GREEN_LASER_PIN,LOW);if (Y_STEP_BIT>0){move_Axis(Y_axis, BLUE_LASER_POS);};digitalWrite(BLUE_LASER_PIN,HIGH);}

    if (tempstring==char('R')){Serial.print("Zhome="); Serial.println(digitalRead(ENDSTOPZ_PIN));Serial.print("Yhome=");
    Serial.println(digitalRead(ENDSTOPY_PIN));Serial.print("Xhome="); Serial.println(digitalRead(ENDSTOPX_PIN));delay(5);}
    if (tempstring==char('o')){digitalWrite(RED_LASER_PIN,LOW);digitalWrite(GREEN_LASER_PIN,LOW);digitalWrite(BLUE_LASER_PIN,LOW);}
    if (tempstring==char('p')){digitalWrite(X_ENABLE_BIT,!BIT_ENABLE);digitalWrite(Y_ENABLE_BIT,!BIT_ENABLE);digitalWrite(Z_ENABLE_BIT,!BIT_ENABLE);}
    if (tempstring==char('P')){digitalWrite(X_ENABLE_BIT,BIT_ENABLE);digitalWrite(Y_ENABLE_BIT,BIT_ENABLE);digitalWrite(Z_ENABLE_BIT,BIT_ENABLE);}
     
    //return 1;//we have a read
    if (tempstring==char('o')){digitalWrite(RED_LASER_PIN,LOW);digitalWrite(GREEN_LASER_PIN,LOW);digitalWrite(BLUE_LASER_PIN,LOW);}

    if (tempstring==char('Z')){move_Axis(Z_axis, ZMAX_HIEGH_MM);}

    if (tempstring==char('z')){move_Axis(Z_axis, Z_START_ABOVE_HOME_MM);}
    if (tempstring==char('X')){move_Axis(X_axis, 0.0);}
    if (tempstring==char('x')){move_Axis(X_axis, 360.0);}
    if (tempstring==char('c')){Serial.print("light_level="); Serial.println(read_photo_cell1());delay(5);}
        if (tempstring==char('C')){Serial.print("light_level="); Serial.println(read_photo_cell2());delay(5);}
    if (tempstring==char('l')){Serial.print("distance="); Serial.println(readladarsensor1());delay(5);}
    if (tempstring==char('L')){Serial.print("distance="); Serial.println(readladarsensor2());delay(5);}

        if (tempstring==char('s')){digitalWrite(RED_LASER_PIN,LOW);digitalWrite(GREEN_LASER_PIN,LOW);digitalWrite(BLUE_LASER_PIN,LOW);move_Axis(Y_axis, ladar_pos);}//we set ladar position to center
if (tempstring==char('h')){
 if (X_STEP_BIT>0){ run_homing_check_X();}//we want a  position for x. since no homeing we set at zero
// if (Y_STEP_BIT>0){  run_homing_check_Y();}//we want y to be homed
if (Z_STEP_BIT>0){   run_homing_check_Z();}//we want z to be homed
  }
    //Serial.println(tempstring);//we wont get rest of commands done unless this is removed
  }
}
