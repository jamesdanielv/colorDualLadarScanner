//these are variables used for code globally
//#ifndef var_scanner
//#define var_scanner

bool phasePolarity=1;//1 is go up

//planner values
long x_pos=0;//we use longs and divide by 1048576 for positions so we have precision down beyond decimal
long y_pos=0;//this guarentees we wont have rounding errors in 
long z_pos=0;//motion. we so the values in x,y,z can be +/- 2^31/65536
int phaseControl=0;
float pos_sensor ;//ladar position to be set

//int accellerate=maxspeednoacellerationZ-SpeedwithaccelerationZ;
//#endif
