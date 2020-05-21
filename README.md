this code is set to use 115200 baud rate. set serial interface you se to 115200, or change the baud rate used in code.


2 software i2c buses are created for ladar sensors. this is because both have same address, and both are active at same time to prevent need to re init on each device use.

to disable a stepper homing in config.h set endstop for that motor to -1. then stepper should just set current pos to 0



when setting motor distance in mm, TPI is turns per inch of threaded rod. if threaded rod is not used then just calc steps per mm from gear reduction and steps per turn. for example a 1.8 deg stepper usually has 200 turns per rotation. 

some things that might help calc steps per mm
https://forum.digikey.com/t/steps-per-revolution-and-step-angle-in-stepper-motors/1146
(this area being currently edited. more info soon!)

this code currently requires serial terminal from arduino ide, or any serial terminal such as in python, node.js, or c++ to be active. once active code will start on arduino. 
if you want it to run code without serial being enabled 
remark out these lines from .ino main tab, which currently is named test2sensor.ino
 while (!Serial) {
    delay(1);
  }



this is a 4 part system

1 firmware with motor control, laser control, light sensor control and dual i2c control of time of flight sensors.
2 hardware with designs, and motor specs as well as laser parts and ladar sensor parts lists and schematics
3 software interface most likely python or node.js for multi operating system support and acces to rs232 comm

4 any project that uses this code, must refference it and not use it for evil robots. more explain of this in future.



