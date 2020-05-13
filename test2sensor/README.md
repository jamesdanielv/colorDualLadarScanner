this code has a planner as well as a gcode parser but it is simple at this point
terminal send h will home all axis, or set them to zero if no home switch

steppers_init() should be called during setup. it sets pin states from config
these are needed for each  axis
run_homing_check_z() 
run_homing_check_Y()
run_homing_check_x()


readladarsensor1() returns sensor 1 data
readladarsensor2() returns sensor 2 data
read_photo_cell1() returns photo cell 1 data
read_photo_cell2() returns photo cell 2 data

parse commands understood by gcode 
r enable red laser, turn others off
g enable green laser, turn others off
b enable blue laser, turn others off
R show end stop status
o turn all lasers off
p power down all motors
P power up all motors
z move z axis to minumum 
Z move z axis to maximum
x move x axis to minumum 
X move x axis to maximum
c show value analog from cell 1
C show value analog from cell 2
l show range from ladar 1 in mm 
L show range from ladar 2 in mm
h home all axis, or set axis to 0.0000
s is set ladar positon to sensor but this is only if ladar is on a y rotation axis

after axis is init and is homed 
all that is needed to move axis is Z_axis, Y_axis. x_axis and the distance.
for example move_Axis(Z_axis, 10.5);//we move z axis up full distance

the remainder beyond resolution is stored in planner this prevents accumulation of errors.


