## servo_control description
### Components
quaternion_euler.h is the header for the function implementing the conversion from quaternion to euler within servo_control.

servo_control.h is the header for the servo_control module.

servo_control.cpp is the actual module implementing stabilization functionalities.

CMakeLists.txt must be compliant with the above mentioned components. If you have doubts see the official developer guide in section "Writing your first application".

### What is its goal
This module is used to stabilize and control the servomotor position so that the camera pitch angle can be both stabilized and set using the RC. 

### How does it work
The module can be divided in 2 different components:
1. The first one is used to stabilize the camera pitch, publishing on the actuator_control_3 topic a value that is a counteraction to the current drone pitch. The module subscribes to vehicle_attitude topic, converts the quaternion representing the current drone orientation and creates the counteraction.
2. The second one is used to implement manual passthrough function. RC commands are published by rc_update module on a new topic named actuator_control_rc so that no conflict is created between rc_update and servo_control. Servo_control subscribes to actuator_control_rc and adds the RC contribution to the stabilization one.

The final result is a stabilized camera with the possibility of setting the stabilization point through RC.
