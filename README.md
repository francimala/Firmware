# PX4 autopilot customization for non standard gimbal and UWB peripherals

## A quick introduction
Hi reader, this PX4 version is related to the master thesis work "PX4 autopilot customization for non standard gimbal and UWB peripherals" developed by Francesco Malacarne and supervised by Simone Silvestro, Marcello Chiaberge and Gianluca Dara. The goal was to implement a single axis gimbal on a ultralight drone (equipped with Pixracer) and to develop a custom driver for UWB modules (DWM1001-dev) to be used for future indoor navigation applications currently under study at PIC4SeR. While the first part was intended to work only for Pixracer-based systems, the second one should work with any configuration without significant changes (Pixracer and Pixhawk in particular).

### Stabilization
The stabilization module is named servo_control. Its working principle can be broken down in the following steps: it subscribes to the attitude topic, it extracts the drone pitch and it creates a counteraction to be published on the pin where the servomotor is connected. In parallel, it takes the radio controller input and it adds it to the stabilization contribution. In the end, the sum of the two components is published on actuator_control_3 topic, so that the servomotor connected to PIN 5 can be driven accordingly.

### UWB driver
The serial driver is named dwm1001. Its working principle can be broken down in the following steps: it opens the serial port (TELEM2 by default), it programs the UWB module with command "lec" and it starts reading the message over the opened serial port. Then, it sorts the information out and it publishes them on a new topic named dwm1001 according to the following structure:
- Distances between the i-th anchor and the tag to be located
- Estimated position (relevant value for indoor navigation)
- Number of detected anchors
- Estimated position indicator for debugging purposes

## Where can you find the added files?
- Stabilization module can be found in directory: Firmware/src/modules/servo_control.
- DWM1001 driver can be found in directory: Firmware/src/drivers/distance_sensor/dwm1001
- rc_update has been slightly modified, it publishes on a different topic (actuator_control_rc instead of actuator_control). It can be found in directory: Firmware/src/modules/rc_update
- mission_block.cpp was slightly modified to support 1-axis gimbal. It can be found in directory: Firmware/src/modules/navigator

It is highly suggested to fork this firmware and to build it as is because I had to make small changes to some CMake files to let the firmware integrate the new modules that I created.

## Some useful links
- PX4 Developer guide: https://dev.px4.io/master/en/
- PX4 User guide: https://docs.px4.io/master/en/
- PX4 Doxygen: https://px4.github.io/Firmware-Doxygen/dc/d61/md_src_drivers_uavcan_uavcan_drivers_stm32__r_e_a_d_m_e.html
- Connection between Arduino and DWM1001-dev module: https://github.com/francimala/Arduino-MEGA-to-DWM1001
- Some YouTube videos I made to explain some concepts: https://www.youtube.com/playlist?list=PLXL_6wK-bGzdde2lsofkw1bZ2mec8i9ra
