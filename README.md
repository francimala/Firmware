# PX4 autopilot customization for non standard gimbal and UWB peripherals

### A quick introduction
Hi reader, this PX4 version is related to the master thesis work "PX4 autopilot customization for non standard gimbal and UWB peripherals" developed by Francesco Malacarne and supervised by Simone Silvestro, Marcello Chiaberge and Gianluca Dara. The goal was to implement a single axis gimbal on a ultralight drone (equipped with Pixracer) and to develop a custom driver for UWB modules (DWM1001-dev) to be used for future indoor navigation applications currently under study at PIC4SeR.

## Where can you find the customization?
- Stabilization module is named servo_control and can be found in directory: Firmware/src/modules/servo_control.
- DWM1001 driver is named dwm1001 and can be found in directory: Firmware/src/drivers/distance_sensor/dwm1001
- rc_update has been slightly modified, it publishes on a different topic (actuator_control_rc instead of actuator_control). It can be found in directory: Firmware/src/modules/rc_update
- mission_block.cpp was slightly modified to support 1-axis gimbal. It can be found in directory: Firmware/src/modules/navigator

## Useful links
PX4 Developer guide: https://dev.px4.io/master/en/
PX4 User guide: https://docs.px4.io/master/en/
PX4 Doxygen: https://px4.github.io/Firmware-Doxygen/dc/d61/md_src_drivers_uavcan_uavcan_drivers_stm32__r_e_a_d_m_e.html
Connection between Arduino and DWM1001-dev module: https://github.com/francimala/Arduino-MEGA-to-DWM1001
Some YouTube videos I made to explain some concepts: https://www.youtube.com/playlist?list=PLXL_6wK-bGzdde2lsofkw1bZ2mec8i9ra
