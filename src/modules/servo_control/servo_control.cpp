/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#define _USE_MATH_DEFINES

#include "servo_control.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/input_rc.h>


int ServoControl::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int ServoControl::custom_command(int argc, char *argv[])
{
        /*
        if (!is_running()) {
                print_usage("not running");
                return 1;
        }

        // additional custom commands can be handled like this:
        if (!strcmp(argv[0], "do-something")) {
                get_instance()->do_something();
                return 0;
        }
         */

        return print_usage("unknown command");
}


int ServoControl::task_spawn(int argc, char *argv[])
{
        _task_id = px4_task_spawn_cmd("servo_control",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

ServoControl *ServoControl::instantiate(int argc, char *argv[])
{

        int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
                        PX4_INFO("Something happened, %d", example_param);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

        ServoControl *instance = new ServoControl(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

        return instance;

}

ServoControl::ServoControl(int example_param, bool example_flag)
        : ModuleParams(nullptr)
{
	keyboard_input = (double)example_param/50;
	//keyboard_input = 0;
	PX4_INFO("Input: %f", keyboard_input);
}

void ServoControl::run()
{
        // Creating an object of the class QuaternionEuler and a structure with type EulerAngles
        QuaternionEuler etq;

        int count = 0;
				int count2 = 0;

        // Running the loop synchronized to the vehicle_attitude topic publication
        int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
				int input_rc_sub = orb_subscribe(ORB_ID(input_rc));

        // Operation needed to publish information on actuator_controls_3 topic
        struct actuator_controls_s out;
        memset(&out, 0, sizeof(out));
        orb_advert_t out_pub = orb_advertise(ORB_ID(actuator_controls_3), &out);

        // Here we initialize the polling routine; if we need more subscriptions
        // it is enough to add components to the array fds[] and initialize its components
        // similarly to the vehicle_attitude one
        px4_pollfd_struct_t fds[2];
        fds[0].fd = vehicle_attitude_sub;
				fds[0].events = POLLIN;
				fds[1].fd = input_rc_sub;
				fds[1].events = POLLIN;

				// initialize parameters
				parameters_update(true);

				while (!should_exit()) {

								// wait for up to 1000ms for data
								int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);
								int pret2 = px4_poll(fds, (sizeof(fds) / sizeof(fds[1])), 1000);

								if (pret == 0) {
									// Timeout: let the loop run anyway, don't do `continue` here

								} else if (pret < 0) {
									// this is undesirable but not much we can do
									PX4_ERR("poll error %d, %d", pret, errno);
									px4_usleep(50000);
									continue;

								} else if ((fds[0].revents & POLLIN)) {

										QuaternionEuler::Quaternion q1;
                    QuaternionEuler::EulerAngles ea1;

                        struct vehicle_attitude_s vehicle_attitude;
                        orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &vehicle_attitude);

                        // Converting quaternion to roll, pitch, yaw
                        q1.w = vehicle_attitude.q[0];
                        q1.x = vehicle_attitude.q[1];
                        q1.y = vehicle_attitude.q[2];
                        q1.z = vehicle_attitude.q[3];

                        ea1 = etq.QuaternionToEuler(q1); // radiants

                        // Now we wait for N cycles before making the comparison
                        if (count >= 10) {

													  count2++;

                            out.control[5] = -ea1.pitch*180/3.1416/50 + keyboard_input + radio_input;
														//out.control[6] = -ea1.pitch*180/3.1416/50 + keyboard_input; // added
                            orb_publish(ORB_ID(actuator_controls_3), out_pub, &out);

														if(count2 > 50) {
															//PX4_INFO("Radio input: %f", radio_input);
															//PX4_INFO("Pitch contribution: %f", -ea1.pitch*180/3.1416/50);
															count2 = 0;
														}

														if (count >= 500) {
															PX4_INFO("We are in the cycle, %f", ServoControl::keyboard_input);
	                            PX4_INFO("Attitude --> Roll: %f, Pitch: %f, Yaw: %f", ea1.roll*180/3.1416, ea1.pitch*180/3.1416, ea1.yaw*180/3.1416);
	                            PX4_INFO("Control --> Roll: %f, Pitch: %f, Yaw: %f",ea1.roll*180/3.1416/60, ea1.pitch*180/3.1416/180, ea1.yaw*180/3.1416);
														}

														count = 0;
                        }

												count = count +1;
												//PX4_INFO("%d",count);
		}
		if (pret2 == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here

		} else if (pret2 < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(50000);
			continue;

		} else if ((fds[1].revents & POLLIN)) {

			struct input_rc_s input_rc;
			orb_copy(ORB_ID(input_rc), input_rc_sub, &input_rc);

			radio_input = (double)(input_rc.values[6]-1500)/500;
			//printf("Radio input: %f\n",radio_input);
			//printf("Topic value: %d\n",input_rc.values[6]);

			if(radio_input < -1 || radio_input > 1) {
				radio_input = 0;
			}
		}

		parameters_update();
	}

        orb_unsubscribe(vehicle_attitude_sub);
				orb_unsubscribe(input_rc_sub);
}

void ServoControl::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

int ServoControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

        PRINT_MODULE_USAGE_NAME("servo_control", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

QuaternionEuler::EulerAngles QuaternionEuler::QuaternionToEuler(Quaternion q) {
    QuaternionEuler::EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1) {
        //angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        if (sinp > 0) {
					angles.pitch = M_PI / 2;
				}
				else if (sinp < 0) {
					angles.pitch = -M_PI / 2;
				}
		}
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

int servo_control_main(int argc, char *argv[])
{
        return ServoControl::main(argc, argv);
}
