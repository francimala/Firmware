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

#include "rc_input_simulation.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/input_rc.h>


int RcInputSimulation::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int RcInputSimulation::custom_command(int argc, char *argv[])
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


int RcInputSimulation::task_spawn(int argc, char *argv[])
{
        _task_id = px4_task_spawn_cmd("rc_input_simulation",
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

RcInputSimulation *RcInputSimulation::instantiate(int argc, char *argv[])
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

        RcInputSimulation *instance = new RcInputSimulation(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

        return instance;

}

RcInputSimulation::RcInputSimulation(int example_param, bool example_flag)
        : ModuleParams(nullptr)
{
	input = (double)example_param/50;
	//input = 0;
	PX4_INFO("Input: %f", input);
}

void RcInputSimulation::run()
{
				int count = 0;
				int count2 = 0;
				// Operation needed to publish information on actuator_controls_3 topic
        struct input_rc_s _in;
        memset(&_in, 0, sizeof(_in));
        orb_advert_t _in_pub = orb_advertise(ORB_ID(input_rc), &_in);

				// initialize parameters
				parameters_update(true);

				PX4_INFO("Entering the endless loop");

				while (!should_exit()) {

					px4_usleep(100);

					if (count2 > 1000) {
						_in.channel_count = 18;
						_in.values[7] = count*100+1000;
						_in.values[6] = count*100+1000;
						orb_publish(ORB_ID(input_rc), _in_pub, &_in);
						count2 = 0;
						count++;
					}
					count2++;
					if(count>10) {
						//PX4_INFO("count reached 2, resetting");
						count = 0;
					}
		}

		parameters_update();
}


void RcInputSimulation::parameters_update(bool force)
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

int RcInputSimulation::print_usage(const char *reason)
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

        PRINT_MODULE_USAGE_NAME("rc_input_simulation", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int rc_input_simulation_main(int argc, char *argv[])
{
        return RcInputSimulation::main(argc, argv);
}
