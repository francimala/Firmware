/****************************************************************************
 *
 *   Copyright (c) 2017-2019 PX4 Development Team. All rights reserved.
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

#include "dwm1001.hpp"

DWM1001::DWM1001(const char *port) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port))
{
	PX4_INFO("DWM1001 function, the first one present, before strncpy");

	// store port name
	strncpy(_port, port, sizeof(_port) - 1);

	// enforce null termination
	_port[sizeof(_port) - 1] = '\0';
}

DWM1001::~DWM1001()
{
	// make sure we are truly inactive
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

bool DWM1001::init()
{
	// status
	int ret = 0;
	PX4_INFO("DWM1001 initialization started");

	do { // create a scope to handle exit conditions using break

		// open fd
		_fd = ::open(_port, O_RDWR | O_NOCTTY | O_SYNC);

		if (_fd < 0) {
			PX4_ERR("Error opening fd");
			return false;
		}

		PX4_INFO("This is fd after opening the connection: %d",_fd);

		// baudrate 115200, 8 bits, no parity, 1 stop bit
		unsigned speed = B115200;
		termios uart_config{};
		int termios_state{};

		if(tcgetattr(_fd, &uart_config) < 0) {
	    PX4_ERR("Error from tcgetattr");
	    return false;
	  };

		// clear ONLCR flag (which appends a CR for every LF)
		uart_config.c_oflag &= ~ONLCR;

		// set baud rate
		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d ISPD", termios_state);
			ret = -1;
			return false;
		}

		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d OSPD\n", termios_state);
			ret = -1;
			return false;
		}

		uart_config.c_cflag |= (CLOCAL | CREAD);	// ignore modem controls
		uart_config.c_cflag &= ~CSIZE;
		uart_config.c_cflag |= CS8;			// 8-bit characters
		uart_config.c_cflag &= ~PARENB;			// no parity bit
		uart_config.c_cflag &= ~CSTOPB;			// only need 1 stop bit
		uart_config.c_cflag &= ~CRTSCTS;		// no hardware flowcontrol

		// setup for non-canonical mode
		uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
		uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		uart_config.c_oflag &= ~OPOST;

		// fetch bytes as they become available
		uart_config.c_cc[VMIN] = 1;
		uart_config.c_cc[VTIME] = 1;

		if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
			PX4_ERR("baud %d ATTR", termios_state);
			ret = -1;
			return false;
		}

		if (_fd < 0) {
			PX4_ERR("FAIL: fd is negative at the end of the initialization");
			ret = -1;
			return false;
		}

		tcflush(_fd, TCIOFLUSH); // flushes both data received but not read, and data written but not transmitted.

	} while (0);

	// close the fd
	::close(_fd);
	_fd = -1;

	PX4_INFO("I have closed the file, this is fd %d",_fd);

	if (ret == PX4_OK) {
		PX4_INFO("Start is now launched");
		start();
	}

	PX4_INFO("DWM1001 initialization compelted, the communication should be fine");
	PX4_INFO("This is ret %d",ret);

	return true;
}

int DWM1001::collect()
{
	perf_begin(_sample_perf);
	PX4_INFO("DWM1001 collection started");

	// clear buffer if last read was too long ago
	//int64_t read_elapsed = hrt_elapsed_time(&_last_read);

	// the buffer for read chars is buflen minus null termination
	int readlen = 10; // how many characters do I want to read? This value must be >= 36
	char readbuf[readlen-1];

	// read from the sensor (uart buffer)
	::read(_fd, &readbuf[0], 1); // this is the KEY
	printf("%c", readbuf[0]);

	perf_end(_sample_perf);

	return PX4_OK;
}

void DWM1001::start()
{
	// schedule a cycle to start things
	PX4_INFO("DWM1001 start function");
	ScheduleOnInterval(1000_us);
}

void DWM1001::stop()
{
	PX4_INFO("DWM1001 stop");
	ScheduleClear();
}

void DWM1001::Run()
{
	PX4_INFO("DWM1001 run");

	if (should_exit()) {
		stop();
	}

	// fds initialized?
	if (_fd < 0) {
		// open fd
		_fd = ::open(_port, O_RDWR | O_NOCTTY | O_SYNC);
	}

	// perform collection
	if (collect() == -EAGAIN) {
		// reschedule to grab the missing bits, time to transmit 9 bytes @ 115200 bps
		ScheduleClear();
		ScheduleOnInterval(1000_us, 87 * 9);
		return;
	}
}

int DWM1001::task_spawn(int argc, char *argv[])
{
	PX4_INFO("task_spawn is launched");
	DWM1001 *instance = new DWM1001(DWM1001_DEFAULT_PORT);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int DWM1001::print_status()
{
	PX4_INFO("print status function");
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	return 0;
}

int DWM1001::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int DWM1001::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

		PRINT_MODULE_DESCRIPTION(
			R"DESCR_STR(
	### Description
	Example of a simple module running out of a work queue.
	)DESCR_STR");

		PRINT_MODULE_USAGE_NAME("dwm1001", "template");
		PRINT_MODULE_USAGE_COMMAND("start");
		PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

		return 0;
}

extern "C" __EXPORT int dwm1001_main(int argc, char *argv[])
{
	PX4_INFO("The main is started");
	return DWM1001::main(argc, argv);
}
