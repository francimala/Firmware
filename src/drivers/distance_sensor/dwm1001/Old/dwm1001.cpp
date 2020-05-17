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
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	CDev(DWM1001_DEFAULT_PORT)
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

int
DWM1001::init()
{
	// status
	int ret = 0;
	PX4_INFO("DWM1001 initialization started");

	do { // create a scope to handle exit conditions using break

		// open fd
		_fd = ::open(_port, O_RDWR | O_NOCTTY);

		if (_fd < 0) {
			PX4_ERR("Error opening fd");
			return -1;
		}

		PX4_INFO("This is fd after opening the connection: %d",_fd);

		// baudrate 115200, 8 bits, no parity, 1 stop bit
		unsigned speed = B115200;
		termios uart_config{};
		int termios_state{};

		tcgetattr(_fd, &uart_config);

		// clear ONLCR flag (which appends a CR for every LF)
		uart_config.c_oflag &= ~ONLCR;

		// set baud rate
		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d ISPD", termios_state);
			ret = -1;
			break;
		}

		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d OSPD\n", termios_state);
			ret = -1;
			break;
		}

		if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
			PX4_ERR("baud %d ATTR", termios_state);
			ret = -1;
			break;
		}

		uart_config.c_cflag |= (CLOCAL | CREAD);	// ignore modem controls
		uart_config.c_cflag &= ~CSIZE;
		uart_config.c_cflag |= CS8;			// 8-bit characters
		uart_config.c_cflag &= ~PARENB;			// no parity bit
		uart_config.c_cflag &= ~CSTOPB;			// only need 1 stop bit
		uart_config.c_cflag &= ~CRTSCTS;		// no hardware flowcontrol

		/*
		// setup for non-canonical mode
		uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
		uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		uart_config.c_oflag &= ~OPOST;

		// fetch bytes as they become available
		uart_config.c_cc[VMIN] = 1;
		uart_config.c_cc[VTIME] = 1;
		*/

		if (_fd < 0) {
			PX4_ERR("FAIL: laser fd");
			ret = -1;
			break;
		}

	} while (0);

	// close the fd
	::close(_fd);
	_fd = -1;

	PX4_INFO("I have closed the file, this is fd %d",_fd);

	if (ret == PX4_OK) {
		PX4_INFO("Start is launched");
		start();
	}

	PX4_INFO("DWM1001 initialization compelted, the communication should be fine");
	PX4_INFO("This is ret %d",ret);
	px4_usleep(10000);

	return ret;
}

int
DWM1001::collect()
{
	perf_begin(_sample_perf);
	PX4_INFO("DWM1001 collection started");

	// clear buffer if last read was too long ago
	int64_t read_elapsed = hrt_elapsed_time(&_last_read);

	// the buffer for read chars is buflen minus null termination
	int readlen = 36; // how many characters do I want to read? This value must be >= 36
	char readbuf[readlen-1];

	int ret = 0;
//	float distance_m = -1.0f;

//////////////////////////////////////////////////////////////////////////

	// Check the number of bytes available in the buffer
	int bytes_available = 0;
	PX4_INFO("This is fd before using ioctl: %d",_fd);
	::ioctl(_fd, FIONREAD, (unsigned long)&bytes_available);
	px4_sleep(1);
	PX4_INFO("ioctl performed in collect, bytes_available: %d", bytes_available);


	if (!bytes_available) {
		PX4_INFO("No bytes available");
		perf_end(_sample_perf);
		return -EAGAIN;
	}

////////////////////////////////////////////////////////////////////////////


	// parse entire buffer
//	const hrt_abstime timestamp_sample = hrt_absolute_time(); // UNLOCK THISSSS

	do {
		// read from the sensor (uart buffer)
		ret = ::read(_fd, &readbuf[0], readlen); // this is the KEY
		PX4_INFO("I'm in the do component");

		if (ret < 0) {
			PX4_ERR("read err: %d", ret);
			perf_count(_comms_errors);
			perf_end(_sample_perf);

			// only throw an error if we time out
			if (read_elapsed > (kCONVERSIONINTERVAL * 2)) {
				/* flush anything in RX buffer */
				tcflush(_fd, TCIFLUSH);
				return ret;

			} else {
				return -EAGAIN;
			}
		}

		_last_read = hrt_absolute_time();

		PX4_INFO("%c",readbuf[0]); // CHECK THIS
///////////////////////////////////////////////////////////////////////////////////////
/*
		// parse buffer
		for (int i = 0; i < ret; i++) {
			dwm1001_parse(readbuf[i], _linebuf, &_linebuf_index, &_parse_state, &distance_m);
		}

		// bytes left to parse
		bytes_available -= ret;

	} while (bytes_available > 0);

	// no valid measurement after parsing buffer
	if (distance_m < 0.0f) {
		perf_end(_sample_perf);
		return -EAGAIN;
	}

	// publish most recent valid measurement from buffer
	_px4_rangefinder.update(timestamp_sample, distance_m);
	*/
///////////////////////////////////////////////////////////////////////////////////////////
} while(0);

	perf_end(_sample_perf);

	return PX4_OK;
}

void
DWM1001::start()
{
	// schedule a cycle to start things
	PX4_INFO("DWM1001 start");
	ScheduleOnInterval(1000_ms);
}

void
DWM1001::stop()
{
	PX4_INFO("DWM1001 stop");
	ScheduleClear();
}

void
DWM1001::Run()
{
	PX4_INFO("DWM1001 run");
	// fds initialized?
	if (_fd < 0) {
		// open fd
		_fd = ::open(_port, O_RDWR | O_NOCTTY);
	}

	// perform collection
	if (collect() == -EAGAIN) {
		// reschedule to grab the missing bits, time to transmit 9 bytes @ 115200 bps
		ScheduleClear();
		ScheduleOnInterval(1000_ms, 87 * 9);
		return;
	}
}

void
DWM1001::print_info()
{
	printf("Using port '%s'\n", _port);
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);

	PX4_INFO("Still WIP");
}
