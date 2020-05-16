/****************************************************************************
 *
 *   Copyright (c) 2017-2018 PX4 Development Team. All rights reserved.
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

/**
 * @file dwm1001dev.cpp
 * @author Francesco Malacarne <s260199@studenti.polito.it>
 *
 * Driver for the DWM1001-dev UWB sensor
 */

#include <float.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/workqueue.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <termios.h>
#ifdef __PX4_CYGWIN
#include <asm/socket.h>
#endif


#include <perf/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>
#include <drivers/device/device.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/airspeed.h>

#include <board_config.h>

#include <systemlib/mavlink_log.h>
#include <uORB/topics/vehicle_status.h>

#define DWM1001_DEFAULT_PORT	"/dev/ttyS1" // see https://github.com/PX4/px4_user_guide/issues/417

/* Configuration Constants */

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class DWM1001dev : public cdev::CDev
{
public:
	DWM1001dev(const char *port);
	virtual ~DWM1001dev();
	virtual int init();
	virtual ssize_t read(device::file_t *filp, char *buffer, size_t buflen);
	virtual int ioctl(device::file_t *filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void print_info();

private:
	char                     _port[20];
	int                      _conversion_interval;
	work_s                   _work{};
	ringbuffer::RingBuffer  *_reports;
	int                      _measure_ticks;
	bool                     _collect_phase;
	int                      _fd;
	char                     _linebuf[30];
	unsigned                 _linebuf_index;
	orb_advert_t						 _airspeed_pub;
	orb_advert_t             mavlink_log_pub = nullptr;
	vehicle_status_s				 status = {};
	hrt_abstime              _last_read;
	hrt_abstime 						 boot_time;
	airspeed_s  						 airspeed;

	int                      _class_instance;
	int                      _orb_class_instance;
	int 										 k = 0;
	perf_counter_t           _sample_perf;
	perf_counter_t           _comms_errors;

	/**
	* Initialise the automatic measurement state machine and start it.
	*/
	void				start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void				stop();

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void				cycle();
	int				collect();

	/**
	* Static trampoline from the workq context; because we don't have a
	* generic workq wrapper yet.
	*
	* @param arg		Instance pointer for the driver that is polling.
	*/
	static void			cycle_trampoline(void *arg);


};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int dwm1001dev_main(int argc, char *argv[]);

DWM1001dev::DWM1001dev(const char *port) :
	CDev(DWM1001_DEFAULT_PORT),
	_conversion_interval(9000),
	_reports(nullptr),
	_measure_ticks(0),
	_collect_phase(false),
	_fd(-1),
	_linebuf_index(0),
	_last_read(0),
	_class_instance(-1),
	_orb_class_instance(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "dwm1001dev_read")),
	_comms_errors(perf_alloc(PC_COUNT, "dwm1001dev_com_err"))
{
	/* store port name */
	strncpy(_port, port, sizeof(_port) - 1);
	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';
}

DWM1001dev::~DWM1001dev()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
DWM1001dev::init()
{
	PX4_INFO("DWM1001 initialization started");
	/* status */
	int ret = 0;
	PX4_INFO("This is ret %d",ret);

	do { /* create a scope to handle exit conditions using break */

		/* open fd */
		_fd = ::open(_port, O_RDONLY | O_NOCTTY);

		if (_fd < 0) {
			PX4_ERR("Error opening fd");
			return -1;
		}
		PX4_INFO("The port was opened, now we set everything up");
		PX4_INFO("This is ret %d",ret);

		/*baudrate 115200, 8 bits, no parity, 1 stop bit */
		unsigned speed = B115200;

		struct termios uart_config;

		int termios_state;

		tcgetattr(_fd, &uart_config);

		/* clear ONLCR flag (which appends a CR for every LF) */
		uart_config.c_oflag &= ~ONLCR;

		/* set baud rate */
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

		uart_config.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
		uart_config.c_cflag &= ~CSIZE;
		uart_config.c_cflag |= CS8;         /* 8-bit characters */
		uart_config.c_cflag &= ~PARENB;     /* no parity bit */
		uart_config.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
		uart_config.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

		/* setup for non-canonical mode */
		uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
		uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		uart_config.c_oflag &= ~OPOST;

		/* fetch bytes as they become available */
		uart_config.c_cc[VMIN] = 1;
		uart_config.c_cc[VTIME] = 1;

		if (_fd < 0) {
			PX4_ERR("FAIL: laser fd");
			ret = -1;
			break;
		}
		PX4_INFO("This is ret %d",ret);
		PX4_INFO("Before CDev init");
		/* do regular cdev init */
		ret = CDev::init();
		PX4_INFO("This is ret %d",ret);
		PX4_INFO("After CDev init");

		ret = 0;

		if (ret != OK) { break; }

		/* allocate basic report buffers */
		_reports = new ringbuffer::RingBuffer(2, sizeof(airspeed_s));

		if (_reports == nullptr) {
			PX4_ERR("mem err");
			ret = -1;
			break;
		}

		_class_instance = register_class_devname(DWM1001_DEFAULT_PORT);

		/* get a publish handle on the range finder topic */
  	_airspeed_pub = nullptr;

		if (_airspeed_pub == nullptr) {
			_airspeed_pub = orb_advertise(ORB_ID(airspeed), &airspeed);
		}


	} while (0);
	PX4_INFO("This is ret %d",ret);
	PX4_INFO("Before closing");
	/* close the fd */
	boot_time = hrt_absolute_time();
	::close(_fd);
	PX4_INFO("This is ret %d",ret);
	PX4_INFO("After closing");
	_fd = -1;

	if (ret == PX4_OK) {
		PX4_INFO("Launching start right now");
		start();
	}

	PX4_INFO("This is ret %d",ret);
	return ret;
	PX4_INFO("DWM1001 initialization compelted, serial port should be open");
}


int
DWM1001dev::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default polling rate */
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(_conversion_interval);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {

					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					int ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(_conversion_interval)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

ssize_t
DWM1001dev::read(device::file_t *filp, char *buffer, size_t buflen)
{
	PX4_INFO("DWM1001 read function started");
	unsigned count = buflen / sizeof(struct airspeed_s);
	//struct airspeed_s *rbuf = reinterpret_cast<struct airspeed_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* manual measurement - run one conversion */
	do {
		_reports->flush();

		/* wait for it to complete */
		px4_usleep(_conversion_interval);

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}

	} while (0);

	return ret;
	PX4_INFO("DWM1001 read function finished");
}

int
DWM1001dev::collect()
{
	PX4_INFO("DWM1001 collect function started");
	perf_begin(_sample_perf);
	char readbuf[30];
	int ret = 0;
	int bytes_available = 0;
	::ioctl(_fd, FIONREAD, (unsigned long)&bytes_available);

	if(bytes_available>0){
	 while(bytes_available>0) {
		/* read from the sensor (uart buffer) */
		ret = ::read(_fd, &readbuf[k], 1);
		if (ret < 0) {
			PX4_ERR("read err: %d", ret);
			perf_count(_comms_errors);
			perf_end(_sample_perf);
		}
		PX4_INFO("%c",readbuf[k]);
		k++;
		}
	}

/*
		if(readbuf[k] == 0x0A && readbuf[k-1] == 0x0D){
			//Incoming message format: $WI,<windspeed>,<direction>,<checksum><CR+LF>
			unsigned checksum_calc = readbuf[1]^readbuf[2]^readbuf[3]^readbuf[4]^readbuf[5]^readbuf[6]^	readbuf[7]^readbuf[8]^readbuf[9]^readbuf[10]^readbuf[11]^readbuf[12]^readbuf[13]^readbuf[14]^readbuf[15]^readbuf[16]^readbuf[17]^readbuf[18];
			unsigned checksum_val = strtoul(&readbuf[20],NULL,16);

			if(checksum_calc == checksum_val){
				_last_read = hrt_absolute_time();
				strtok(readbuf,",");
    		char *ws1 = strtok(NULL,",");
    		char *ws2;
    		if(ws1[5] == 0){
    		  ws2 = &ws1[6];
    		}
    		else{
    		  ws2 = &ws1[5];
    		}
    		//(void)ws2;
    		char *dir = strtok(NULL,",");
    		//(void)dir;
    		float windspeed = atof(ws2);
    		float direction = ((atof(dir) - 0) * 71) / 4068;
    		float aspd = windspeed * cosf(direction);

    		if(aspd < 0.0f){
    			aspd = 0.0f;
    		}

				airspeed.timestamp = hrt_absolute_time();
				airspeed.indicated_airspeed_m_s = aspd;
				airspeed.true_airspeed_m_s = aspd;
				airspeed.air_temperature_celsius = -1000.0;
				airspeed.confidence = 1.0f;
				status.aspd_fault_declared = false;
				status.aspd_use_inhibit = false;
				k=0;
				break;
			}
		}
		k++;
		if(k > 25){
			return -EAGAIN;
		}
	}
}
*/

/*
	if(hrt_absolute_time() - boot_time > 5000000){
		if(hrt_absolute_time()-_last_read > 700000){
			mavlink_log_critical(&mavlink_log_pub, "AIRSPEED DATA LINK LOST  - stopping use");
			status.aspd_fault_declared = true;
			status.aspd_use_inhibit = true;
			return -EAGAIN;
		}
	}
*/
	//orb_publish(ORB_ID(airspeed), _airspeed_pub, &airspeed);
	ret = OK;
	perf_end(_sample_perf);
	return ret;
}

void
DWM1001dev::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&DWM1001dev::cycle_trampoline, this, 1);
}

void
DWM1001dev::stop()
{
	work_cancel(HPWORK, &_work);
}

void
DWM1001dev::cycle_trampoline(void *arg)
{
	DWM1001dev *dev = (DWM1001dev *)arg;

	dev->cycle();
}

void
DWM1001dev::cycle()
{
	/* fds initialized? */
	if (_fd < 0) {
		/* open fd */
		_fd = ::open(_port, O_RDONLY | O_NOCTTY);
	}

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		int collect_ret = collect();

		if (collect_ret == -EAGAIN) {
			PX4_INFO("stopping driver");
			stop();
			PX4_INFO("driver stopped");
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 */
		if (_measure_ticks > USEC2TICK(_conversion_interval)) {

			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(HPWORK,
				   &_work,
				   (worker_t)&DWM1001dev::cycle_trampoline,
				   this,
				   _measure_ticks - USEC2TICK(_conversion_interval));

			return;
		}
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&DWM1001dev::cycle_trampoline,
		   this,
		   USEC2TICK(_conversion_interval));
}

void
DWM1001dev::print_info()
{
	printf("Using port '%s'\n", _port);
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %d ticks\n", _measure_ticks);
	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace dwm1001dev
{

DWM1001dev	*g_dev;

int start(const char *port);
int stop();
int test();
int info();
void usage();

/**
 * Start the driver.
 */
int
start(const char *port)
{
	PX4_INFO("Local start function launched");
	int fd;

	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return 1;
	}

	/* create the driver */
	g_dev = new DWM1001dev(port);
	PX4_INFO("g_dev created");

	if (g_dev == nullptr) {
		PX4_INFO("Failed due to g_dev=nullptr");
		goto fail;
	}

	if (OK != g_dev->init()) {
		PX4_INFO("Failed due to the OK =! g_dev");
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = px4_open(DWM1001_DEFAULT_PORT, O_RDONLY);
	PX4_INFO("fd is set");

	if (fd < 0) {
		PX4_ERR("Opening device '%s' failed", port);
		goto fail;
	}

	if (px4_ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_INFO("Failed due to ioctl");
		goto fail;
	}

	return 0;

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	PX4_ERR("driver start failed");
	return 1;
}

/**
 * Stop the driver
 */
int stop()
{
	if (g_dev != nullptr) {
		PX4_INFO("stopping driver");
		delete g_dev;
		g_dev = nullptr;
		PX4_INFO("driver stopped");

	} else {
		PX4_ERR("driver not running");
		return 1;
	}

	return 0;
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
int
test()
{
	struct airspeed_s report;
	ssize_t sz;

	int fd = px4_open(DWM1001_DEFAULT_PORT, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("%s open failed (try 'dwm1001dev start' if the driver is not running", DWM1001_DEFAULT_PORT);
		return 1;
	}

	/* do a simple demand read */
	sz = px4_read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		PX4_ERR("immediate read failed");
		close(fd);
		return 1;
	}

	print_message(report);

	/* start the sensor polling at 2 Hz rate */
	if (OK != px4_ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		PX4_ERR("failed to set 2Hz poll rate");
		return 1;
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		px4_pollfd_struct_t fds{};

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		int ret = px4_poll(&fds, 1, 2000);

		if (ret != 1) {
			PX4_ERR("timed out");
			break;
		}

		/* now go get it */
		sz = px4_read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			PX4_ERR("read failed: got %zi vs exp. %zu", sz, sizeof(report));
			break;
		}

		print_message(report);
	}

	/* reset the sensor polling to the default rate */
	if (OK != px4_ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		PX4_ERR("failed to set default poll rate");
		return 1;
	}

	PX4_INFO("PASS");
	return 0;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	return 0;
}

/**
 * Print a little info on how to use the driver.
 */
void
usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Serial bus driver for the Benewake TFmini LiDAR.

Most boards are configured to enable/start the driver on a specified UART using the SENS_DWM1001dev_CFG parameter.

Setup/usage information: https://docs.px4.io/en/sensor/dwm1001dev.html

### Examples

Attempt to start driver on a specified serial device.
$ dwm1001dev start -d /dev/ttyS1
Stop driver
$ dwm1001dev stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("dwm1001dev", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start","Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", false);
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 1, 25, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop","Stop driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test","Test driver (basic functional tests)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("info","Print driver information");
}

} // namespace

int
dwm1001dev_main(int argc, char *argv[])
{
	int ch;
	const char *device_path = DWM1001_DEFAULT_PORT;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "R:d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {

		case 'd':
			device_path = myoptarg;
			break;

		default:
			PX4_WARN("Unknown option!");
			return -1;
		}
	}

	if (myoptind >= argc) {
		goto out_error;
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[myoptind], "start")) {
		if (strcmp(device_path, "") != 0) {
			return dwm1001dev::start(device_path);

		} else {
			PX4_WARN("Please specify device path!");
			dwm1001dev::usage();
			return -1;
		}
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[myoptind], "stop")) {
		return dwm1001dev::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[myoptind], "test")) {
		return dwm1001dev::test();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[myoptind], "info") || !strcmp(argv[myoptind], "status")) {
		dwm1001dev::info();
		return 0;
	}

out_error:
	PX4_ERR("unrecognized command");
        dwm1001dev::usage();
	return -1;
}
