/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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

/*
 * @file dwm1001.cpp
 * DWM1001 driver
 * @author Francesco Malacarne - s260199@studenti.polito.it
 */

#include "dwm1001.hpp"

 //__EXPORT int dwm1001_main(int argc, char *argv[]);

 /* static variables */
 static bool dwm1001_thread_should_exit = false;        /**< dwm1001 exit flag */
 static bool dwm1001_thread_running = false;            /**< dwm1001 status flag */
 static int dwm1001_task;                               /**< Handle of dwm1001 task / thread */

 int DWM1001::set_uart_baudrate(const int _fd)
 {
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
    //ret = -1;
    return false;
  }

  if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
    PX4_ERR("CFG: %d OSPD", termios_state);
    //ret = -1;
    return false;
  }

  uart_config.c_cflag |= (CLOCAL | CREAD);	// ignore modem controls
  uart_config.c_cflag &= ~CSIZE;
  uart_config.c_cflag |= CS8;			          // 8-bit characters
  uart_config.c_cflag &= ~PARENB;			      // no parity bit
  uart_config.c_cflag &= ~CSTOPB;			      // only need 1 stop bit
  uart_config.c_cflag &= ~CRTSCTS;		      // no hardware flowcontrol


  // setup for non-canonical mode
  uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
  uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  uart_config.c_oflag &= ~OPOST;
  // fetch bytes as they become available
  uart_config.c_cc[VMIN] = 1;
  uart_config.c_cc[VTIME] = 1;

  if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
    PX4_ERR("baud %d ATTR", termios_state);
    //ret = -1;
    return false;
  }

  tcflush(_fd, TCIOFLUSH); // flushes both data received but not read, and data written but not transmitted.

  return true;
}

 int DWM1001::uart_init(char const *uart_name)
 {
     int serial_fd = open(uart_name, O_RDWR | O_NOCTTY | O_SYNC);
     //int serial_fd = open(uart_name, O_RDWR | O_NOCTTY); // not working

     if (serial_fd < 0) {
         PX4_ERR("failed to open port: %s", uart_name);
         return false;
     }
     return serial_fd;
 }

 int DWM1001::dwm1001_programming(const int _fd)
 {

  /*
  hrt_abstime time_now = hrt_absolute_time();
  const hrt_abstime timeout_usec = time_now + 1000000; // us
  while (time_now < timeout_usec) {
   time_now = hrt_absolute_time();
  }
  */

  int num_bytes = 0;
  // Flush the receive buffer: data received but not read.
  tcflush(_fd, TCIFLUSH);

  num_bytes = ::write(_fd, "reset\r", 6); // \r is working, it counts as 1 char
  tcdrain(_fd);


  hrt_abstime time_now = hrt_absolute_time();
  const hrt_abstime timeout_usec = time_now + 2000000; // us
  while (time_now < timeout_usec) {
   time_now = hrt_absolute_time();
  }


  if(num_bytes != 6) {
    PX4_ERR("Something went wrong with writing reset!");
    return false;
  }

  num_bytes = ::write(_fd, "\r\r", 2); // \r is working
  tcdrain(_fd);


  time_now = hrt_absolute_time();
  const hrt_abstime timeout_usec_2 = time_now + 2000000; // us
  while (time_now < timeout_usec_2) {
    //PX4_INFO("time_now: %llu\ntimeout_usec: %llu", time_now, timeout_usec);
    time_now = hrt_absolute_time();
  }


  if(num_bytes != 2) {
    PX4_ERR("Something went wrong with writing enter twice!");
    return false;
  }

  num_bytes = ::write(_fd, "lec\r", 4); // \r is working
  tcdrain(_fd);

  if(num_bytes != 4) {
    PX4_ERR("Something went wrong with writing lec!");
    return false;
  }

  PX4_INFO("The dwm1001 module should be programmed with lec");

  return true;
 }

 void DWM1001::usage(const char *reason)
 {
     if (reason) {
         PX4_INFO("%s", reason);
     }

     PX4_INFO("usage: dwm1001 {start|stop}");
     exit(1);
 }

 int dwm1001_main(int argc, char *argv[])
 {

     if (argc < 2) {
         PX4_INFO("missing command");
     }

     if (!strcmp(argv[1], "start")) {

         if (dwm1001_thread_running) {
             PX4_INFO("running");
             /* this is not an error */
             exit(0);
         }

         dwm1001_thread_should_exit = false;
         dwm1001_task = px4_task_spawn_cmd("dwm1001",
                         SCHED_DEFAULT,
                         SCHED_PRIORITY_DEFAULT,
                         3000,
                         dwm1001_thread_main,
                         (char * const *)&argv[0]);
         dwm1001_thread_running = true;
         exit(0);
     }

     if (!strcmp(argv[1], "stop")) {
         dwm1001_thread_should_exit = true;
         exit(0);
     }

     if (!strcmp(argv[1], "status")) {
         if (dwm1001_thread_running) {
             PX4_INFO("running");

         } else {
             PX4_INFO("not started");
         }

         exit(0);
     }

 return 0;
 }

 int dwm1001_thread_main(int argc, char *argv[])
 {
//     int cnt = 0;
     DWM1001 _dwm1001;
     int readlen = 10; // how many characters do I want to read? This value must be >= 36
     char readbuf[readlen-1];
//     char data[36];

     PX4_INFO("The main task is now started, first the initializaiton, then the baudrate set");

     //UART open
     int uart = _dwm1001.uart_init(DWM1001_PORT);
     if(false == uart)return -1;
     if(false == _dwm1001.set_uart_baudrate(uart)){
             PX4_INFO("ERROR: initialization failed");
             return -1;
     }
     PX4_INFO("DWM1001 initialization succeded! This is uart: %d", uart);

     if(false == _dwm1001.dwm1001_programming(uart)) {
       PX4_INFO("ERROR: DWM1001 programming failed");
       return -1;
     }

     tcflush(uart, TCIOFLUSH);

     PX4_INFO("Initialization compeleted, the endless loop starts");
     while (!dwm1001_thread_should_exit) {

       // ADDED
       perf_begin(_dwm1001._sample_perf);

       ::read(uart, &readbuf[0], 1); // ret is the number of read values.

       printf("%c", readbuf[0]);

       perf_end(_dwm1001._sample_perf);
     }

     dwm1001_thread_running = false;
     close(uart);
     perf_free(_dwm1001._sample_perf);
     perf_free(_dwm1001._comms_errors);
     return 0;
}
