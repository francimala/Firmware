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
 * @author Francesco Malacarne - francesco.malacarne@gmail.com
 */

#include "dwm1001.hpp"

 __EXPORT int dwm1001_main(int argc, char *argv[]);

 /* static variables */
 static bool thread_should_exit = false;        /**< dwm1001 exit flag */
 static bool thread_running = false;        /**< dwm1001 status flag */
 static int dwm1001_task;                /**< Handle of dwm1001 task / thread */

 int set_uart_baudrate(const int _fd)
 {
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
    return false;
  }

  if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
    PX4_ERR("CFG: %d OSPD", termios_state);
    ret = -1;
    return false;
  }

  if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
    PX4_ERR("baud %d ATTR", termios_state);
    ret = -1;
    return false;
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

  if (_fd < 0) {
    PX4_ERR("FAIL: laser fd");
    ret = -1;
    break;
  }
  */
  return true;
 }

 int uart_init(char const *uart_name)
 {
     int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

     if (serial_fd < 0) {
         PX4_ERR("failed to open port: %s", uart_name);
         return false;
     }
     return serial_fd;
 }

 void
 usage(const char *reason)
 {
     if (reason) {
         PX4_INFO("%s", reason);
     }

     PX4_INFO("usage: dwm1001 {start|stop}");
     exit(1);
 }

 int
 dwm1001_main(int argc, char *argv[])
 {

     if (argc < 2) {
         usage("missing command");
     }

     if (!strcmp(argv[1], "start")) {

         if (thread_running) {
             PX4_INFO("running");
             /* this is not an error */
             exit(0);
         }

         thread_should_exit = false;
         dwm1001_task = px4_task_spawn_cmd("dwm1001",
                         SCHED_DEFAULT,
                         SCHED_PRIORITY_DEFAULT,
                         3000,
                         dwm1001_thread_main,
                         (char * const *)&argv[0]);
         thread_running = true;
         exit(0);
     }

     if (!strcmp(argv[1], "stop")) {
         thread_should_exit = true;
         exit(0);
     }

     if (!strcmp(argv[1], "status")) {
         if (thread_running) {
             PX4_INFO("running");

         } else {
             PX4_INFO("not started");
         }

         exit(0);
     }

 return 0;
 }

 int
 dwm1001_thread_main(int argc, char *argv[])
 {
     int cnt = 0;
     int readlen = 36; // how many characters do I want to read? This value must be >= 36
     char readbuf[readlen-1];
     for (int rst = 0; rst < readlen; rst++) {
       readbuf[rst] = '0';
     }


     px4_usleep(1000);

     PX4_INFO("This is readlen %d", readlen);

     //UART open
     int uart = uart_init(DWM1001_PORT);
     if(false == uart)return -1;
     if(false == set_uart_baudrate(uart)){
             PX4_INFO("ERROR: initialization failed");
             return -1;
     }
     PX4_INFO("DWM1001 initialization succeded!");

     while (!thread_should_exit) {

       // ADDED
       perf_begin(_sample_perf);

       ret = ::read(uart, &readbuf[0], readlen); // ret is the number of read values.

       //printf("%c%c%c%c\n", readbuf[32], readbuf[33], readbuf[34], readbuf[35]);

       for (cnt = 0; cnt < ret; cnt++) {
         printf("%c",readbuf[cnt]);
       }
     }

     thread_running = false;
     close(uart);
     perf_free(_sample_perf);
     perf_free(_comms_errors);
     return 0;
}



/*
     //main thread loop
     while (!thread_should_exit) {

       ret = ::read(uart, &readbuf[0], readlen); // ret is the number of read values. It is
       px4_sleep(2);
       while(readbuf[0] != 'D') {
         ret = ::read(uart, &readbuf[0], readlen);
       }
       PX4_INFO("This is ret: %d, now I wait for 2 seconds, then I'll start the for cycle", ret);
       px4_usleep(1000);
       if(ret == readlen) {
         for (cnt = 0; cnt < ret; cnt++) {
           printf("%c",readbuf[cnt]);
         }
         px4_usleep(1000);
         printf("First\nSecond\n");
         px4_sleep(2);
         PX4_INFO("Waited 2 seconds, this is count %d, now I should see ret",cnt);
       }

       else {

         PX4_INFO("The value of ret was not good! This is how readbuf looks like");
         px4_sleep(1);
         for (int rst = 0; rst < readlen; rst++) {
           printf("%c", readbuf[rst]);
         }
         printf("\n");
         px4_sleep(1);
         PX4_INFO("Now I'm resetting the vector");
         px4_sleep(1);
         for (int rst = 0; rst < readlen; rst++) {
           readbuf[rst] = '0';
         }
         PX4_INFO("The readbuf was reset, here it is:");
         px4_sleep(1);
         for (int rst = 0; rst < readlen; rst++) {
           printf("%c", readbuf[rst]);
           //PX4_INFO("This is count: %d", cnt);
         }
         printf("\n");
         px4_sleep(1);
       }


     }
     thread_running = false;
     close(uart);
     return 0;
 }
 */
