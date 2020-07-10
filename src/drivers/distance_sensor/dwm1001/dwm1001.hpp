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
 * @file dwm1001.hpp
 * DWM1001 driver
 * @author Francesco Malacarne - s260199@studenti.polito.it
 */

 #include <px4_platform_common/px4_config.h>
 #include <px4_platform_common/tasks.h>
 #include <px4_platform_common/posix.h>
 #include <px4_platform_common/log.h>
 #include <px4_platform_common/defines.h>
 #include <px4_platform_common/module.h>
 #include <px4_platform_common/module_params.h>
 #include <px4_platform_common/posix.h>

 #include <sys/types.h>
 #include <sys/stat.h>
 #include <fcntl.h>

 #include <unistd.h>
 #include <stdlib.h>
 #include <stdio.h>
 #include <stdarg.h>
 #include <poll.h>
 #include <string.h>
 #include <math.h>

 #include <termios.h>
 #include <unistd.h>
 #include <stdbool.h>
 #include <errno.h>
 #include <drivers/drv_hrt.h>
 #include <lib/perf/perf_counter.h>

 #include <uORB/uORB.h>
 #include <uORB/Subscription.hpp>
 #include <uORB/Publication.hpp>
 #include <uORB/topics/parameter_update.h>
 #include <uORB/topics/dwm1001.h>
 #include <uORB/topics/dwm1001_raw.h>

 #define DWM1001_PORT	"/dev/ttyS2" // see https://github.com/PX4/px4_user_guide/issues/417

 #ifdef NAN
 /* NAN is supported */
 #endif

 extern "C" __EXPORT int dwm1001_main(int argc, char *argv[]);

class DWM1001 //: public ModuleBase<DWM1001>, public ModuleParams
{
public:
  /**
   * Print the correct usage
   */
  static void usage(const char *reason);
  /**
   * Serial communication configuration
   */
  int set_uart_baudrate(const int _fd);
  /**
   * UART initialization
   */
  static int uart_init(char const *uart_name);
  /**
   * DWM1001 programming (lec)
   */
  int dwm1001_programming(const int _fd);

  int ret;

  perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
  perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
};

/**
 * Main loop of dwm1001 driver
 */
int dwm1001_thread_main(int argc, char *argv[]);
