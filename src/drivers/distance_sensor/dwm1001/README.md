## dwm1001 description
### Components
dwm1001.h is the header for the dwm1001 module.

mdule.yaml is used for some configuration parameters.

dwm1001.cpp is the actual driver.

CMakeLists.txt must be compliant with the above mentioned components. If you have doubts see the official developer guide in section "Writing your first application".

Old_queue folder contains the sketch of the driver implemented as work_queue instead of stand alone task. However, it is not working, but I leave it as it may be helpful for future development.

Old_working folder is a backup.

### What is its goal
This driver should initialize the communication with the DWM1001-dev module, program it with command lec, parse the read information and publish them in a clear way in a new topic named dwm1001.

### How does it work
The driver execution is very simple: whenever started with command dwm1001 start, the main function is launched: a task named dwm1001, with default priority and stack equal to 3000 is created. Such a task points at the function dwm1001 thread main implementing the actual module as a basic task, namely composed by a first initialization part and then an endless loop collecting data and publishing them on
a specific topic. 

During the initialization part three functions are called for setting the DWM1001 module up:

1. uart init: responsible for opening the serial communication with the correct port (specified within the header file, for instance in case of TELEM2 it is /dev/ttyS2).
2. set uart baudrate: responsible for the configuration of the serial communication parameters: 115200-8N1, namely 115200 as baudrate, start bit, 8 bit data length, no parity bit, 1 stop bit.
3. dwm1001 programming: responsible for programming the DWM1001 module so that it sends the correct information to the drone.
Then, during the endless loop, the read information are parsed, distances are isolated and published on a new topic named dwm1001.h according to the custom message definition dwm1001.msg.

#### uart_init
It is basically aimed at opening the serial port with the correct interface (TELEM2). The standard open function is used and the parameter to be set are:
- RDWR: to open the the communication in read and write mode.
- NOCTTY: needed to be portable.
- SYNC: to not cause problems with the writing operations during the module programming phase.

#### set_uart_baudrate
It follows the standard C configurations for setting the communication up (115200-8N1). The only relevant command to be added is tcflush in order to flush both the received data but not read and the written data but not transmitted (using parameter TCIOFLUSH).

#### dwm1001_programming
This is the non-standard component of the code since it needs to directly interact with the module,
programming it to respond with specific commands. The first operation to be performed is the module
reset, because the initial condition is not known, although required. According to the module datasheet,
the command reset\r is needed. The main issue with write operation is that it requires some time to
actually write the desired bits, this is why the function tcdrain must be involved. Nevertheless, the
module itself needs some time to digest the incoming reset command, therefore an escamotage should be
used to let some time pass without freezing the application (so no px4 sleep command can be used):
the absolute time is called, a fixed amount of time is added to the absolute time, and then a cycle starts
until the next absolute time is the one given by the previous, plus the threshold; at that time the code
can continue.

After the reset command, two consecutive enter commands are passed to land into the programming
mode. Before continuing with next instructions some time must be waited.e.

Once the board is in the programming mode, it is possible to write the ultimate command used to extract
data from the module. In this case, the command is lec.

The lec command is going to return a message containing 3 relevant information:
- The number of anchors being detected and used to estimate the position of the tag.
- The position of the i-th anchor with respect to the coordinate system’s origin (this is actually
defined by the environment designer using the DWM1001 app) plus the distance between the tag
and the i-th anchor (measured).
- The tag position estimation plus the quality level of the estimation. This part of the message is
present only if there is a sufficient number of anchors.
An example of message could be:
DIST,4,AN0,1151,5.00,8.00,2.25,6.44,AN1,0CA8,0.00,8.00,2.25,6.50,AN2,111C,5.00,0.00,
2.25,3.24,AN3,1150,0.00,0.00,2.25,3.19,POS,2.55,2.01,1.71,98\r\n
Where:
- DIST: indicator for the first part of the message containing the location of the anchors in the
reference system. These coordinates are defined by the environment designer and set by means of
the Android app.
- 4: number of anchors detected.
- AN0,1151: number of the anchor + anchor’s ID.
- 5.00,8.00,2.25: anchor position coordinates into the reference frame defined a priori.
- 6.44: distance between the i-th anchor and the tag (measured).
- POS: indicator for the second part of the message containing the estimated position of the tag into
the reference system.
- 2.55,2.01,1.71: X, Y and Z estimated coordinates of the tag into the reference system.
- 98: quality level of the estimation.
- \r\n: end of the message.

#### Driver's main functions
The main challenges to be managed by this driver where two:
1. The capability of reading a serial message with dynamic length.
2. The capability of adapting to the presence of different number of anchors.
To handle the first requirement, the design choice was to read serial characters 1 by 1, store them in a
specific container, and parse the entire message in a second instance. In particular, the driver will spot
DIST over the serial bus and it will start storing all components into a char array until the end of the
message is detected (\r\n). Once the message is stored in this array, it will be split into its relevant
components in order to be published on the topic of interest.
Managing the second requirement was a bit more difficult due to the PX4 architecture. In an ideal
condition it would be enough to read the number of anchors being detected, and to publish a topic
containing a float array with a length proportional to the number of anchors (in fact the number of
distances is proportional to such a number). However, PX4 requires topic messages to be with a fixed
length. As a consequence, there was the need of constraining the maximum number of anchors so that
the message will be published always with a fixed length equal to the established one. Before a cycle
starts, the driver will set to 0 the entire array and it will fill it only with the detected components (that
is proportional to the number of anchors read); the remaining ones will be set to 0.
The application subscriber will read the at least two values: the number of anchors and the float containing
the distances. Knowing the number of anchors, it will read the array containing the distances only until
the correct value, discarding the remaining ones.
If the POS indicator is detected within the message, a boolean variable will be set to true and the estimated
position will be delivered in the topic, otherwise the published array will be made of zeros. Similarly to
what described before, the user application will read both the array and the boolean variable deciding
what to do based on the values of these variables.
