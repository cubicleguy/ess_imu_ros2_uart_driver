# README for Epson IMU Driver using UART interface for ROS2 Node

## What is this repository for?

* This code provides interface between Epson IMU (newer G325/G365/G370 or mature G320/G354/G364/V340) and ROS2 using the UART interface.
* The UART connection can be either direct or by USB-serial converter such as FTDI bridge ICs.
* The src/epson_imu_node.cpp is the ROS2 C++ wrapper used to communicate with ROS2
* The other source files in src/ are based on the C driver released by Epson:
  [Epson IMU UART-only Linux User-space Driver Example](https://vdc.epson.com/imu-products/imu-inertial-measurement-units)
* Information about ROS2, and tutorials can be found: [ROS.org](https://index.ros.org/doc/ros2/)

## What kind of hardware or software will I likely need?

* Epson USB evaluation board or equivalent FTDI USB-Serial interface to connect to ROS host (tty/serial) [See Evaluation Boards](https://global.epson.com/products_and_drivers/sensing_system/technical_info/evaluation_tools/)
* Epson IMU (V340/G320/G325/G354/G364/G365/G370) [IMU models](https://global.epson.com/products_and_drivers/sensing_system/imu/)
* ROS2 Dashing or later (via download) [ROS.org](https://index.ros.org/doc/ros2/Installation/#installationguide)
* This software was developed and tested on the following:
```
  ROS2:        Dashing & Eloquent
  Description: Ubuntu 18.04.4 LTS
  Release:     18.04
  Codename:    bionic
```

## How do I use the driver?

* This code assumes that the user is familiar with building ROS2 packages using the colcon build process.
* This is *NOT* detailed instructions describing step by step procedures on how to build and install this ROS2 driver.
* Please refer to the ROS.org website for more detailed instructions on the ROS package build process. [ROS.org](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/)
* *NOTE:* At bare minimum, you must modify the CMakeLists.txt to select the desired IMU model, and build the package atleast once or any time you use a different IMU model.
* If the IMU model is unchanged, then subsequent changes to IMU settings can be done by editing the IMU model specific launch.py file.
* The IMU model specific launch.py file should only be called with the same colcon built binary matching the IMU model.
* *NOTE* Do not mix IMU model launch.py files & IMU model colcon built binaries.


## How do I use the driver if usleep() is not supported for time delays?

* NOTE: In the hcl_linux.c, there are functions for time delays in millisecond and microseconds using seDelayMS() and seDelayMicroSecs(), respectively.
* On embedded Linux platforms, these may need to be redirected to platform specific delay routines if usleep() is not supported.
* For example on RaspberryPi, the time delay functions for millisecond and microseconds can be redirected to WiringPi library delay() and delayMicroseconds(), respectively.
* If a hardware delay is not available from a library, then a software delay loop is possible but not preferred.

## How do I use the driver with GPIOs to control IMU RESET#, DRDY, EXT pins?

* Because this driver connects to the IMU using the UART interface, the use of GPIO pins for connecting to the IMU RESET#, EXT, or DRDY is purely optional and mainly intended for embedded Linux platforms (non-PC based).
* When possible, connecting the RESET# is recommended to force Hardware Reset during every IMU initialization, for better robustness.
* Although this code does not implement GPIO functions, this code is structured to easily redirect to low-level hardware GPIO function calls for ease of implementation.
* There is no standard method to implement GPIO connections on embedded Linux platform, but the following files typically need changes:

```
  src/hcl_linux.c
  src/hcl_gpio.c
  src/hcl_gpio.h
```

* Typically, an external library needs to be invoked to initialize & enable GPIO HW functions.

* This typically requires changes to hcl_linux.c

  - add #include to external library near the top of hcl_linux.c
  - add the initialization call inside the seInit() function in hcl_linux.c

For example on an Raspberry Pi, the following changes can be made to hcl_linux.c:

```
  .
  .
  .
  #include <stdint.h>
  #include <stdio.h>
  #include <wiringPi.h>  // <== Added external library

  int seInit(void)
  {
    // Initialize wiringPi libraries                                                   // <== Added
    printf("\r\nInitializing libraries...");                                           // <== Added
    if(wiringPiSetupGpio() != 0) {                                                     // <== Added external library initialization
      printf("\r\nError: could not initialize wiringPI libraries. Exiting...\r\n");    // <== Added
      return NG;                                                                       // <== Added
    }                                                                                  // <== Added
    printf("...done.");

    return OK;
  }
  .
  .
  .

```

* Typically, the GPIO pins need to be assigned according to pin numbering specific to the HW platform.

* This typically requires changes to hcl_gpio.h

For example on an Raspberry Pi, the following changes to hcl_gpio.h with the following pin mapping:

```
    Epson IMU                   Raspberry Pi
    ---------------------------------------------------
    EPSON_RESET                 RPI_GPIO_P1_15 (GPIO22) Output
    EPSON_DRDY                  RPI_GPIO_P1_18 (GPIO24) Input

```

```
  // Prototypes for generic GPIO functions
  int gpioInit(void);
  int gpioRelease(void);

  void gpioSet(uint8_t pin);
  void gpioClr(uint8_t pin);
  uint8_t gpioGetPinLevel(uint8_t pin);

  #define RPI_GPIO_P1_15              22                    // <== Added
  #define RPI_GPIO_P1_18              24                    // <== Added

  #define EPSON_RESET                 RPI_GPIO_P1_15        // <== Added
  #define EPSON_DRDY                  RPI_GPIO_P1_18        // <== Added
  .
  .
  .
```


* Typically, the external library will have GPIO pin control functions such as set_output, set_input, set, reset, read_pin_level, etc...

* This requires changes to hcl_gpio.c

  - Redirect function calls in hcl_gpio.c for gpioInit(), gpioRelease(), gpioSet(), gpioClr(), gpioGetPinLevel() to equivalent external library pin control functions.

  - For example on an Raspberry Pi, the following changes to hcl_gpio.c:

```
  #include "hcl.h"
  #include "hcl_gpio.h"
  #include <wiringPi.h>                         // <== Added external library
  .
  .
  .

  int gpioInit(void)
  {
    pinMode(EPSON_RESET, OUTPUT);               // <== Added external call RESET Output Pin
    pinMode(EPSON_DRDY, INPUT);                 // <== Added external call DRDY Input Pin
    pullUpDnControl(EPSON_DRDY, PUD_OFF) ;      // <== Added external call Disable any internal pullup or pulldown resistance

    return OK;
  }
  .
  .
  .
  int gpioRelease(void)
  {
    return OK;
  }
  .
  .
  .
  void gpioSet(uint8_t pin)
  {
    digitalWrite(pin, HIGH);                    // <== Added external call set pin HIGH
  }

  .
  .
  .
  void gpioClr(uint8_t pin)
  {
    digitalWrite(pin, LOW);                     // <== Added external call set pin LOW
  }

  .
  .
  .
  uint8_t gpioGetPinLevel(uint8_t pin)
  {
    return (digitalRead(pin));                  // <== Added external call to return pin state of input pin
  }

  .
  .
  .
```


## How do I build, install, run this ROS2 package?

The Epson IMU ROS2 driver is designed to build in the ROS colcon build environment.
Therefore, a functional colcon workspace in ROS2 is a prerequisite.
Refer to the ROS2 Tutorials for more info: [ROS2 Tutorial](https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/)

For more information on ROS & colcon setup refer to
[Installing and Configuring ROS Environment](https://index.ros.org/doc/ros2/Tutorials/Configuring-ROS2-Environment/).


1. Place files (including folders) into a new folder within your colcon workspace "src" folder.
   For example, we recommend using the folder name "epson_imu_uart_ros2"
```
   <colcon_workspace>/src/epson_imu_uart_ros2/ <-- place files here
```
2. Modify the CMakeLists.txt to select the desired Epson IMU model that is attached to the ROS system.
   Refer to the comment lines inside the CMakeLists.txt for additional info.
   *NOTE:* You *MUST* re-build using colcon build when changing IMU models.

3. From the colcon_workspace folder run "colcon build" to build all ROS2 packages located in the <colcon_workspace>/src/ folder.
   To build this specific package type the following:
```
   <colcon_workspace>/colcon build --packages-select epson_imu_uart_ros2 --symlink-install
```
   Re-run the above "colcon build" command to rebuild the driver after making any changes to the CMakeLists.txt or any of the .c or .cpp or .h source files.
   It is not necessary to "colcon build" if changes are only made to the launch.py files

   *NOTE:* It is recommended to change IMU settings by editing the parameters in the ROS2 launch.py file, wherever possible, instead of modifying the .c or .cpp source files directly

4. Reload the current ROS environment variables that may have changed after the colcon build process.
```
   From the <colcon_workspace>: . install/setup.bash
```

5. Modify the appropriate launch file for the IMU model in the launch/ folder to set your desired IMU configure parameter options at runtime:


Parameter            | Comment
-------------------- | -------------
serial_port          | specifies the serial port name i.e. /dev/ttyUSB0
frame_id             | specifies the value in the IMU message frame_id field
imu_topic            | specifies the topic name for publishing IMU messages
burst_polling_rate   | specifies the driver internal polling rate for the serial port input buffer (typically does not need changing)
imu_dout_rate        | specifies the IMU output data rate
imu_filter_sel       | specifies the IMU filter setting
quaternion_output_en | enables or disables quaternion output (supported only be G325PDF1 or G365PDF1)
atti_profile         | specifies the attitude motion profile (supported only be G325PDF1 or G365PDF1)
output_32bit_en      | enables or disables outputing sensor data in 32 or 16 bit
time_correction_en   | enables time correction function using IMU counter reset function & external 1PPS connection to IMU GPIO2/EXT pin. Cannot be used with external trigger.
ext_trigger_en       | enables triggering of IMU samples using IMU external trigger function on IMU GPIO2/EXT pin. Cannot be used with time_correction function which uses the counter reset function.

   *NOTE:* The ROS2 launch.py file passes IMU configuration settings to the IMU at runtime.
           Therefore does not rebuilding with colcon when changing the launch file.

6. To start the Epson IMU ROS2 driver use the appropriate launch file (located in launch/) from console.

   For example, for the Epson G365 IMU:
```
   <colcon_workspace>/ros2 launch epson_imu_uart_ros2 g325_g365_launch.py
```

   - The launch.py file contains parameters for configuring settings at runtime:

   - All parameters are described in the inline comments of the launch file.

     *g320_g354_g364_launch.py*
       - For G320/G354/G364, outputs to ROS topic imu/data_raw (gyro, accel, but no quaternion orientation)

     *g325_g365q_launch.py*
       - For G325PDF1/G365PDx1, outputs to ROS topic imu/data (gyro, accel data, including quaternion orientation)

     *g325_g365_launch.py*
       - For G325PDF0/G365PDx0, outputs to ROS topic imu/data_raw (gyro, accel data, but no quaternion orientation)

     *g370_launch.py*
       - For G370, outputs to ROS topic imu/data_raw (gyro, accel, but no quaternion orientation)

     *v340_launch.py*
       - For V340, outputs to ROS topic imu/data_raw (gyro, accel, but no quaternion orientation)


### Example console output of colcon build for G370PDF0:
```
guest@guest-VirtualBox:~/dev_ws$ colcon build --packages-select epson_imu_uart_ros2 --symlink-install
Starting >>> epson_imu_uart_ros2
Finished <<< epson_imu_uart_ros2 [3.65s]                       

Summary: 1 package finished [3.84s]
```


### Example console output of launching ROS2 node for G370PDF0:
```
guest@guest-VirtualBox:~/dev_ws$ ros2 launch epson_imu_uart_ros2 g370_launch.py 
[INFO] [launch]: All log files can be found below /home/guest/.ros/log/2020-08-17-08-48-35-708704-guest-VirtualBox-21291
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [imu_node-1]: process started with pid [21301]
[imu_node-1] [INFO] [epson_node]: serial_port:		/dev/ttyUSB0
[imu_node-1] [INFO] [epson_node]: frame_id:		imu_link
[imu_node-1] [INFO] [epson_node]: time_correction_en:	0
[imu_node-1] [INFO] [epson_node]: ext_trigger_en:	0
[imu_node-1] [INFO] [epson_node]: burst_polling_rate:	4000.0
[imu_node-1] [INFO] [epson_node]: imu_dout_rate:	4
[imu_node-1] [INFO] [epson_node]: imu_filter_sel:	6
[imu_node-1] [INFO] [epson_node]: imu_topic:		/epson_imu/data_raw
[imu_node-1] [INFO] [epson_node]: output_32bit_en:	1
[imu_node-1] [WARN] [epson_node]: Not specified param quaternion_output_en. Set default value:	0
[imu_node-1] [WARN] [epson_node]: Not specified param atti_profile. Set default value:	0
[imu_node-1] Attempting to open port.../dev/ttyUSB0
[imu_node-1] [INFO] [epson_node]: Checking sensor power on status...
[imu_node-1] [INFO] [epson_node]: Initializing Sensor...
[imu_node-1] [INFO] [epson_node]: Epson IMU initialized.
[imu_node-1] [INFO] [epson_node]: PRODUCT ID:	G370PDFN
[imu_node-1] [INFO] [epson_node]: SERIAL ID:	N0000023
[imu_node-1] 
```


## What does this ROS2 IMU Node Publish as an Messages?

The Epson IMU ROS2 driver will publish messages which will vary slightly on the IMU model and output configuration.
- For IMU models such as G320/G354/G364/G370/V340, the IMU messages will only contain fields for angular rate (gyro) and linear acceleration (accel) data.
- For IMU models G325/G365 there is quaternion output function:
  - IMU messages can *also* output orientation from the internal extended Kalman Filter when quaternion output is *enabled*.
  - IMU messages will only contain angular rate (gyro) and linear acceleration (accel) data when quaternion output is *disabled*.

The Epson IMU ROS2 driver will publish IMU messages as convention per [REP 145](http://www.ros.org/reps/rep-0145.html).

### Without Quaternion Output
For non-quaternion output models, the ROS2 driver will publish to the following ROS topics:
```
/epson_imu/data_raw <-- orientation field will output 0's
```


#### ROS Topic Message /<imu_topic>/data_raw
```
---
header:
  stamp:
    sec: 1602020116
    nanosec: 892343523
  frame_id: imu_link
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 0.0
orientation_covariance: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity:
  x: 0.0008243194897659123
  y: 9.91016931948252e-05
  z: -0.00020670934463851154
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration:
  x: -1.4295457601547241
  y: -2.184906244277954
  z: 9.49894905090332
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---

```


### With Quaternion Output
For quaternion output models, the ROS2 driver will publish to the following ROS topics:
```
/epson_imu/data <-- orientation field will contain quaternions
```


#### ROS Topic Message /<imu_topic>/data

```
---
header:
  stamp:
    sec: 1602020004
    nanosec: 667342534
  frame_id: imu_link
orientation:
  x: -0.11234959959983826
  y: 0.07211977988481522
  z: 0.007866359315812588
  w: 0.9910168647766113
orientation_covariance: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity:
  x: -0.0018992983968928456
  y: 0.0007579181110486388
  z: 0.0010170674649998546
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration:
  x: -1.4219565391540527
  y: -2.176177740097046
  z: 9.500624656677246
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---


## Why am I seeing high latencies or slower than expected IMU data rates when using USB-UART bridges?
   
If your connection between the Epson IMU UART and the Linux host is by FTDI (or similar USB-UART bridge devices,
the default latency_timer setting may be too large (i.e. typically 16msec).

There are 2 recommended methods to reduce this value to 1msec.

### Modifying latency_timer by sysfs mechanism

The example below reads the latency_timer setting for /dev/ttyUSB0 which returns 16msec.
Then, the latency_timer is set to 1msec, and confirmed by readback.

*NOTE*: May require root (sudo su) access on your system to modify. 
```
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
16
echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
1
```

### Modifying low_latency flag using setserial utility

The example below sets the low_latency flag for /dev/ttyUSB0.
This will have the same effect as setting the latency_timer to 1msec.
This can be confirmed by running the setserial command again.

```
user@user-VirtualBox:~$ setserial /dev/ttyUSB0
/dev/ttyUSB0, UART: unknown, Port: 0x0000, IRQ: 0

user@user-VirtualBox:~$ setserial /dev/ttyUSB0 low_latency

user@user-VirtualBox:~$ setserial /dev/ttyUSB0
/dev/ttyUSB0, UART: unknown, Port: 0x0000, IRQ: 0, Flags: low_latency
```

## Package Contents

The Epson IMU ROS2 driver-related sub-folders & root files are:
```
   launch/        <== various example launch files for Epson IMU models
   src/           <== source code for ROS2 node C++ wrapper, IMU C driver, and additional README_src.md
   CmakeLists.txt <== build script for colcon build
   package.xml    <== colcon package description
   README.md      <== general README for the ROS2 driver
```

## License

### The Epson IMU C++ Wrapper ROS2 Node is released under BSD-3 license.

[This software is BSD-3 licensed.](http://opensource.org/licenses/BSD-3-Clause)

Original Code Development:
Copyright (c) 2019, Carnegie Mellon University. All rights reserved.

Additional Code contributed:
Copyright (c) 2020, Seiko Epson Corp. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
   may be used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

### The Epson IMU C driver software is released as public domain.

THE SOFTWARE IS RELEASED INTO THE PUBLIC DOMAIN.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
NONINFRINGEMENT, SECURITY, SATISFACTORY QUALITY, AND FITNESS FOR A
PARTICULAR PURPOSE. IN NO EVENT SHALL EPSON BE LIABLE FOR ANY LOSS, DAMAGE
OR CLAIM, ARISING FROM OR IN CONNECTION WITH THE SOFTWARE OR THE USE OF THE
SOFTWARE.


## References
1. https://index.ros.org/doc/ros2/
2. https://github.com/technoroad/ADI_IMU_TR_Driver_ROS2
