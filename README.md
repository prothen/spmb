# SPMB
## Brief
The Signal&Power Management Board (**SPMB**) provides firmware for Arduino UNO devices using Atmega328p microcontrollers facilitating a Robot Operating System (**ROS**) interface for a radio controlled (**RC**) supervised vehicle. Here the current configuration is based on the **TRAXXAS** platforms (in specific TRX4). The SPMB provides direct RC forwarding and publishing of actuated control signals to the ROS interface.  With a predefined fast switching sequence of channels the software based control algorithms are activated. Furthermore a failsafe for idled software connections is embedded such that RC operation is activated if less than *10Hz* control frequency is present. If a RC channel is not read successfully, e.g. broken wires or empty batteries, the channel is deactivated and the output goes in the channels default control signal. In case of velocity and steering this corresponds to 0 actuation.
In addition lowpass filtering of all control signals, both remote and software based is included and can be customised by adapting the parameters under `firmware/setup_macros.h`. The main and control loop time is configured and tested at *50Hz*. Theoretically much higher frequencies are possible, however interrupt reading capability greatly deteriorates with a fast main loop frequency. Therefore it is recommended to not vary these frequencies to higher values or otherwise stable performance can not be guaranteed.
Also the firmware provides a led signaling class which provides easily accessible information about the current SPMB's state machine status.  

More detailed **Documentation** can be found under https://prothen.github.io/spmb/index.html.

- **Main loop** in firmware.ino depending on
    - **StatusIndicator**       
        + si.h - LED signaling and exposing information to environment
    - **InterruptManager**      
        + input_rc.h - Input via RC
    - **ROSInterface**          
        + ros_interface.h - Input via ROS and publish actuated control
    - **LowPass**               
        + lowpass.h - Filter for output elements
    - **OutputDriverI2C** 
        + output_i2c.h - Output via I2C to timer PCB
    - **SetupSPMB**
        + setup.h - Initial pin configurations and setup of objects
    - **StateMachine**          
        + sm.h - Execute main logic loop and interface with all involved elements (see [here](https://prothen.github.io/spmb/classSPMB_1_1StateMachine.html))

***

## Features
- RC Forwarding
- Safety logic for both ROS- and RC-based control
- Actuation output via ROS for debugging and learning of manual controlled maneuvers or continuation of actuation-based estimator while supervised intervention
- Switching logic with state machine considering idle conditions, various error cases and providing supervisor interrupt via braking maneuver
- *50 Hz* actuation and lowpass filtered control
- OOP based modules with independent use cases both with and without ROS usage (serial and ros nodehandle.loginfo debugging possible)
- Installation of udev rules for unique device node under `/dev/spmb`
- Scripts for installing ros libraries to Arduino library folder `~/Arduino/libraries/`
- Doxygen based API as reference as basis for continuoous integration

***

## Execute the ROS Interface

- Run `roslaunch spmb run.launch`

(*If any configuration parameters regarding baudrate have been adjusted then parse the corresponding parameter in the launch file e.g. `roslaunch spmb run.launch baud:=yourbaudrateinteger`*)

- After approximately 3-4 seconds the terminal will output 

```
setting /run_id to 4e4bd514-a7b2-11e9-a7b3-4485007bad36
process[rosout-1]: started with pid [25122]
started core service [/rosout]
process[serial_node-2]: started with pid [25128]
[INFO] [1563271988.137866]: ROS Serial Python Node
[INFO] [1563271988.149294]: Connecting to /dev/spmb at 57600 baud
[ERROR] [1563272005.372626]: Unable to sync with device; possible link problem or link software version mismatch such as hydro rosserial_python with groovy Arduino
[INFO] [1563272005.404425]: Note: publish buffer size is 100 bytes
[INFO] [1563272005.406264]: Setup publisher on actuated [spmb/actuated]
[INFO] [1563272005.418630]: Note: subscribe buffer size is 100 bytes
[INFO] [1563272005.419878]: Setup subscriber on request [spmb/request]
```

(*Note that initially there will be a single error output regarding a sync problem. After this the publishers should be initialised and no further error messages should be shown.*)

***

## Dependencies

### Install Arduino Libraries

Go to `cd ~/Arduino/libraries`

- `git clone https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library.git`

- `git clone https://github.com/maniacbug/StandardCplusplus.git`

- Download and install Arduino (v1.8.5) (x64 https://www.arduino.cc/download_handler.php?f=/arduino-1.8.5-linux64.tar.xz)

- Build the repository to generate header files for messages with `catkin build` in any directory of your catkin workspace

- Source your workspace with `source ~/your_workspace/devel/setup.bash`

- Run the `make_library.sh` script in the root of this repository

- If desired install the udev rule file under `resources/99-spmb.rules`, which will install the permanent device node `dev/spmb` to avoid the deviating node identifier ttyACMx with x changing based on the amount of ttyACM devices or reconnections of the same device) 

### Interface with ROS
- After installing the udev rules (see below) the ROS Interface can be accessed with 
```rosrun rosserial_python serial_node.py _port:=/dev/spmb _baud:=57600```

- The `rosrun` command is conveniently embedded into the launch file under `launch/run.launch` (see *Execute the ROS Interface* section above)

### Install udev-rules
- You can find the rules under the folder `resources` with the name `99-spmb.rules`
- Easiest way to identify the device node is to unplug all usb devices from your work station except the Arduino UNO. Then before connecting the Arduino you can list the devices with `ls /dev` and then after connecting the Arduino entering the command `ls /dev` should show a new device node entry. (Usually `/dev/ttyACMx` where x is some number relating to the amount of currently registered ttyACM devices)

- Assuming that your Arduino identifies with `/dev/ttyACM0` you can list the hardware information with the command

```
udevadm info -ap /sys/class/tty/ttyACM0
```

- From there extract the serial attribute with 

```
udevadm info -ap /sys/class/tty/ttyACM0 | grep "^    ATTRS{serial}=="
```

which should show something similar to 

```
$ udevadm info -ap /sys/class/tty/ttyACM0 | grep "^    ATTRS{serial}=="
    ATTRS{serial}=="85430353331351E0D120"
    ATTRS{serial}=="0000:00:14.0"
```

- Open the file `99-spmb.rules` under `resources/` and replace the default entry for serial `ATTRS{serial}=="55736313238351219281"` with your terminal output e.g. `ATTRS{serial}=="85430353331351E0D120"`

- Copy the file into the udev folder using super user privileges

```
sudo cp resources/99-spmb.rules /etc/udev/rules.d/
```

- Reload and trigger the rules using

 ```
 udevadm control --reload-rules && udevadm trigger
 ```
 
*If `ls /dev` doesn't show the entry* **dev/spmb** *try rebooting your work station and if the entry still doesn't show reiterate throught the instructions to make sure you followed them correctly.*

***

## Contribution

Pull requests from forked repositories and opening new issues are very welcome and much appreciated.

Contributions are also invited to adhere to the [C++ Google Style Guide](https://google.github.io/styleguide/cppguide.html).

**Maintainer:** Philipp Rothenhäusler (*phirot a t kth.se | philipp.rothenhaeusler a t gmail.com*)

***

## LICENSE
BSD 3-Clause License

Copyright (c) 2019, Philipp Rothenhäusler
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.