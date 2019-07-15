# SPMB
## Brief
The Signal&Power Management Board (**SPMB**) provides firmware for Arduino UNO devices using Atmega328p microcontrollers facilitating a Robot Operating System (**ROS**) interface for a radio controlled (**RC**) supervised vehicle. Here the current configuration is based on the **TRAXXAS** platforms (in specific TRX4). The SPMB provides direct RC forwarding and publishing of actuated control signals to the ROS interface.  With a predefined fast switching sequence of channels the software based control algorithms are activated. Furthermore a failsafe for idled software connections is embedded such that RC operation is activated if less than *10Hz* control frequency is present. If a RC channel is not read successfully, e.g. broken wires or empty batteries, the channel is deactivated and the output goes in the channels default control signal. In case of velocity and steering this corresponds to 0 actuation.
In addition lowpass filtering of all control signals, both remote and software based is included and can be customised by adapting the parameters under `firmware/setup_macros.h`. The main and control loop time is configured and tested at *50Hz*. Theoretically much higher frequencies are possible, however interrupt reading capability greatly deteriorates with a fast main loop frequency. Therefore it is recommended to not vary these frequencies to higher values or otherwise stable performance can not be guaranteed.
Also the firmware provides a led signaling class which provides easily accessible information about the current SPMB's state machine status.  

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


## Dependencies
**Install Libraries into `~/Arduino/libraries`:**

Go to `cd ~/Arduino/libraries`

- `git clone https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library.git`

- `git clone https://github.com/maniacbug/StandardCplusplus.git`

- Download and install Arduino (v1.8.5) (x64 https://www.arduino.cc/download_handler.php?f=/arduino-1.8.5-linux64.tar.xz)

- Build the repository to generate header files for messages with `catkin build` in any directory of your catkin workspace

- Source your workspace with `source ~/your_workspace/devel/setup.bash`

- Run the `make_library.sh` script in the root of this repository

- If desired install the udev rule file under `resources/99-spmb.rules`, which will install the permanent device node `dev/spmb` to avoid the deviating node identifier ttyACMx with x changing based on the amount of ttyACM devices or reconnections of the same device) 

## Execute the ROS Interface

- Run `roslaunch spmbv2 run.launch`

(*If any configuration parameters regarding baudrate have been adjusted then parse the corresponding parameter in the launch file e.g. `roslaunch spmbv2 run.launch baud:=yourbaudrateinteger`*)

- After approximately 3-4 seconds the terminal will output 

```
process[serial_node-1]: started with pid [10506]
[INFO] [1563182250.327048]: ROS Serial Python Node
[INFO] [1563182250.351442]: Connecting to /dev/spmb at 57600 baud
[ERROR] [1563182267.573225]: Unable to sync with device; possible link problem or link software version mismatch such as hydro rosserial_python with groovy Arduino
[INFO] [1563182267.619900]: Note: publish buffer size is 100 bytes
[INFO] [1563182267.620545]: Setup publisher on actuated [spmbv2/actuated]
[INFO] [1563182267.636111]: Note: subscribe buffer size is 100 bytes
[INFO] [1563182267.636602]: Setup subscriber on request [spmbv2/request]
```

## Interface with ROS
- After installing the udev rules (see below) the ROS Interface can be accessed with 
```rosrun rosserial_python serial_node.py _port:=/dev/spmb _baud:=57600```

- The `rosrun` command is conveniently embedded into the launch file under `launch/run.launch` (see *Execute the ROS Interface* section above)

## Install udev-rules
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
    ATTRS{serial}=="**85430353331351E0D120**"
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
 
*(if `ls /dev` doesn't show the entry **/dev/spmb** try rebooting your work station and if the entry still doesn`t show reiterate throught the instructions to make sure you followed them correctly)*