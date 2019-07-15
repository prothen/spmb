# SPMB

## Dependencies
**Install Libraries into `~/Arduino/libraries`:**

Go to `cd ~/Arduino/libraries`

- `git clone https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library.git`

- `git clone https://github.com/maniacbug/StandardCplusplus.git`

- Download and install Arduino (v1.8.5)

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

## Notes
- Main loop should be limited with a delay to avoid running in a unlimited loop (interrupts won't work otherwise)
- ```https://github.com/maniacbug/StandardCplusplus.git``` (use 1.8.5 arduino ide compiler for this library!!!)

## Interface with ROS
- After installing the udev rules (see below) the ROS Interface can be accessed with 
```rosrun rosserial_python serial_node.py _port:=/dev/spmb _baud:=115200```

## UDEV RULES
- show hardware info for component, e.g. ttyACM0 ``` udevadm info /dev/ttyACM0 ``` and ```udevadm info -a -p /sys/class/tty/ttyACM0 ```
- reload and trigger rules ```udevadm control --reload-rules && udevadm trigger```
- connecting several usb devices to embedded devices can cause non-static assignment of device ports. This likely causes incompatibility once a fixed port is chosen. Hence it is recommended to install .rules that assign a unique identifier once a know device is reconnected. You can find such a rule that can be copied in the corresponding folder below.