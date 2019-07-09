# SPMB

## Notes
- Main loop should be limited with a delay to avoid running in a unlimited loop (interrupts won't work otherwise)


## Interface with ROS
- After installing the udev rules (see below) the ROS Interface can be accessed with 
```rosrun rosserial_python serial_node.py dev:=/dev/spmb```

## UDEV RULES
- show hardware info for component, e.g. ttyACM0 ``` udevadm info /dev/ttyACM0 ``` and ```udevadm info -a -p /sys/class/tty/ttyACM0 ```
- reload and trigger rules ```udevadm control --reload-rules && udevadm trigger```
- connecting several usb devices to embedded devices can cause non-static assignment of device ports. This likely causes incompatibility once a fixed port is chosen. Hence it is recommended to install .rules that assign a unique identifier once a know device is reconnected. You can find such a rule that can be copied in the corresponding folder below.