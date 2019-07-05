#pragma once

#include <stdint.h>
#include <avr/interrupt.h>

#include <Arduino.h>
#include <StandardCplusplus.h>
#include <vector>
#include <string>

#include "setup_macros.h"
#include "lowpass.h"
#include "util.h"

#ifdef ROS_ACTIVE
    #ifndef ROS_LOADED
    #define ROS_LOADED
            #include <ros.h>
            #include <spmbv2/request.h>
            #include <spmbv2/actuated.h>
            #include "ros_interface.h"
    #endif /* ROS_LOADED */
#endif /* ROS_ACTIVE */

#include "input_rc.h"
#include "output_i2c.h"
#include "sm.h"
#include "setup.h"

