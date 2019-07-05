#pragma once

#include "spmb.h"
#include <Adafruit_PWMServoDriver.h>

namespace SPMB{
    
    #define I2C_ADDRESS 0x00
    #define I2C_BIT_RESOLUTION 0x00
    #define I2C_BIT_LOW 

    class OutputDriverI2C{
        public:
            OutputDriverI2C(util::control* signals);
            util::control* signals; // link to state machine outputs
            void actuate();
            void _convert_ppm_to_bits();
    };

}