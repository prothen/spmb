#pragma once

#include "spmb.h"

#include <Adafruit_PWMServoDriver.h>

namespace SPMB{

    class OutputDriverI2C{
        public:    
            double mUsBit;
            double mOffsetDriver;        
            Adafruit_PWMServoDriver mDriver;

            void configure();
            void actuate(util::control_filtered* output);
    };
}