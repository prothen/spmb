#pragma once

#include "spmb.h"

#include <Adafruit_PWMServoDriver.h>

namespace SPMB{
    
    /*! Output driver using I2C via PCA9688 */
    class OutputDriverI2C{
        public:    
            double      mUsBit;                         //!< Amount of microseconds per bit of 4096 bits total resolution
            double      mOffsetDriver;                  //! Compensation of output driver period deviation from input in microseconds        
            Adafruit_PWMServoDriver mDriver;            //! Adafruit driver to interface with timer chip

            /*! configure pins and frequencies */
            void configure();

            /*! send periods to be actuated */
            void actuate(util::control_filtered* output);
    };
}