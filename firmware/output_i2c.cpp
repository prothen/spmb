#include "spmb.h"

namespace SPMB{

    void OutputDriverI2C::configure(){
        double freq = 108.;
        double resolution = 4096.;
        double perus = 1000000. / freq; //us per period
        mUsBit = perus / resolution; // us per bit
        mOffsetDriver = 106.; // 106us offset in adafruit driver

        mDriver.begin();
        mDriver.setPWMFreq(freq);
        delay(10);
    }

    void OutputDriverI2C::actuate(util::control_filtered * output){   
        mDriver.setPWM(M_OUT_STEERING, 0, uint16_t((output->steering.value() - mOffsetDriver) / mUsBit));
        mDriver.setPWM(M_OUT_VELOCITY, 0, uint16_t((output->velocity.value() - mOffsetDriver) / mUsBit));
        mDriver.setPWM(M_OUT_TRANSMISSION, 0, uint16_t((output->transmission.value() - mOffsetDriver) / mUsBit));
        mDriver.setPWM(M_OUT_DIFFERENTIAL_FRONT, 0, uint16_t((output->differential_front.value() - mOffsetDriver) / mUsBit));
        mDriver.setPWM(M_OUT_DIFFERENTIAL_REAR, 0, uint16_t((output->differential_rear.value() - mOffsetDriver) / mUsBit));
    }
}