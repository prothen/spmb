/*
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
*/

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