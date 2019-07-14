#include "spmb.h"

namespace SPMB{
    OutputDriverI2C::OutputDriverI2C(util::control* signals_in):
    driver()
    {
        signals = signals_in;
    }

    void OutputDriverI2C::actuate(){
/*         
        uint16_t off_tick = 0;
        uint8_t index = 0;
        //convert with separate function
        // car 1: 
        // car 2:
        // car 3: -32
        // car 4: -100
        // car 5: 
        // car 2:
        off_tick = uint16_t(PWM_EXTERNAL_MIN_TICK + ((ms-80-PWM_LOW)/(PWM_HIGH-PWM_LOW))*(PWM_EXTERNAL_MAX_TICK-PWM_EXTERNAL_MIN_TICK));
        
                //off_tick-=27;
        if (k==0){
                //Serial.print("External PWM "); Serial.print(k); Serial.print(" | tick"); Serial.println(off_tick);
                //off_tick-=27;
        }
        else{
        }
        index = k*2;
external_pwm.setPWM(index, 0 , off_tick);
 */
        
        ;}
    void OutputDriverI2C::_convert_ppm_to_bits(){;}

}