#pragma once

#include "spmb.h"

namespace SPMB{
    
    template <typename T>
    class LowPass{	
        public:
            uint16_t mTimeConstant;
            long mTimestamp;
            T mValue;

            /*
            */
            void _correct_period(volatile uint16_t &period_corrected);

            void update_state (T NewState);
            T value();
            LowPass(float time_constant_in_seconds, T default_value);
    };
    template <typename T>
    void LowPass<T>::_correct_period(volatile uint16_t &period){
        if (PWM_LOW <= period  && period <= PWM_HIGH){
            ;
        }
        else if (period < PWM_LOW){
            period = PWM_LOW;
        }
        else{
            period = PWM_HIGH;
        }
    } 

    template <typename T>
    T LowPass<T>::value(){
        return mValue;
    }

    template <typename T>
    LowPass<T>::LowPass(float time_constant_in_seconds, T default_value){
        mTimeConstant = uint16_t(s2ms * time_constant_in_seconds);
        mTimestamp = micros();
        mValue = default_value;
    }

    template <typename T>
    void LowPass<T>::update_state (T StateNew){
        double dt = us2ms*(micros() - mTimestamp);
        double dn = (mTimeConstant) + dt; 
        mValue = T(((mTimeConstant)/dn) * (mValue) + (dt/dn) * (StateNew));
        _correct_period(mValue);
        mTimestamp = micros();
    }
    
}