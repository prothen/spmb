/*
 Copyright (c) 2019, Philipp Rothenhäusler. All rights reserved.

 Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name of Philipp Rothenhäusler nor the names of its contributors may
   be used to endorse or promote products derived from this software without
   specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include "spmb.h"

namespace SPMB{
    
    //! Provides a templated lowpass filter class for scalar elements.
    template <typename T>
    class LowPass{	
        public: 
            uint16_t    mTimeConstant;      //!< filter constant 
            long        mTimestamp;         //!< timestamp since last update
            T           mValue;             //!< most recent value after last filter update
            
            //! Correct period  
            void _correct_period(volatile uint16_t &period_corrected);

            //! Update State  
            void update_state (T NewState);

            //! Constructor  
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