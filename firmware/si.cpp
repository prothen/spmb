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

#include "spmb.h"

namespace SPMB{
    
    StatusIndicator::StatusOutput::StatusOutput(SIStatus status, uint8_t pin, uint16_t period){
        Status = status;
        Pin = pin;
        Period = period;
    }

    StatusIndicator::StatusIndicator(){
        mStatus = SIStatus::OFF;
        mValue = 0;
        mTimestamp = millis();   
    }

    void StatusIndicator::configure(){
        
        // ADD OFF Status CONFIGURATION
        StatusOutput tmp0(SIStatus::OFF, 13, 2000);
        mOutput.push_back(tmp0);

        // ADD RC_ON Status CONFIGURATION
        StatusOutput tmp1(SIStatus::RC_ON, 13, 500);  
        mOutput.push_back(tmp1);

        // ADD SW_ON Status CONFIGURATION
        StatusOutput tmp2(SIStatus::SW_ON, 13, 200);
        mOutput.push_back(tmp2);

        // ADD EMERGENCY Status CONFIGURATION
        StatusOutput tmp3(SIStatus::EMERGENCY, 13, 100);
        mOutput.push_back(tmp3);
        
        for (std::vector<StatusOutput>::iterator it = mOutput.begin(); it != mOutput.end(); it++){
            pinMode(it->Pin, OUTPUT);
        }  

        mActiveOutput = &mOutput.back();
        
    }

    void StatusIndicator::switch_status(SIStatus new_status){
        if (new_status != mStatus){  
            for(std::vector<StatusOutput>::iterator it = mOutput.begin(); it != mOutput.end(); it++){
                if (it->Status == new_status){
                    mStatus = new_status;
                    mActiveOutput = &*it;
                    break; // \Todo This condition only holds if exactly one output is for each status assigned (unique indicators).
                } else {;}
            }
        } else{;}
    }

    void StatusIndicator::blink(){
        if (util::IS_TIME_IN_MS(mTimestamp, (mActiveOutput->Period))){
            uint8_t pin = mActiveOutput->Pin;
            if (mValue) { 
                mValue = 0;
                digitalWrite(pin, 1);
            } else{
                mValue = 1;
                digitalWrite(pin, 0);
            }
        } else {;}            
    }
} // namespace SPMB