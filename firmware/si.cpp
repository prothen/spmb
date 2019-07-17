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