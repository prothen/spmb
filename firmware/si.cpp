#include "spmb.h"

namespace SPMB{
    
    StatusIndicator::StatusIndicator(){
        mStatus = OFF;
        mTimestamp = millis();

        // ADD OFF Status CONFIGURATION
        StatusOutput tmp;
        tmp.status = OFF;
        tmp.pin = 13;     
        tmp.period = 5000;
        mOutput.push_back(tmp);

        // ADD RC_ON status CONFIGURATION
        tmp.status = RC_ON;
        tmp.pin = 13; 
        tmp.period = 750;    
        mOutput.push_back(tmp);

        // ADD SW_ON status CONFIGURATION
        tmp.status = SW_ON;
        tmp.pin = 13;   
        tmp.period = 400;  
        mOutput.push_back(tmp);

        // ADD EMERGENCY status CONFIGURATION
        tmp.status = EMERGENCY;
        tmp.pin = 13;   
        tmp.period = 100;  
        mOutput.push_back(tmp);

        for (std::vector<StatusOutput>::iterator it = mOutput.begin(); it != mOutput.end(); it++){
            pinMode(it->pin, OUTPUT);
            digitalWrite(it->pin, LOW);
        }      
    }

    void StatusIndicator::switch_status(SIStatus new_status){
        for(std::vector<StatusOutput>::iterator it = mOutput.begin(); it!=mOutput.end(); it++){
            if (it->status == new_status){
                mActiveOutput = it;
                break;
            } else {;}
        }
    }

    void StatusIndicator::blink(){
        if (util::IS_TIME_IN_MS(mTimestamp, mActiveOutput->period)){
            uint8_t tmp = digitalRead(mActiveOutput->pin);
            digitalWrite(mActiveOutput->pin, !tmp);
        } else {;}
    }
} // namespace SPMB