#include "spmb.h"

namespace SMPB{
    void LowPass::filter (float * StateNew){
        uint8_t i = 0;
        for (auto state = mStates.begin(); state != mStates.end(); state++){
            float dt = (micros() - (*state).timestamp);
            float dn = ((*state).time_period) + dt; 
            (*state).value = ((*state).time_period/dn) * (*state).value + (dt/dn) * StateNew[i];
            (*state).timestamp = micros();
            StateNew[i] = (*state).value;
            i++;
        }
    }

    LowPass::LowPass(uint8_t Instances, float * TimeConstant){
        std::vector<element> tmp_new_states;
        for(uint8_t i = 0; i < Instances; i++){
            element * tmp_element = new element;
            (*tmp_element).value = 0;
            (*tmp_element).time_period = TimeConstant[i];
            (*tmp_element).timestamp = micros();
            tmp_new_states.push_back(*tmp_element);
        }
        mStates = tmp_new_states;
    }
}