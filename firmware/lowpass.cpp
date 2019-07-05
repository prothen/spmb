#include "spmb.h"

namespace SPMB{
    template <typename T>
    LowPass<T>::LowPass(uint8_t instances, float time_constant){
        std::vector<element<T>> tmp_new_states;
        for(uint8_t i = 0; i < instances; i++){
            element<T> * tmp_element = new element<T>;
            tmp_element->value = 0;
            tmp_element->time_period = s2ms * time_constant;
            tmp_element->timestamp = micros();
            tmp_new_states.push_back(*tmp_element);
        }
        mStates = tmp_new_states;
    }

    template <typename T>
    void LowPass<T>::filter (T * StateNew){
        uint8_t i = 0;
        for (auto state = mStates.begin(); state != mStates.end(); state++){
            float dt = us2ms * (micros() - state->timestamp);
            float dn = (state->time_period) + dt; 
            state->value = (state->time_period/dn) * state->value + (dt/dn) * StateNew[i];
            state->timestamp = micros();
            StateNew[i] = state->value;
            i++;
        }
    }
}