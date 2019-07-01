#pragma once

#include "spmb.h"

namespace SMPB{
    typedef struct element {
        float value;
        float time_period;
        float timestamp;
    };

    #define MS_SCALE 1000000.
    class LowPass{	
        public:
            std::vector<element> mStates;
            
            void filter (float *);

            LowPass(uint8_t, float * time_constant);
    };
}