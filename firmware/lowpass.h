#pragma once

#include "spmb.h"

namespace SPMB{

    template <typename T>
    struct element{
        T value;
    } ;

    template <typename T>
    class LowPass{	
        public:
            uint16_t time_period;
            long timestamp;
            std::vector<element<T>> mStates;

            void filter (T *);

            LowPass(uint8_t, float time_constant);
    };

}