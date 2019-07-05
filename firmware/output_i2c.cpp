#include "spmb.h"

namespace SPMB{
    OutputDriverI2C::OutputDriverI2C(util::control* signals_in){
        signals = signals_in;
    }

    void OutputDriverI2C::actuate(){;}
    void OutputDriverI2C::_convert_ppm_to_bits(){;}

}