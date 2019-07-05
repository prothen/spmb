#pragma once

#include "spmb.h"

namespace SPMB {
    class ROSInterface{
        public:
            util::control signals;
            void cb_request_(spmbv2::request data);
            // parse to control struct 
    };
}
