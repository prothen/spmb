#pragma once

#include "spmb.h"

namespace SPMB{
    class SetupManager{
        public:
            InterruptManager * mInterruptManager;
            void _common();
            void _configure_interrupts();
            void configure();
            void delay_start(int);
            SetupManager();
    };
}