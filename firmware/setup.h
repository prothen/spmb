#pragma once

#include "spmb.h"

namespace SPMB{
    class SetupManager{
        public:
            InterruptManager * mInterruptManager;

            void _common();
            void _configure_interrupts();
            void configure(InterruptManager* interrupt_manager_in);
            void delay_start(float seconds_to_wait);
            SetupManager();

    };
}