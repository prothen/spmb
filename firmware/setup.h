#pragma once

#include "spmb.h"

namespace SPMB{
    class SetupManager{
        public:
            InterruptManager * mInterruptManager;
            OutputDriverI2C * mOutput;
            void _common();
            void configure_common();

            void _configure_interrupts();
            void configure_interrupts(InterruptManager* interrupt_manager_in);

            void configure_output(OutputDriverI2C* output_in);
            void delay_start(float seconds_to_wait);
            SetupManager();

    };
}