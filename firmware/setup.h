#pragma once

#include "spmb.h"

//! Namespace collecting all SPMB classes and utilities used in firmware.ino.
namespace SPMB{
    
    //! Runs the configure script and pin setup for the main classes like StatusIndicator, InterruptManager and OutputDriverI2C
    class SetupManager{
        public:
            //! Pointer to status_indicator for initialisation
            StatusIndicator * mStatusIndicator;
            
            //! Pointer to interrupt_manager for initialisation
            InterruptManager * mInterruptManager;
            
            //! Pointer to output driver for initialisation
            OutputDriverI2C * mOutput;

            //! Delay start for initialisation
            void delay_start(float seconds_to_wait);

            //! Common setup script
            void _common();

            //! Configure more common setup tools            
            void configure_common();

            //! Configure status indicator
            void configure_status_indicator(StatusIndicator* status_indicator_in);

            //! Configure interrupts by looping through each that is present in interrupt manager.n
            void _configure_interrupts();

            //! Wrapper around configure interrupts
            void configure_interrupts(InterruptManager* interrupt_manager_in);

            //! Configure output driver
            void configure_output(OutputDriverI2C* output_in);

            //! Arm autonomous system.
            void arm_system();

            //! Pointer to status_indicator for initialisation
            SetupManager();

    };
}