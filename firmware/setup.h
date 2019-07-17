/*
 Copyright (c) 2019, Philipp Rothenhäusler. All rights reserved.

 Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name of Philipp Rothenhäusler nor the names of its contributors may
   be used to endorse or promote products derived from this software without
   specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

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