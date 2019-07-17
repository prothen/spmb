/*
BSD 3-Clause License

Copyright (c) 2019, Philipp Rothenhäusler
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include "spmb.h"

namespace SPMB {
    //! Provides interface and actuation wrapper for status indicator output pins.
    /*! Status indicator switching status LEDs to visualise internal states and information. */
    class StatusIndicator{
        public:     
            //! Enumeration of all possible modes
            typedef enum {
                OFF, 
                RC_ON, 
                SW_ON, 
                EMERGENCY
            } SIStatus;                                                 
            //! Collection of output references and status assignment.
            /*! Provides capability to extend output to various output pins depending on desired status. */
            struct StatusOutput{
                SIStatus Status = SIStatus::OFF;                            //!< status for this output to be active.
                uint8_t Pin = 0;                                            //!< Port to identify output register
                uint16_t Period = 1500;                                     //!< Period in ms.
                // \Todo use buzzer pointer instead of Pin and rename to OutputLocation
                StatusOutput(SIStatus, uint8_t, uint16_t);
            };

            //! Indicates the current status
            SIStatus mStatus;                                           

            //! Reference to output indicator                                              
            std::vector<StatusOutput> mOutput;                          
            
            //! Pointer to current active status output.
            StatusOutput* mActiveOutput;

            //! Stores the current value of the indication output device \Todo might to be included in output struct.
            uint8_t mValue;
            
            //! Timestamp of last update of LED status. 
            long mTimestamp;                                            

            //! Add pins as outputs.
            StatusIndicator(); 

            //! Set desired output pins and configure member variables.                                         
            void configure();

            //! Change mActiveOutput pointer.                                           
            void switch_status(SIStatus new_status);                   

            //! Toggle the LED status according to the current output.
            void blink();                                               
    };    
} // namespace SPMB