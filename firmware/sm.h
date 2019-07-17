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

namespace SPMB{
    
    //! Executes main logic loop such as switching logic, reading inputs and actuating control signals to hardware interface.
    class StateMachine{
        public:
            //! Facilitates interface with hardware status indicators such as LEDs and piezo buzzers.
            StatusIndicator* mStatusIndicator;          

            //! Pointer to interrupt manager provides access to all interrupt results.
            InterruptManager* mInterruptManager;         

            #ifdef ROS_ACTIVE
            //! Pointer to ROS interface that provides access to all ROS related actions. 
            ROSInterface<myHardware>* mRosi;            
            #endif /* ROS_ACTIVE */

            //! Pointer to output driver that provides access to all hardware related actuation actions. 
            OutputDriverI2C* mOutput;                   

            //! Filtered control output for actuation via output driver.
            util::control_filtered mControlFiltered;    

            //! Indicates state machine health status. 
            bool mALIVE;                                
            
            //! Activation boolean for software control loop 
            bool mSW_CONTROL_ACTIVE;                    
            
            //! Timestamp of last main loop iteration.
            long mSMTimestamp; 

            //! Timestamp of last control actuation.                          
            long mCTRLTimestamp;                         

            //! Time period of main logic loop 
            uint16_t mSMTimePeriod;  

            //! Timp period of control loop                   
            uint16_t mCTRLTimePeriod;                   
            
            //! Define the system states.
            typedef enum {
                OFF,
                ARM,
                SWITCH
            } SWStates;                                 

            //! State of switching logic, see macros for states. 
            uint8_t mSWLState;                          
            
            //! Time period of switching logic to detect idle 
            uint16_t mSWLTimePeriod;                  
              
            //! Timestamp of switching logic 
            long mSWLTimestamp;                         
            
            //! Channel 4 (differential_front) state 
            boolean mS1;                                

            //! Channel 5 (differential_rear) state 
            boolean mS2;                                
            
            //! Previous Channel 4 (differential_front) state 
            boolean mS1_prev;                           

            //! Previous Channel 5 (differential_rear) state 
            boolean mS2_prev;

            //< Constructor                           
            StateMachine(); 

            //! Configure pointer to status indicator.
            void configure_status_indicator(StatusIndicator* status_indicator_in);

            #ifdef ROS_ACTIVE
            //! Configure pointer to interrupt manager and ros_interface.
            void configure_input( InterruptManager* interrupt_manager_in, ROSInterface<myHardware>* ros_interface_in);
            #else
            //! Configure pointer to interrupt manager.
            void configure_input( InterruptManager* interrupt_manager_in);
            #endif /* ROS_ACTIVE */
            
            //! Configure pointer to output driver.
            void configure_output(OutputDriverI2C* output_in);

            //! Update rc signal input.
            void _update_rc_signals(util::control &signals_rc);

            //! Execute switching logic to determine current operation mode.
            void _swl_execute_switching_logic(util::control* signals_rc);

            //! Verify whether switching idle condition is not violated and reset otherwise.
            void _swl_check_idle_transition();

            //! Delay according to remaining time until next desired loop.
            void wait_for_next_cycle();
            
            //! Process rc inputs.
            void _process_rc_inputs(util::control &signals_rc);
            
            #ifdef ROS_ACTIVE
            //! Process ROS inputs.
            void _process_ros_inputs(util::control &signals_ros);
            #endif /* ROS_ACTIVE */

            //! Update output signal into control_filtered.
            void update_output_signals();

            #ifdef ROS_ACTIVE
            //! Expose actuation signals to ROS via publishing.
            void _expose_actuated_signals_to_ros();
            #endif /* ROS_ACTIVE */
            
            //! Actutate output signals to hardware via output driver.
            void actuate();

            //! Indicate states and information to environment via hardware.
            void indicate_status();
    
            //! Execute logic in case of critical errors.
            void critical_error();  
    }; /* class StateMachine */
} // namespaces SPMB