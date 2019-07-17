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

namespace SPMB {
    //! Collection of all register necessary for interfacing with an interrupt pin.
    class InterruptInput{ 
        public:
            std::string         mLabel;                     //!< Label identifier for searching interrupt with interrupt manager
            volatile uint8_t    mBit;                       //!< Bit number in Register
            volatile uint8_t *  mDataRegister;              //!< Data register for corresponding interrupt and providing the pin value
            volatile uint8_t *  mDirectionRegister;         //!< Direction register defining the pin as either an input (0) or output (1)
            volatile uint8_t *  mCfgRegister;               //!< Configuration register providing the option to activate pull-ups (1) and no pull-ups (0)
            volatile uint8_t *  mInterruptRegister;         //!< Interrupt activation register, enables the Bit's interrupt.

            volatile long       mTimer;                     //!< Timestamp variable for last interrupt activation
            volatile uint16_t   mTimePeriodValid;           //!< Intermediary variable for always storing only accepted time periods
            volatile uint16_t   mTimePeriod;                //!< Temporary and volatile time period variable that can contain erroneous reads
            volatile uint16_t   mTimePeriodDefault;         //!< Default time period as substitution of erroneous reads as output in case of errors 

            volatile bool       mStatus;                    //!< Status boolean, showing good (true) or bad status (false) of interrupt
            volatile bool       mReceivedHigh;              //!< Indicator that a high pin level has been read. Only possible after low read
            volatile bool       mReceivedLow;               //!< Indicator that a low value has been read.
            volatile bool       mNewInterruptAvailable;     //!< Indicator that a complete cycle has been observed and that pin is disarmed.

            volatile uint8_t    mErrorCount;                //!< Statistic about both the negative errors (when to disable with mStatus false) as well as the amount the interrupt has been ignored in rotation and when it is time to turn back on (mStatus true).

            InterruptInput();                               //!< Default Constructor

            //! Parse register arguments to member variables   
            void initialise(std::string labelInput_,
                            uint16_t DefaultTimePeriod_,
                            uint8_t BitInput_,
                            volatile    uint8_t * DirectionRegister_,
                            volatile    uint8_t * CfgRegister_,
                            volatile    uint8_t * DataRegister_,
                            volatile    uint8_t * InterruptRegister_);      
            
            //! Takes care of setting up all the pins for input definitions and pull-up configuration of respective pin.   
            void configure();
            
            //! Disarms the Pin for Pin Change Interrupts.   
            void disarm_interrupt();

            //! Arms the Pin for Pin Change Interrupts   
            void arm_interrupt();

            //! Handles complete interrupt logic (is called by InterruptManager)   
            void update_timer();

            //! Checks whether to reenable or to disable a interrupt with a certain error trigger count   
            void switch_error_status();

            //! Reset Interrupt  
            void reset();
    };

    //! Collects Interrupts with same properties in groups and 
    class InterruptGroup{
        public:
            //! Constructor
            InterruptGroup();

            //! Index for current iterated interrupt \Todo replace with iterators.
            volatile uint8_t mIdx;

            //! Interrupt pountires
            std::vector<InterruptInput*> mInterrupts;
            
            //! Pointer to current interrupt
            InterruptInput* mInterruptActive;

            //! Append new interrupt to vector   
            void append_interrupt(InterruptInput * NewInterruptInput);

            //! Update timer for each interrupt   
            void update_timer();

            //! Attempt to rotate and search for a good status in one of the interrupts and update skipped interrupts with bad status flag.   
            void _attempt_rotating();
            
            //! Rotate active interrupt to next interrupt in interrupt group with good status flag and update error flags.   
            void rotate_interrupts();
    };

    //! Collection of Interrupt Groups (e.g. sorted as time sensitive and non-time sensitive ).   
    class InterruptManager{
        public:
            //!< Collection of all interrupt groups storing their pointers.
            std::vector<InterruptGroup*> mInterruptGroups;             

            //! Append a new interrupt group to the interrupt manager   
            void append_group(InterruptGroup * newInterruptGroup);
            
            //! Rotate all interrupts or attempt to rotate in case some are idle or in an erroneous state   
            void rotate_interrupts();

            //! Arm all interrupts.   
            void arm_interrupts();

            //! Get time period for a interrupt identified with the provided label argument and store result in time_period argument. 
            bool get_time_period(std::string label, volatile uint16_t &time_period);
    };
}