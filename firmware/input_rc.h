#pragma once

#include "spmb.h"

/* Custom Interrupt Class */
namespace SPMB {
    class InterruptInput{ 
        public:
            std::string mLabel;
            volatile uint8_t mBit;
            volatile uint8_t * mDataRegister; 
            volatile uint8_t * mDirectionRegister; 
            volatile uint8_t * mCfgRegister; 
            volatile uint8_t * mInterruptRegister;

            volatile long mTimer;
            volatile uint16_t mTimePeriodValid;
            volatile uint16_t mTimePeriod;
            volatile uint16_t mTimePeriodDefault;

            volatile bool mStatus;
            volatile bool mReceivedHigh;
            volatile bool mReceivedLow;
            volatile bool mNewInterruptAvailable;

            volatile uint8_t mErrorCount;

            InterruptInput();   

            /*
                Parse register arguments to member variables
            */
            void initialise(std::string labelInput_,
                            uint16_t DefaultTimePeriod_,
                            uint8_t BitInput_,
                            volatile    uint8_t * DirectionRegister_,
                            volatile    uint8_t * CfgRegister_,
                            volatile    uint8_t * DataRegister_,
                            volatile    uint8_t * InterruptRegister_);      
            
            /*
                Takes care of setting up all the pins for input definitions and pull-up configuration of respective pin
            */
            void configure();
            
            /*
            Disarms the Pin for Pin Change Interrupts
            */
            void disarm_interrupt();

            /*
            Arms the Pin for Pin Change Interrupts
            */
            void arm_interrupt();

            /*
            Handles complete interrupt logic (is called by InterruptManager)
            */
            void update_timer();

            /*
                Checks whether to reenable or to disable a interrupt with a certain error trigger count
            */
            void switch_error_status();

            /* 
                Reset Interrupt 
            */
            void reset();
    };

    /*
    Collects Interrupts with same properties
    */
    class InterruptGroup{
        public:
            InterruptGroup();
            volatile uint8_t mIdx;
            std::vector<InterruptInput*> mInterrupts;
            InterruptInput* mInterruptActive;
            void append_interrupt(InterruptInput * NewInterruptInput);
            void update_timer();
            void rotate_interrupts();
    };

    /*
    Collection of Interrupt Groups (e.g. sorted as time sensitive and non-time sensitive )
    */
    class InterruptManager{
        public:
            std::vector<InterruptGroup*> mInterruptGroups;
            void append_group(InterruptGroup * newInterruptGroup);
            void rotate_interrupts();
            void arm_interrupts();

            bool get_time_period(std::string label, volatile uint16_t &time_period);
    };
}