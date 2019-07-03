#pragma once

#include "spmb.h"

/* Custom Interrupt Class */
namespace SPMB {
    #define ms2us 1000.
    class InterruptInput{ 
        public:
            std::string mLabel;
            volatile uint8_t mBit;
            volatile uint8_t * mDataRegister; 
            volatile uint8_t * mDirectionRegister; 
            volatile uint8_t * mCfgRegister; 
            volatile uint8_t * mInterruptRegister;

            volatile long mTimer;
            volatile uint16_t mTimePeriod;

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
                Reset Interrupt 
            */
            void reset();
    };

    /*
    Collects Interrupts with same properties
    */
    class InterruptGroup{
        public:
            std::vector<InterruptInput*> mInterrupts;
            std::vector<InterruptInput*>::iterator mInterruptActive;
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

            volatile uint16_t get_time_period(std::string label);
    };
}

/*
// Port register, shows value
// Direction register high (output) or low (input)
// Configuration register input: high (pull_up) or low (no pull_up)
            // output: high(writes high) low (writes low)
            // writing to mDATARegister (PINxn) toggles mCfgRegister (PORTxn)

*/