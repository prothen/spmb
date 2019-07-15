#pragma once

#include "spmb.h"

/* Custom Interrupt Class */
namespace SPMB {
    class InterruptInput{ 
        public:
            std::string mLabel;                         //!< Label identifier for searching interrupt with interrupt manager
            volatile uint8_t mBit;                      //!< Bit number in Register
            volatile uint8_t * mDataRegister;           //!< Data register for corresponding interrupt and providing the pin value
            volatile uint8_t * mDirectionRegister;      //!< Direction register defining the pin as either an input (0) or output (1)
            volatile uint8_t * mCfgRegister;            //!< Configuration register providing the option to activate pull-ups (1) and no pull-ups (0)
            volatile uint8_t * mInterruptRegister;      //!< Interrupt activation register, enables the Bit's interrupt.

            volatile long mTimer;                       //!< Timestamp variable for last interrupt activation
            volatile uint16_t mTimePeriodValid;         //!< Intermediary variable for always storing only accepted time periods
            volatile uint16_t mTimePeriod;              //!< Temporary and volatile time period variable that can contain erroneous reads
            volatile uint16_t mTimePeriodDefault;       //!< Default time period as substitution of erroneous reads as output in case of errors 

            volatile bool mStatus;                      //!< Status boolean, showing good (true) or bad status (false) of interrupt
            volatile bool mReceivedHigh;                //!< Indicator that a high pin level has been read. Only possible after low read
            volatile bool mReceivedLow;                 //!< Indicator that a low value has been read.
            volatile bool mNewInterruptAvailable;       //!< Indicator that a complete cycle has been observed and that pin is disarmed.

            volatile uint8_t mErrorCount;               //!< Statistic about both the negative errors (when to disable with mStatus false) as well as the amount the interrupt has been ignored in rotation and when it is time to turn back on (mStatus true).

            InterruptInput();                           //!< Default Constructor

            /* Parse register arguments to member variables */
            void initialise(std::string labelInput_,
                            uint16_t DefaultTimePeriod_,
                            uint8_t BitInput_,
                            volatile    uint8_t * DirectionRegister_,
                            volatile    uint8_t * CfgRegister_,
                            volatile    uint8_t * DataRegister_,
                            volatile    uint8_t * InterruptRegister_);      
            
            /*! Takes care of setting up all the pins for input definitions and pull-up configuration of respective pin. */
            void configure();
            
            /*! Disarms the Pin for Pin Change Interrupts. */
            void disarm_interrupt();

            /* Arms the Pin for Pin Change Interrupts */
            void arm_interrupt();

            /*! Handles complete interrupt logic (is called by InterruptManager) */
            void update_timer();

            /*! Checks whether to reenable or to disable a interrupt with a certain error trigger count */
            void switch_error_status();

            /*! Reset Interrupt*/
            void reset();
    };

    /*! Collects Interrupts with same properties */
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

    /*! Collection of Interrupt Groups (e.g. sorted as time sensitive and non-time sensitive ). */
    class InterruptManager{
        public:
            std::vector<InterruptGroup*> mInterruptGroups;
            void append_group(InterruptGroup * newInterruptGroup);
            void rotate_interrupts();
            void arm_interrupts();

            bool get_time_period(std::string label, volatile uint16_t &time_period);
    };
}