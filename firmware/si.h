#pragma once

#include "spmb.h"

namespace SPMB {

    /*! Provides capability to extend output to various output pins depending on current status. */
    struct StatusOutput{
        uint8_t status;                               //!< status for this output to be active.
        uint8_t pin;                                //!< Pin number for output signaling.
        uint16_t period;                            //!< Period in ms.
    };
  
    /*! Status indicator switching status LEDs to visualise internal states and information. */
    class StatusIndicator{
        public: 
            typedef enum {
                OFF, 
                RC_ON, 
                SW_ON, 
                EMERGENCY
            } SIStatus;                                                 //!< Enumeration of all possible modes
            
            SIStatus mStatus;                                           //!< Current status of status indicator
            std::vector<StatusOutput> mOutput;                            //!< Reference to output indicator
            StatusOutput* mActiveOutput;
            long mTimestamp;                                            //!< Timestamp of last update of LED status. 

            StatusIndicator();                                          //!< Add pins as outputs
            void configure();                                           //!< Configure pins as outputs
            void switch_status(SIStatus new_status);                      //!< Change mActiveOutput pointer
            void blink();                                               //!< Toggle the LED status according to the current output
    };

    
} // namespace SPMB