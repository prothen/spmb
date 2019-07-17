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