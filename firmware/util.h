#pragma once

#include "spmb.h"

namespace SPMB{

    //! Namespace collecting all utility functions and structs.
    namespace util{
        
         //! Struct summarising control outputs  
        struct control {
            uint16_t steering = 1500;                               //!< Steering control
            uint16_t velocity = 1500;                               //!< Velocity control
            uint16_t transmission = 1000;                           //!< Transmission control
            uint16_t differential_front = 2000;                     //!< Differential Front control
            uint16_t differential_rear = 2000;                      //!< Differential Rear control
        };

         //! Struct summarising filtered control outputs  
        struct control_filtered {
                LowPass<uint16_t> steering;                         //!< Steering filtered output control
                LowPass<uint16_t> velocity;                         //!< Velocity filtered output control
                LowPass<uint16_t> transmission;                     //!< Transmission filtered output control
                LowPass<uint16_t> differential_front;               //!< Differential filtered Front output control
                LowPass<uint16_t> differential_rear;                //!< Differential filtered Rear output control
                control_filtered();                                 //!< Constructor filtered output control
        };

         //! Clear Screen  
        void clear_screen();

         //! Print binary  
        void print_binary(uint8_t binary_number);
        
         //! Print std::string  
        void print(std::string text_in, bool new_line);

         //!  Print text  
        void print(const char* text, bool new_line);

         //! Print number int  
        void print(int number, bool new_line);

         //! Print number int  
        void print(uint16_t number, bool new_line);

         //! Print number long  
        void print(long number, bool new_line);
        
         //! Print number float  
        void print(float number, bool new_line);

         //! Alarm - easy pin toggle Interrupt  
        void toggle(uint8_t reg, uint8_t port);
        
         //! Correct periods   
        void correct_period(volatile uint16_t period, volatile uint16_t &period_corrected);

         //! Transate period to pwm  
        int8_t period_to_pwm(volatile uint16_t input);

         //! Translate pwm to period  
        uint16_t pwm_to_period(volatile int8_t input);

         //! saturate read pwm to keep period in range [1,2] ms and frequency 50 Hs with T=20ms  
        boolean IS_VALID(volatile uint16_t &period);

         //! saturate read pwm to keep period in range [1,2] ms and frequency 50 Hs with T=20ms  
        void IS_VALID(volatile uint16_t &period, volatile boolean &valid);

         //! Return true if state has a distinct state and is not noised.  
        boolean HAS_CLEAR_STATE(volatile uint16_t period);
        
         //! Return the value assigned to certain periods from setup_macro.h  
        boolean STATE(volatile uint16_t period);

         //! Return true if the stamp is obsolete with regard to provided period in ms.  
        boolean IS_IDLE(volatile long stamp_in_us, uint16_t period_in_ms);

         //! Update timestamp if time period is exceeded and return true.  
        boolean IS_TIME(long &timestamp_in_us, uint16_t time_period_in_ms);
        
         //! Update timestamp if time period is exceeded and return true.  
        boolean IS_TIME_IN_MS(long &timestamp_in_ms, uint16_t time_period_in_ms);
    }
}