#pragma once

#include "spmb.h"

namespace SPMB{

    namespace util{
        
        struct control { // switch to an enum
            uint16_t steering;
            uint16_t velocity;
            uint16_t transmission;
            uint16_t differential_front;
            uint16_t differential_rear;
        };

        struct control_filtered {
                LowPass<uint16_t> steering;
                LowPass<uint16_t> velocity;
                LowPass<uint16_t> transmission;
                LowPass<uint16_t> differential_front;
                LowPass<uint16_t> differential_rear;
                control_filtered();
        };
        /*
        Clear Screen
        */
        void clear_screen();

        /*
        Print binary
        */
        void print_binary(uint8_t binary_number);
        
        /* 
        Print std::string
        */
        void print(std::string text_in, bool new_line);

        /*
        Print text
        */
        void print(const char* text, bool new_line);

        /*
        Print number int
        */
        void print(int number, bool new_line);

        /*
        Print number int
        */
        void print(uint16_t number, bool new_line);

        /*
        Print number long
        */
        void print(long number, bool new_line);
        
        /*
        Print number float
        */
        void print(float number, bool new_line);

        /*
        * Alarm - easy pin toggle Interrupt - 4kHz //TODO: Write quick Macro
        */
        void toggle(uint8_t reg, uint8_t port);

        /*
        Adapting LED signaling state
        */
        void blink_led();

        /*
        */
        void correct_period(volatile uint16_t period, volatile uint16_t &period_corrected);

        /*
        Transate period to pwm
        */
        int8_t period_to_pwm(volatile uint16_t input);

        /*
        Translate pwm to period
        */
        uint16_t pwm_to_period(volatile int8_t input);

        /*
        saturate read pwm to keep period in range [1,2] ms and frequency 50 Hs with T=20ms
        */
        boolean IS_VALID(volatile uint16_t &period);

        /*
        saturate read pwm to keep period in range [1,2] ms and frequency 50 Hs with T=20ms
        */
        void IS_VALID(volatile uint16_t &period, volatile boolean &valid);

        /*
        */
        boolean HAS_CLEAR_STATE(volatile uint16_t period);
        
        /*
        */
        boolean STATE(volatile uint16_t period);

        /*
        */
        boolean IS_IDLE(volatile long stamp_in_us, uint16_t period_in_ms);

        /*
        */
        boolean IS_TIME(long &timestamp_in_us, uint16_t time_period_in_ms);
  
    }
}