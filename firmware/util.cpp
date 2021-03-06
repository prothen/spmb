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

#include "spmb.h"


namespace SPMB{

    util::control_filtered::control_filtered():
        steering(LP_T_STEERING, DEFAULT_PWM_STEERING),
        velocity(LP_T_VELOCITY, DEFAULT_PWM_VELOCITY),
        transmission(LP_T_TRANSMISSION, DEFAULT_PWM_TRANSMISSION),
        differential_front(LP_T_DIFFERENTIAL_FRONT, DEFAULT_PWM_DIFFERENTIAL_FRONT),
        differential_rear(LP_T_DIFFERENTIAL_REAR, DEFAULT_PWM_DIFFERENTIAL_REAR)
    {;}

    void util::clear_screen() {
    #if DIAGNOSIS
    for (int i = 0;i<15;i++){
        Serial.println(""); 
    }
    #endif /* DIAGNOSIS */
    } 

    void util::print(std::string text_in, bool new_line) {
    #if DIAGNOSIS
        const char * text = text_in.c_str();
        Serial.print(text);
        if (new_line){
            Serial.println("");
        }
        else{;}
    #endif /* DIAGNOSIS */
    }       

    void util::print(const char* text, bool new_line) {
    #if DIAGNOSIS
        Serial.print(text);
        if (new_line){
            Serial.println("");
        }
        else{;}
    #endif /* DIAGNOSIS */
    }

    void util::print(int number, bool new_line) {
    #if DIAGNOSIS
        Serial.print(number);
        if (new_line){
            Serial.println("");
        }
        else{;}
    #endif /* DIAGNOSIS */
    }    

    void util::print(volatile uint16_t number, bool new_line) {
    #if DIAGNOSIS
        Serial.print(number);
        if (new_line){
            Serial.println("");
        }
        else{;}
    #endif /* DIAGNOSIS */
    }

    void util::print(long number, bool new_line) {
    #if DIAGNOSIS
        Serial.print(number);
        if (new_line){
            Serial.println("");
        }
        else{;}
    #endif /* DIAGNOSIS */
    }

    void util::print(float number, bool new_line) {
    #if DIAGNOSIS
        Serial.print(number);
        if (new_line){
            Serial.println("");
        }
        else{;}
    #endif /* DIAGNOSIS */
    }

    void util::print_binary(uint8_t binary_number) {
    #if DIAGNOSIS
    for (uint8_t k = 0x80; k; k >>= 1) {
        if (binary_number & k) {
        Serial.print("1");
        }
        else {
        Serial.print("0");
        }
    }
    Serial.println("");
    #endif /* DIAGNOSIS */
    }

    void util::toggle(uint8_t reg, uint8_t port){
            static boolean state = false;
            if (state){
                    port &= (~reg);
            }
            else{
                    port |= reg;
            }
    }

    void util::correct_period(volatile uint16_t period, volatile uint16_t &period_corrected){
        if (PWM_LOW <= period  && period <= PWM_HIGH){
            period_corrected = period;
        }
        else if (period < PWM_LOW){
            period_corrected = PWM_LOW;
        }
        else{
            period_corrected = PWM_HIGH;
        }
    }

    int8_t util::period_to_pwm(volatile uint16_t input){
            return int8_t( ( float((input-1500.f))/500.f ) * 100.f ); // \Todo: Use Macro identifiers from setup_macro.h 
    }

    uint16_t util::pwm_to_period(volatile int8_t input){
            return uint16_t( (input/100.0)*float(PWM_NEUTRAL-PWM_LOW) + PWM_NEUTRAL );
    }

    boolean util::IS_VALID(volatile uint16_t &period) {
        if ((PWM_LOW_TH <= period) && (period <= PWM_HIGH_TH)) {
            return true;
        }
        else{ 
            return false;
        }
    }

    boolean util::HAS_CLEAR_STATE(volatile uint16_t period){
        if ((period < PWM_CLEAR_TH_LOW) || (PWM_CLEAR_TH_HIGH < period)){
            return true;
        }
        else{
            return false;
        }
    }
    boolean util::STATE(volatile uint16_t period){
        if (period <= PWM_CLEAR_TH_LOW){
            return false;
        }
        else{
            return true;
        }
    }

    boolean util::IS_IDLE(volatile long stamp_in_us, uint16_t period_in_ms){
        if ((micros() - stamp_in_us) > (ms2us * period_in_ms) ){
            return true;
        }
        else{
            return false;
        }
    }

    boolean util::IS_TIME(long &timestamp_in_us, uint16_t time_period_in_ms) {
        if ((micros() - timestamp_in_us) >= (ms2us * time_period_in_ms)){
            timestamp_in_us = micros();
            return true;
        }
        else{
            return false;
        }
    }    

    boolean util::IS_TIME_IN_MS(long &timestamp_in_ms, uint16_t time_period_in_ms) {
        if ((millis() - timestamp_in_ms) >= long(time_period_in_ms)){
            timestamp_in_ms = millis();
            return true;
        }
        else{
            return false;
        }
    }
}
