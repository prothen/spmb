#include "spmb.h"


namespace SPMB{

    util::control_filtered::control_filtered():
        steering(LP_T_STEERING, DEFAULT_PWM_STEERING),
        velocity(LP_T_VELOCITY, DEFAULT_PWM_VELOCITY),
        transmission(LP_T_TRANSMISSION, DEFAULT_PWM_TRANSMISSION),
        differential_front(LP_T_DIFFERENTIAL_FRONT, DEFAULT_PWM_DIFFERENTIAL_FRONT),
        differential_rear(LP_T_DIFFERENTIAL_REAR, DEFAULT_PWM_DIFFERENTIAL_REAR)
    {;}

    /* initialise
    util::status_led{
            uint8_t off;
            uint8_t idle;
            uint8_t sw_active;
            uint8_t rc_intervention; //switch out after xx time
        }
    */

    /*
    Clear Screen
    */
    void util::clear_screen() {
    #if DIAGNOSIS
    for (int i = 0;i<15;i++){
        Serial.println(""); 
    }
    #endif /* DIAGNOSIS */
    } 

    /*
        Print text
    */
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

    /*
        Print text
    */
    void util::print(const char* text, bool new_line) {
    #if DIAGNOSIS
        Serial.print(text);
        if (new_line){
            Serial.println("");
        }
        else{;}
    #endif /* DIAGNOSIS */
    }
    /*
        Print number int
    */
    void util::print(int number, bool new_line) {
    #if DIAGNOSIS
        Serial.print(number);
        if (new_line){
            Serial.println("");
        }
        else{;}
    #endif /* DIAGNOSIS */
    }    
    
    /*
        Print number int
    */
    void util::print(volatile uint16_t number, bool new_line) {
    #if DIAGNOSIS
        Serial.print(number);
        if (new_line){
            Serial.println("");
        }
        else{;}
    #endif /* DIAGNOSIS */
    }


    /*
        Print number long
    */
    void util::print(long number, bool new_line) {
    #if DIAGNOSIS
        Serial.print(number);
        if (new_line){
            Serial.println("");
        }
        else{;}
    #endif /* DIAGNOSIS */
    }
    /*
        Print number float
    */
    void util::print(float number, bool new_line) {
    #if DIAGNOSIS
        Serial.print(number);
        if (new_line){
            Serial.println("");
        }
        else{;}
    #endif /* DIAGNOSIS */
    }
    /*
    Print binary
    */
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

    /*
    * Alarm - easy pin toggle Interrupt - 4kHz //TODO: Write quick Macro
    */
    void util::toggle(uint8_t reg, uint8_t port){
            static boolean state = false;
            if (state){
                    port &= (~reg);
            }
            else{
                    port |= reg;
            }
    }


    /*
    Adapting LED signaling state
    */
    void util::blink_led() {
        static boolean STATUS = 0;
        static boolean ti_led = millis();
        if (( millis() - ti_led) > 1000 ) {
            STATUS = !STATUS;
            digitalWrite(LED_BUILTIN, STATUS);
            ti_led = millis();
        }
        else {;}
    }


    /*
    */
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

    /*
    Transate period to pwm
    */
    int8_t util::period_to_pwm(volatile uint16_t input){
            return int8_t( ( float((input-1500.f))/500.f ) * 100.f ); // TODO: Use Macro identifiers!
    }

    /*
    Translate pwm to period
    */
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

    /*
    */
    boolean util::IS_IDLE(volatile long stamp_in_us, uint16_t period_in_ms){
        if ((micros() - stamp_in_us) > (ms2us * period_in_ms) ){
            return true;
        }
        else{
            return false;
        }
    }

    /*
        is it time for the next loop? 
    */
    boolean util::IS_TIME(long &timestamp_in_us, uint16_t time_period_in_ms) {
        if ((micros() - timestamp_in_us) >= (ms2us * time_period_in_ms)){
            timestamp_in_us = micros();
            return true;
        }
        else{
            return false;
        }
    }
}
