#include "spmb.h"


namespace SPMB{
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
    Transate period to pwm
    */
    int8_t util::period_to_pwm(volatile uint16_t input){
            return int8_t( ( float((input-PWM_NEUTRAL))/(PWM_NEUTRAL-PWM_LOW) ) * 100.0 );
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
}
