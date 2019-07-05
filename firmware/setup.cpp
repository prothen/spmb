#include "spmb.h"

namespace SPMB{
    SetupManager::SetupManager(){
        #if DIAGNOSIS
            Serial.begin(BAUD_RATE);
        #endif /* DIAGNOSIS */
    }
    void SetupManager::_common(){
        // Configure Interrupt Pin change inputs
        // reset all configuration registers
        util::print("Setup: MCUCR - ", false);
        util::print_binary(MCUCR);

        DDRB &= 0x00;
        DDRC &= 0x00;
        DDRD &= 0x00;
        PORTB &= 0x00;
        PORTC &= 0x00;
        PORTD &= 0x00;
        
        /* Configure control register to use all three Pin Change Interrupt Vectors */
        PCICR |= 0x07;
        PCIFR = 0x00;

        /* Deactivate all Interrupts by Default */
        PCMSK0 = 0x00; // PCINT[7:0]
        PCMSK1 = 0x00; // PCINT[15:8]
        PCMSK2 = 0x00; // PCINT[23:16]
    }

    void SetupManager::configure(){
        cli();
        util::print("########################################", true);
        util::print("Setup: Starting configuration:", true);
        _common();
        _configure_interrupts();
        sei();
        
    }
    void SetupManager::delay_start(int seconds_to_wait){
        util::print("Setup: Idle ", false);
        util::print(seconds_to_wait, false);
        util::print(" seconds before starting setup...", true);

        unsigned long lseconds_to_wait = 1000 * seconds_to_wait;
        long t0 = millis();
        while (millis() - t0 < lseconds_to_wait){;}
    }
    void SetupManager::_configure_interrupts(){
        int i = 0;
        for(typename std::vector<InterruptGroup*>::iterator it = (*mInterruptManager).mInterruptGroups.begin(); it != (*mInterruptManager).mInterruptGroups.end(); ++it){
            util::print("########################################", true);
            util::print("Setup: Configure Interrupt Group Number:", false);
            util::print(i, true);
            i++;

            int j = 0;
            for(typename std::vector<InterruptInput*>::iterator iit = (**it).mInterrupts.begin(); iit != (**it).mInterrupts.end(); ++iit){
                util::print("Setup: Configure Interrupt Number:", false);
                util::print(j, true);
                j++;
                (**iit).configure();
                util::print("Configure complete for: ", false);
                util::print((**iit).mLabel, true);
            }
            util::print("Added ", false);
            util::print((**it).mInterrupts.size(),false);
            util::print(" Interrupts", true);
        }
    }
}