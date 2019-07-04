#include "spmb.h"

using namespace SPMB;

InterruptManager interrupt_manager;
InterruptGroup B;
InterruptGroup C;    
InterruptGroup D; 
InterruptInput interrupt_steering;
InterruptInput interrupt_velocity;
InterruptInput interrupt_transmission;
InterruptInput interrupt_differential_front;
InterruptInput interrupt_differential_rear;

       
void setup(){  
    SetupManager SetupSPMB;
    SetupSPMB.delay_start(1);

    /* Global Interrupt Objects */
    interrupt_steering.initialise("Steering", 1500, BIT0, &DDRB, &PORTB, &PINB, &PCMSK0);                     // PB0 - D08 - PCINT0 
    interrupt_velocity.initialise("Velocity", 1500, BIT0, &DDRC, &PORTC, &PINC, &PCMSK1);                     // PC0 - D14 - PCINT8
    interrupt_transmission.initialise("Transmission", 1000, BIT4, &DDRD, &PORTD, &PIND, &PCMSK2);             // PD4 - D04 - PCINT20
    interrupt_differential_front.initialise("Differential Front", 1500, BIT5, &DDRD, &PORTD, &PIND, &PCMSK2);  // PD4 - D04 - PCINT20
    interrupt_differential_rear.initialise("Differential Rear", 1500, BIT6, &DDRD, &PORTD, &PIND, &PCMSK2);    // PD4 - D04 - PCINT20

    B.append_interrupt(&interrupt_steering);
    C.append_interrupt(&interrupt_velocity);
    D.append_interrupt(&interrupt_transmission);
    D.append_interrupt(&interrupt_differential_front);
    D.append_interrupt(&interrupt_differential_rear);
    
    interrupt_manager.append_group(&B);
    interrupt_manager.append_group(&C);
    interrupt_manager.append_group(&D);

    SetupSPMB.mInterruptManager = &interrupt_manager;
    SetupSPMB.configure();

    interrupt_manager.arm_interrupts();
}

//! The main loop algorithm
/*!
   The main logic loop algorithm
*/

void loop()
{   
    static volatile long t0 = micros();
    static volatile long dt = micros();
    while(1){

        // define these timer placeholders as array in state machine
        volatile uint16_t   time_period_steering, 
                            time_period_velocity, 
                            time_period_transmission, 
                            time_period_differential_rear, 
                            time_period_differential_front;
        
        dt = micros() - t0;
        if ((s2ms * T_loop_rate) < (us2ms * dt)){
            t0 = micros();
            util::print("Main Loop: Loop time is ", false);
            util::print(long(us2ms*dt), false);
            util::print(" ms - no waiting for next loop required.", false);
        }
        else{
            util::print("Main Loop: Loop time is ", false);
            util::print(long(us2ms*dt), false);
            util::print(" ms - wait for  ", false);
            util::print(long((s2ms * T_loop_rate) - (us2ms * dt)), false);
            util::print(" ms.", true);
            delay(long((s2ms * T_loop_rate) - (us2ms * dt)));
            t0 = micros();
        }

        interrupt_manager.rotate_interrupts();
        interrupt_manager.get_time_period("Steering", time_period_steering);
        interrupt_manager.get_time_period("Velocity", time_period_velocity);
        interrupt_manager.get_time_period("Transmission", time_period_transmission);
        interrupt_manager.get_time_period("Differential Front", time_period_differential_front);
        interrupt_manager.get_time_period("Differential Rear", time_period_differential_rear);
        util::print("Time Period Steering: ", false);
        util::print(time_period_steering, true);
        util::print("Time Period Velocity: ", false);
        util::print(time_period_velocity, true);
        util::print("Time Period Transmission: ", false);
        util::print(time_period_transmission, true);
        util::print("Time Period Differential Front: ", false);
        util::print(time_period_differential_front, true);
        util::print("Time Period Differential Rear: ", false);
        util::print(time_period_differential_rear, true);

        // process interrupt measurements 
        // go through all interrupt groups and if vaild and new time period then update
        // else toggle and reset
        // update ros time periods 
        // select new time periods 
        // lowpass filetr them
        // actuate 
        // publish 
        // sleep
    }    
}

ISR(PCINT0_vect) { 
    (*interrupt_manager.mInterruptGroups[0]).update_timer();
}
ISR(PCINT1_vect) {
    (*interrupt_manager.mInterruptGroups[1]).update_timer();
}
ISR(PCINT2_vect) {
    (*interrupt_manager.mInterruptGroups[2]).update_timer();
}