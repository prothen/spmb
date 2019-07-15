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

OutputDriverI2C output;

/*! The main logic loop algorithm */
void loop()
{
    StateMachine sm;
    sm.mInterruptManager = &interrupt_manager;
    
    #ifdef ROS_ACTIVE
    ROSInterface<myHardware> rosi("request", "actuated");
    sm.configure_input(&interrupt_manager, &rosi);
    #else
    sm.configure_input(&interrupt_manager);
    #endif /* ROS_ACTIVE */

    sm.configure_output(&output);

    while(1){
        if (sm.mALIVE){
            sm.update_output_signals();
            sm.actuate();
            sm.wait_for_next_cycle();
        }
        else{
            sm.critical_error();
        }    
    }
}

/*! Configure Interrupts and pins */       
void setup(){
    SetupManager SetupSPMB;
    SetupSPMB.delay_start(1);

    interrupt_steering.initialise("Steering", DEFAULT_PWM_STEERING, BIT0, &DDRB, &PORTB, &PINB, &PCMSK0);                               // PB0 - D08 - PCINT0 
    interrupt_velocity.initialise("Velocity", DEFAULT_PWM_VELOCITY, BIT0, &DDRC, &PORTC, &PINC, &PCMSK1);                               // PC0 - D14 - PCINT8
    interrupt_transmission.initialise("Transmission", DEFAULT_PWM_TRANSMISSION, BIT4, &DDRD, &PORTD, &PIND, &PCMSK2);                   // PD4 - D04 - PCINT20
    interrupt_differential_front.initialise("Differential Front", DEFAULT_PWM_DIFFERENTIAL_FRONT, BIT5, &DDRD, &PORTD, &PIND, &PCMSK2); // PD4 - D04 - PCINT20
    interrupt_differential_rear.initialise("Differential Rear", DEFAULT_PWM_DIFFERENTIAL_REAR, BIT6, &DDRD, &PORTD, &PIND, &PCMSK2);    // PD4 - D04 - PCINT20

    B.append_interrupt(&interrupt_steering);
    C.append_interrupt(&interrupt_velocity);
    D.append_interrupt(&interrupt_transmission);
    D.append_interrupt(&interrupt_differential_front);
    D.append_interrupt(&interrupt_differential_rear);
    
    interrupt_manager.append_group(&B);
    interrupt_manager.append_group(&C);
    interrupt_manager.append_group(&D);

    SetupSPMB.configure_common();
    SetupSPMB.configure_interrupts(&interrupt_manager);
    SetupSPMB.configure_output(&output);

    interrupt_manager.arm_interrupts();
}

ISR(PCINT0_vect) { 
    interrupt_manager.mInterruptGroups[0]->update_timer();
}
ISR(PCINT1_vect) {
    interrupt_manager.mInterruptGroups[1]->update_timer();
}
ISR(PCINT2_vect) {
    interrupt_manager.mInterruptGroups[2]->update_timer();
}
