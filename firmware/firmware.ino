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

using namespace SPMB;

StatusIndicator status_indicator;

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

    sm.configure_status_indicator(&status_indicator);
    
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
            sm.indicate_status();
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

    // PB0 - D08 - PCINT0 
    interrupt_steering.initialise(              "Steering", 
                                                DEFAULT_PWM_STEERING, 
                                                BIT0, &DDRB, &PORTB, &PINB, &PCMSK0); 

    // PC0 - D14 - PCINT8                 
    interrupt_velocity.initialise(              "Velocity", 
                                                DEFAULT_PWM_VELOCITY, 
                                                BIT0, &DDRC, &PORTC, &PINC, &PCMSK1);     

    // PD4 - D04 - PCINT20                          
    interrupt_transmission.initialise(          "Transmission", 
                                                DEFAULT_PWM_TRANSMISSION, 
                                                BIT4, &DDRD, &PORTD, &PIND, &PCMSK2);                  

     // PD4 - D04 - PCINT20 
    interrupt_differential_front.initialise(    "Differential Front", 
                                                DEFAULT_PWM_DIFFERENTIAL_FRONT, 
                                                BIT5, &DDRD, &PORTD, &PIND, &PCMSK2);

     // PD4 - D04 - PCINT20
    interrupt_differential_rear.initialise(     "Differential Rear", 
                                                DEFAULT_PWM_DIFFERENTIAL_REAR, 
                                                BIT6, &DDRD, &PORTD, &PIND, &PCMSK2);   

    B.append_interrupt(&interrupt_steering);
    C.append_interrupt(&interrupt_velocity);
    D.append_interrupt(&interrupt_transmission);
    D.append_interrupt(&interrupt_differential_front);
    D.append_interrupt(&interrupt_differential_rear);
    
    interrupt_manager.append_group(&B);
    interrupt_manager.append_group(&C);
    interrupt_manager.append_group(&D);

    SetupSPMB.configure_common();
    SetupSPMB.configure_status_indicator(&status_indicator);
    SetupSPMB.configure_interrupts(&interrupt_manager);
    SetupSPMB.configure_output(&output);
    SetupSPMB.arm_system();
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
