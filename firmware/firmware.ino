#include "spmb.h"

using namespace SPMB;

StateMachine sm;

ROSInterface rosi;

void cb_request(const spmbv2::request& msg){util::print(msg.steering, false);}

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
    SetupSPMB.configure(&interrupt_manager);

    interrupt_manager.arm_interrupts();

}

//! The main loop algorithm
/*!
   The main logic loop algorithm
*/

void loop()
{  
    // setup ros
    spmbv2::actuated msg_actuated;
    spmbv2::request msg_request;
    ros::NodeHandle_<ArduinoHardware, 1, 1, 240, 240> Nh;
    ros::Publisher pub("actuated", &msg_actuated);
    ros::Subscriber<spmbv2::request> sub("request", &cb_request);

    //rosi.configure(&nh, &sub, &pub);

    while(1){
        // use state machine and run loop while sm.alive()
    }    
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