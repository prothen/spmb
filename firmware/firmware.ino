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
    interrupt_steering.initialise("steering", BIT0, &DDRB, &PORTB, &PINB, &PCMSK0);        // PB0 - D08 - PCINT0 
    interrupt_velocity.initialise("velocity", BIT0, &DDRC, &PORTC, &PINC, &PCMSK1);        // PC0 - D14 - PCINT8
    interrupt_transmission.initialise("transmission", BIT4, &DDRD, &PORTD, &PIND, &PCMSK2);    // PD4 - D04 - PCINT20
    interrupt_differential_front.initialise("differentialfront", BIT5, &DDRD, &PORTD, &PIND, &PCMSK2);    // PD4 - D04 - PCINT20
    interrupt_differential_rear.initialise("differentialrear", BIT6, &DDRD, &PORTD, &PIND, &PCMSK2);    // PD4 - D04 - PCINT20

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


    while(1){

        static volatile long t0 = micros();
        static volatile long dt = micros();
        volatile uint16_t   time_period_steering, 
                            time_period_velocity, 
                            time_period_transmission, 
                            time_period_differential_rear, 
                            time_period_differential_front;
        t0 = micros();
        delay(300);
        interrupt_manager.rotate_interrupts();
        time_period_steering = interrupt_manager.get_time_period("steering");
        time_period_velocity = interrupt_manager.get_time_period("velocity");
        time_period_transmission = interrupt_manager.get_time_period("transmission");
        time_period_differential_front = interrupt_manager.get_time_period("differentialfront");
        time_period_differential_rear = interrupt_manager.get_time_period("differentialrear");
        //util::print()
        util::print("Time Period Steering:", false);
        util::print(time_period_steering, true);
        util::print("Time Period Velocity:", false);
        util::print(time_period_velocity, true);
        util::print("Time Period Transmission:", false);
        util::print(time_period_transmission, true);
        util::print("Time Period Differential Front:", false);
        util::print(time_period_differential_front, true);
        util::print("Time Period Differential Rear:", false);
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


        dt = micros() - t0;
        //util::print("Main Loop active with loop time: ", false);
        //util::print(dt, true);
        
        // switch interrupts if valid time period has been recorded for active interrupt in interrupt group 
        // apply lowpass filter to read time periods 
        // update external timer module
        // update state machine and trigger switch in necessary (state machine should indicate current mode with led freq)
        // sent ros message
        // create spmb object
        // init interruptpins from global in interrupt groups
        // init interrupt groups in interruptManager
        // init error_object
        // init input manager (add error object link)
        // define input manager
        // update inputs
        // DEFINE MAIN VARIABLES
        // actuate 

        // Add Scheduler Object
        
        // while STATUS_BOOLEAN from state machine // is set to False by Error Object
                // INPUT_H
        //update_input_pwm_periods();
                //actuate();
                //ROS_publish_data();
                //update_control_mode(); // TODO: Replace with state machine
                //update_measurements();
                //print_diagnosis();
    }    
    // TODO: limit loop time to 100 Hz 
}
// scheduler using interrut_manager:
/*
v1
    - always choose one interrupt and then rotate, if no signal received, drop (if not empty)
    - in main loop check last updated, if above threshold, put on include
    - every 5s reintroduce interrupt and check whether it receives sth

v2
    - iterate through all interrupts and rotate their sequence each loop
    ( if the problem is in fact a priority collision and not hardware related )

*/


// implement groups active for all three in the same way
// but if only one interrupt in group then never reject (always keep one)
ISR(PCINT0_vect) { 
    (*interrupt_manager.mInterruptGroups[0]).update_timer();
}
ISR(PCINT1_vect) {
    (*interrupt_manager.mInterruptGroups[1]).update_timer();
}
ISR(PCINT2_vect) {
    (*interrupt_manager.mInterruptGroups[2]).update_timer();
}



/*

    // test automatic configuration with initialise function
    // test pull up and pull down
    // test multiple interrupts 
    for(long i=0; i<=100000; i++){
        for(long y=0; y<=100000; y++){;};
    }
    util::print_binary(PINB&BIT0);

    //util::print("Interrupt", false); 
    //util::print("PCINT0_vect", true); // wait to open terminal
    util::print_binary(PINB);
*/