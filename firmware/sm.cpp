#include "spmb.h"

namespace SPMB{

    void StateMachine::configure(   InterruptManager* interrupt_manager_in,
                                    ROSInterface* ros_interface_in
                                    ){
        // parse macros to state machine
        //mInterruptManager = interrupt_manager_in;
        mROSInterface = ros_interface_in;

    }

    void StateMachine::update_signals(util::control &signals_rc){  
        // always update interrupts
        
        mInterruptManager->rotate_interrupts();

        // parse new time periods  
        mInterruptManager->get_time_period("Steering", signals_rc.steering);
        mInterruptManager->get_time_period("Velocity", signals_rc.velocity);
        mInterruptManager->get_time_period("Transmission", signals_rc.transmission);
        mInterruptManager->get_time_period("Differential Front", signals_rc.differential_front);
        mInterruptManager->get_time_period("Differential Rear", signals_rc.differential_rear);
        
        // run logic based on tmpControl (switch mode)
    }


    void StateMachine::wait_for_next_cycle(){
        uint16_t dt = uint16_t(micros() - timestamp);
        if (T_loop_rate < dt){
            util::print("Main Loop: Loop time is ", false);
            util::print(dt, false);
            util::print(" ms - no waiting for next loop required.", false);
        }
        else{
            util::print("Main Loop: Loop time is ", false);
            util::print(dt, false);
            util::print(" ms - wait for  ", false);
            util::print((T_loop_rate - dt), false);
            util::print(" ms.", true);
            delay(T_loop_rate - dt);
        }
        timestamp = micros();
    }

    void StateMachine::execute_main_loop(){
        // add pointers to relevant modules that need to be interfaced
            // state
            // loop rate
            // timestamp
            // update timer
            // check timer
            // switch 
            // define switch for different modes
            // define callbacks for different modes


        // define these timer placeholders as array in state machine
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