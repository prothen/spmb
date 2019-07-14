#include "spmb.h"

namespace SPMB{
  
    StateMachine::StateMachine():
        mControlFiltered()
    {
        mALIVE = true;
        mSW_CONTROL_ACTIVE = true; // TODO: DEBUG test case - change back to false !

        mSMTimePeriod = uint16_t(s2ms * T_LOOP_RATE);
        mCTRLTimePeriod = uint16_t(s2ms * T_CONTROL_LOOP_RATE);

        mSMTimestamp = micros();
        mCTRLTimestamp = micros();

        mSWLTimePeriod = uint16_t(s2ms * SWL_TIME_PERIOD_RESET);
        mSWLState=0;
        mS1=false;
        mS2=false;
        mS1_prev=mS1;
        mS2_prev=mS2;
    }
    #ifdef ROS_ACTIVE
    void StateMachine::configure(   InterruptManager* interrupt_manager_in, ROSInterface<myHardware>* ros_interface_in){
        mInterruptManager = interrupt_manager_in;
        mRosi = ros_interface_in;
    }
    #else
    void StateMachine::configure(   InterruptManager* interrupt_manager_in){
        mInterruptManager = interrupt_manager_in;
    }
    #endif
    
    void StateMachine::wait_for_next_cycle(){
        uint16_t dt = us2ms * uint16_t(micros() - mSMTimestamp);
        if (mSMTimePeriod < dt){
            util::print("Main Loop: Loop time is ", false);
            util::print(dt, false);
            util::print(" ms - no waiting for next loop required.", false);
        }
        else{
            util::print("Main Loop: Loop time is ", false);
            util::print(dt, false);
            util::print(" ms - wait for  ", false);
            util::print(float(mSMTimePeriod - dt), false);
            util::print(" ms.", true);
            delay(mSMTimePeriod - dt);
        }
        mSMTimestamp = micros();
    }
    
    void StateMachine::critical_error(){
        util::print("Critical Erorr encountered: Nominal State Machine Operation is disable", true);
    }

    void StateMachine::_update_rc_signals(util::control &signals_rc){          
        mInterruptManager->rotate_interrupts();
        mInterruptManager->get_time_period("Steering", signals_rc.steering);
        mInterruptManager->get_time_period("Velocity", signals_rc.velocity);
        mInterruptManager->get_time_period("Transmission", signals_rc.transmission);
        mInterruptManager->get_time_period("Differential Front", signals_rc.differential_front);
        mInterruptManager->get_time_period("Differential Rear", signals_rc.differential_rear);
        
        /*
        util::print("diff front: ", false);
        util::print(signals_rc.differential_front, true);
        util::print("diff rear: ", false);
        util::print(signals_rc.differential_rear, true);
        */
    }

    void StateMachine::_swl_execute_switching_logic(util::control* signals_rc){
        
        if ((mSW_CONTROL_ACTIVE && (signals_rc->velocity < PWM_CLEAR_TH_LOW)) 
                ||!(mInterruptManager->mInterruptGroups[1]->mInterrupts.front()->mStatus)){
            mSW_CONTROL_ACTIVE = false;
            mSWLState = SWL_OFF;
        }else{;}
        

        if (util::HAS_CLEAR_STATE(signals_rc->differential_front) && util::HAS_CLEAR_STATE(signals_rc->differential_rear)){
            mS1 = util::STATE(signals_rc->differential_front);
            mS2 = util::STATE(signals_rc->differential_rear);

            /*
            util::print("mS1: ", false);
            util::print(mS1, true);
            util::print("mS2: ", false);
            util::print(mS2, true);
            util::print("dt: ", false);
            util::print(float(us2ms*(micros()-mSWLTimestamp)), true);
            */

            switch(mSWLState){
                case SWL_OFF: {
                    mSWLTimestamp = micros();
                    if (mS1^mS2) {
                        mSWLState = SWL_ARM;
                        mS1_prev = mS1;
                        mS2_prev = mS2;
                    }else{;}
                    break;
                }
                case SWL_ARM: {
                    if ((mS1^mS2)&&(mS1!=mS1_prev)&&(mS2!=mS2_prev)) {
                        mSWLState = SWL_SWITCH;
                        mS1_prev = mS1;
                        mS2_prev = mS2;
                    }
                    else if(!(mS1^mS2)){
                        _swl_check_idle_transition();
                    }
                    else{
                        mSWLTimestamp = micros();
                    }
                    break;
                }
                case SWL_SWITCH: {
                    _swl_check_idle_transition();
                    if ((mS1^mS2)&&(mS1!=mS1_prev)&&(mS2!=mS2_prev)) {
                        mSWLState = SWL_OFF;
                        mS1_prev = mS1;
                        mS2_prev = mS2;
                        mSW_CONTROL_ACTIVE = true;
                    }else{;}
                    break;
                }
                default: ; //error
            }
        }
        else{;} // do nothing
        
        /*
        util::print("SW_CONTROL_ACTIVE:",false);
        util::print(mSW_CONTROL_ACTIVE, true);
        util::print("STATEMACHINE STATE:",false);
        util::print(mSWLState, true); 
        */       
    }

    void StateMachine::_swl_check_idle_transition(){
        if(util::IS_IDLE(mSWLTimestamp, mSWLTimePeriod)){ // TODO: DEBUG test case
            mSWLState = SWL_OFF;
            mS1 = false;
            mS2 = false;
            mS1_prev = mS1;
            mS2_prev = mS2;
            
            /*
            util::print("reset state machine", true);
            util::print("time_period: ", false);
            util::print(mSWLTimePeriod, true);
            util::print("time_period: ", false);
            util::print(mSWLTimestamp, true);
            */
            
        } else{;}
    }

    void StateMachine::_process_rc_inputs(util::control &signals_rc){
        _update_rc_signals(signals_rc);
        _swl_execute_switching_logic(&signals_rc);
    }
    
    #ifdef ROS_ACTIVE
    void StateMachine::_process_ros_inputs(util::control &signals_ros){
        mRosi->mNh.spinOnce();
        signals_ros = mRosi->mSignals;
    }
    #endif /* ROS_ACTIVE */


    void StateMachine::update_output_signals(){
        util::control output_signals;
        util::control signals_rc;
        util::control signals_ros;

        _process_rc_inputs(signals_rc);
        
        #ifdef ROS_ACTIVE
        _process_ros_inputs(signals_ros);
        #endif /* ROS_ACTIVE */
        if (mSW_CONTROL_ACTIVE){
            #ifdef ROS_ACTIVE
            output_signals = signals_ros;
            #else
            output_signals = signals_rc;
            #endif /* ROS_ACTIVE */
        }
        else{
            output_signals = signals_rc;
        }
        mControlFiltered.steering.update_state(output_signals.steering);
        mControlFiltered.velocity.update_state(output_signals.velocity);
        mControlFiltered.transmission.update_state(output_signals.transmission);
        mControlFiltered.differential_front.update_state(output_signals.differential_front);
        mControlFiltered.differential_rear.update_state(output_signals.differential_rear);
    
        /*
        util::print("steering: ", false);
        util::print(signals_rc.steering, true);
        util::print("steering filtered : ", false);
        util::print(mControlFiltered.steering.value(), true);
        */
    }
    #ifdef ROS_ACTIVE
    void StateMachine::_expose_actuated_signals_to_ros(){
        if (util::IS_TIME(mCTRLTimestamp, mCTRLTimePeriod)){
            //mRosi->mNh.loginfo(mControlFiltered.steering.value());
            mRosi->publish(&mControlFiltered);
        } else{;}
    } 
    #endif /* ROS_ACTIVE */

    void StateMachine::actuate(){
        // TODO: add actuate either servo library or via i2c

        #ifdef ROS_ACTIVE
        _expose_actuated_signals_to_ros();
        #endif /* ROS_ACTIVE */
    }

}