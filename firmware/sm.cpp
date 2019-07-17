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

namespace SPMB{
    
    StateMachine::StateMachine():
        mControlFiltered()
    {
        mALIVE = true;
        mSW_CONTROL_ACTIVE = false;
        mStatusIndicator->switch_status(StatusIndicator::SIStatus::RC_ON);

        mSMTimePeriod = uint16_t(s2ms * T_LOOP_RATE);
        mCTRLTimePeriod = uint16_t(s2ms * T_CONTROL_LOOP_RATE);

        mSMTimestamp = micros();
        mCTRLTimestamp = micros();

        mSWLTimePeriod = uint16_t(s2ms * SWL_TIME_PERIOD_RESET);
        mSWLState=SWStates::OFF;
        mS1=false;
        mS2=false;
        mS1_prev=mS1;
        mS2_prev=mS2;
    }

    void StateMachine::configure_status_indicator(StatusIndicator* status_indicator_in){
        mStatusIndicator = status_indicator_in;
    }

    #ifdef ROS_ACTIVE
    void StateMachine::configure_input(   InterruptManager* interrupt_manager_in, ROSInterface<myHardware>* ros_interface_in){
        mInterruptManager = interrupt_manager_in;
        mRosi = ros_interface_in;
    }
    #else
    void StateMachine::configure_input(   InterruptManager* interrupt_manager_in){
        mInterruptManager = interrupt_manager_in;
    }
    #endif
    
    void StateMachine::configure_output(   OutputDriverI2C* output_in){
        mOutput = output_in;
    }

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
                ||!(mInterruptManager->mInterruptGroups[1]->mInterrupts.front()->mStatus)
                #ifdef ROS_ACTIVE
                || mRosi->mIsIdle
                #endif /*  ROS_ACTIVE */
                ){
            mSWLState = SWStates::OFF;
            mSW_CONTROL_ACTIVE = false;
            mStatusIndicator->switch_status(StatusIndicator::SIStatus::RC_ON);
        }else{;}
        

        if (util::HAS_CLEAR_STATE(signals_rc->differential_front) && util::HAS_CLEAR_STATE(signals_rc->differential_rear)){
            mS1 = util::STATE(signals_rc->differential_front);
            mS2 = util::STATE(signals_rc->differential_rear);

            util::print("mS1: ", false);
            util::print(mS1, true);
            util::print("mS2: ", false);
            util::print(mS2, true);
            util::print("dt: ", false);
            util::print(float(us2ms*(micros()-mSWLTimestamp)), true);

            switch(mSWLState){
                case SWStates::OFF: {
                    mSWLTimestamp = micros();
                    if (mS1^mS2) {
                        mS1_prev = mS1;
                        mS2_prev = mS2;
                        mSWLState = SWStates::ARM;
                    }else{;}
                    break;
                }
                case SWStates::ARM: {
                    if ((mS1^mS2)&&(mS1!=mS1_prev)&&(mS2!=mS2_prev)) {
                        mS1_prev = mS1;
                        mS2_prev = mS2;
                        mSWLState = SWStates::SWITCH;
                    }
                    else if(!(mS1^mS2)){
                        _swl_check_idle_transition();
                    }
                    else{
                        mSWLTimestamp = micros();
                    }
                    break;
                }
                case SWStates::SWITCH: {
                    _swl_check_idle_transition();
                    if ((mS1^mS2)&&(mS1!=mS1_prev)&&(mS2!=mS2_prev)) {
                        mS1_prev = mS1;
                        mS2_prev = mS2;
                        mSWLState = SWStates::OFF;
                        mSW_CONTROL_ACTIVE = true;
                        mStatusIndicator->switch_status(StatusIndicator::SIStatus::SW_ON);
                    }else{;}
                    break;
                }
                default: ; 
            }
        } else{;} 
        
        util::print("SW_CONTROL_ACTIVE:",false);
        util::print(mSW_CONTROL_ACTIVE, true);
        util::print("STATEMACHINE STATE:",false);
        util::print(mSWLState, true);        
    }

    void StateMachine::_swl_check_idle_transition(){
        if(util::IS_IDLE(mSWLTimestamp, mSWLTimePeriod)){ // TODO: DEBUG test case
            mS1 = false;
            mS2 = false;
            mS1_prev = mS1;
            mS2_prev = mS2;
            mSWLState = SWStates::OFF;
            
            util::print("reset state machine", true);
            util::print("time_period: ", false);
            util::print(mSWLTimePeriod, true);
            util::print("time_period: ", false);
            util::print(mSWLTimestamp, true);            
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
        mRosi->mIsIdle = util::IS_IDLE(mRosi->mTimestamp, mRosi->mMinTimePeriod);
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
    
        util::print("steering: ", false);
        util::print(signals_rc.steering, true);
        util::print("steering filtered : ", false);
        util::print(mControlFiltered.steering.value(), true);
    }
    #ifdef ROS_ACTIVE
    void StateMachine::_expose_actuated_signals_to_ros(){
        if (util::IS_TIME(mCTRLTimestamp, mCTRLTimePeriod)){
            mRosi->publish(&mControlFiltered);
        } else{;}
    } 
    #endif /* ROS_ACTIVE */

    void StateMachine::actuate(){
        mOutput->actuate(&mControlFiltered);

        #ifdef ROS_ACTIVE
        _expose_actuated_signals_to_ros();
        #endif /* ROS_ACTIVE */
    }

    void StateMachine::indicate_status(){
        mStatusIndicator->blink();
    }

    void StateMachine::critical_error(){
        util::print("Critical Erorr encountered: Nominal State Machine Operation is disable", true); 
        mStatusIndicator->switch_status(StatusIndicator::SIStatus::EMERGENCY);
    }
}