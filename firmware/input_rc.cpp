/*
 Copyright (c) 2019, Philipp Rothenhäusler. All rights reserved.

 Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name of Philipp Rothenhäusler nor the names of its contributors may
   be used to endorse or promote products derived from this software without
   specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "spmb.h"

namespace SPMB{

    InterruptInput::InterruptInput(){;}            

    void InterruptInput::initialise(    std::string labelInput_,
                                        uint16_t TimePeriodDefault_,
                                        uint8_t bit_,
                                        volatile uint8_t * direction_register_,
                                        volatile uint8_t * configuration_register_,
                                        volatile uint8_t * data_register_,
                                        volatile uint8_t * interrupt_register_) {
        mLabel = labelInput_;
        mTimePeriodDefault = TimePeriodDefault_;
        mBit = bit_;
        mDirectionRegister = direction_register_;
        mCfgRegister = configuration_register_; 
        mDataRegister = data_register_,
        mInterruptRegister = interrupt_register_;
        
        mStatus = true;
        mErrorCount = 0;
        mTimer = micros();
        mTimePeriod = 0;

        reset();
        
        disarm_interrupt();
    } 

    void InterruptInput::configure(){
        *mDirectionRegister &= ~mBit;
        *mCfgRegister |= mBit; 
        
        util::print("InterruptInput: Configure input and output pins for ", false);
        util::print(mLabel.c_str(), true); 
        util::print("InterruptInput: Direction ", false);
        util::print_binary(*mDirectionRegister);
        util::print("InterruptInput: Config    ", false);
        util::print_binary(*mCfgRegister);
    }

    void InterruptInput::disarm_interrupt(){
        *mInterruptRegister &= ~mBit;

        /* 
        util::print("InterruptInput: Disarmed for ", false);
        util::print(mLabel.c_str(), true); 
        util::print("InterruptInput: Interrupt ", false);
        util::print_binary(*mInterruptRegister);  
        */
    }

    void InterruptInput::arm_interrupt(){
        *mInterruptRegister |= mBit;

        /* 
        util::print("InterruptInput: Armed for ", false);
        util::print(mLabel.c_str(), true); 
        util::print("InterruptInput: Interrupt ", false);
        util::print_binary(*mInterruptRegister);  
        */
    }

    void InterruptInput::update_timer(){
        if (!mNewInterruptAvailable && mStatus){
            long tmpTime = micros();

            /*
            util::print("update timer:", false);
            util::print(mReceivedHigh, false);
            util::print(mReceivedLow, false);
            util::print(mNewInterruptAvailable, false);
            util::print(mTimePeriod, false);
            util::print(" :", false);
            */

            if(((* mDataRegister) & mBit) && mReceivedLow){
                mTimer = tmpTime;
                mReceivedHigh = true;
                //util::print("RECEIVED_HIGH.", true);
            }
            else if ( !((* mDataRegister) & mBit) && (!(mReceivedHigh))) {
                mReceivedLow = true;
                //util::print("RECEIVED_LOW.(first)", true);
            }
            else if ( !((* mDataRegister) & mBit) && mReceivedHigh && mReceivedLow && !mNewInterruptAvailable) {
                    mTimePeriod = uint16_t(tmpTime - mTimer);
                    mNewInterruptAvailable = true;
                    //util::print("RECEIVED_LOW.(second: complete cycle): ", false);
                    //util::print(mTimePeriod, true);
            } else{;}
        } else{;}
    }

    void InterruptInput::switch_error_status(){
        if (mStatus){
            if (n_interrupt_error_switch_off <= mErrorCount){
                mStatus = false;
                mErrorCount = 0;
                mTimePeriodValid = mTimePeriodDefault;
                
                /*
                util::print("Error switch: Disabled ", false);
                util::print(mLabel, true);     
                */
            }
        }
        else{
            if (n_interrupt_error_switch_on <= mErrorCount){
                mStatus = true;
                mErrorCount = 0;
                
                /* 
                util::print("Error switch: Enabled ", false);
                util::print(mLabel, true); 
                */
            }
        }
    }

    void InterruptInput::reset(){
        mReceivedHigh = false;
        mReceivedLow = false;
        mNewInterruptAvailable = false;

        /* 
        util::print("Interrupt ", false);
        util::print(mLabel, false);
        util::print(" has been reset.", true); 
        */
    }

    InterruptGroup::InterruptGroup(){
        mIdx = 0;
    }

    void InterruptGroup::update_timer(){
        //util::print("INT: update timer from interrupt group", true);
        mInterruptActive->update_timer();
    }

    void InterruptGroup::_attempt_rotating(){
        if (mInterrupts.size() > 1) {
                //util::print("Start rotating: ", true);
                uint8_t tmpIdx = mIdx;
                while ((mInterrupts[tmpIdx]->mLabel == mInterrupts[mIdx]->mLabel) || (!(mInterrupts[mIdx]->mStatus))){
                    //util::print("Interrupt - Rotate: Interrupt Status not good or same Pointer: Check for next element: ", true);
                    //util::print((*mInterrupts[mIdx]).mLabel, true);
                    if (mIdx < (mInterrupts.size()-1)){
                        mIdx++; 
                        //util::print("end not reached ", true);
                    } else{
                        mIdx = 0;
                        //util::print("end reached ", true);
                    }
                    if (!(mInterrupts[mIdx]->mStatus)) {
                        mInterrupts[mIdx]->mErrorCount++;
                        mInterrupts[mIdx]->switch_error_status();
                        
                        /*
                        util::print("Interrupt - Rotate: Skip disabled Interupt detected :", false);
                        util::print((*mInterrupts[mIdx]).mLabel, false);
                        util::print("Interrupt - Rotate:  with Count: ", false);
                        util::print((*mInterrupts[mIdx]).mErrorCount, true);
                        */
                    }
                }
                mInterruptActive = mInterrupts.at(mIdx);                
            } else{
                mInterrupts[mIdx]->mErrorCount = 0;
                mInterrupts[mIdx]->mStatus = true;
            } // keep the same Interrupt
    }

    void InterruptGroup::rotate_interrupts(){
        if (util::IS_VALID(mInterruptActive->mTimePeriod) && mInterruptActive->mNewInterruptAvailable) {
            //util::print("Interrupt - Rotate: Received Interrupt Time Period successfully: ", false);
            //util::print((*mInterruptActive).mLabel, true);
            
            mInterruptActive->mErrorCount = 0.; // \Todo Decide when or how often to reset error counter, now only counting consecutive errors, reset on each valid read
            mInterruptActive->disarm_interrupt();
            util::correct_period(mInterruptActive->mTimePeriod, mInterruptActive->mTimePeriodValid);
            mInterruptActive->mTimePeriod = 0;
            _attempt_rotating();
            
        } else{
            if (mInterruptActive->mStatus){
                //util::print("Interrupt: Error detected : Count ", false);
                mInterruptActive->mErrorCount++;
                //util::print((*mInterruptActive).mErrorCount, true);
                mInterruptActive->switch_error_status();
                //util::print("FAILED fetching period: ", false);
                //util::print((**mInterruptActive).mLabel, true);
            } else{
                _attempt_rotating();
            }
        } 
        mInterruptActive->arm_interrupt();
        mInterruptActive->reset();
    }

    void InterruptGroup::append_interrupt(InterruptInput * NewInterruptInput){
        mInterrupts.push_back(NewInterruptInput);
        mInterruptActive = mInterrupts.front();
    }

    void InterruptManager::append_group(InterruptGroup * NewInterruptGroup){
        mInterruptGroups.push_back(NewInterruptGroup);
    }
    
    bool InterruptManager::get_time_period(std::string label, volatile uint16_t &time_period){
        //util::print("start searching for label: ", false);
        //util::print(label, true);
        //int i = 0;
        for(typename std::vector<InterruptGroup*>::iterator it = mInterruptGroups.begin(); it != mInterruptGroups.end(); it++){

            //util::print("    group: ", false);
            //util::print(i, true);
            //i++;
            //int j = 0;
            for(typename std::vector<InterruptInput*>::iterator iit = (**it).mInterrupts.begin(); iit != (**it).mInterrupts.end(); iit++){
                /* util::print("        interrupt: ", false);
                util::print(j, false);
                util::print(" ( ", false);
                util::print((**iit).mLabel, false);
                util::print(" )", true);
                 */
                //j++;
                if ((**iit).mLabel == label){
                    //util::print((**iit).mLabel.c_str(), false);
                    time_period = (**iit).mTimePeriodValid;
                    return true;
                } else{;}
            }
        }
        return false;
    }

    void InterruptManager::rotate_interrupts(){
        //util::print("    start rotating", true);
        //int i = 0;
        for(std::vector<InterruptGroup*>::iterator it = mInterruptGroups.begin(); it != mInterruptGroups.end(); it++){
            //util::print("        ... rotating ", false);
            //util::print(i, true);
            //i++;
            (**it).rotate_interrupts();
        }
    }

    void InterruptManager::arm_interrupts(){
        util::print("Start arming all active interrupts:", true);
        for(std::vector<InterruptGroup*>::iterator it = mInterruptGroups.begin(); it != mInterruptGroups.end(); it++){
            ((**it).mInterruptActive)->arm_interrupt();
        }
    }
}