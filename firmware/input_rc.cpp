#include "spmb.h"

namespace SPMB{

    /**
     * Defines a Interrupt pin and all its properties (needs to be in global namespace during runtime)
     */
    InterruptInput::InterruptInput(){;}            
    /**
     * Defines a Interrupt pin and all its properties (needs to be in global namespace during runtime)
     */
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
    /**
     * Takes care of setting up all the pins for input definitions and pull-up configuration of respective pin
     */
    void InterruptInput::configure(){
        *mDirectionRegister &= ~mBit;
        *mCfgRegister &= mBit; // TODO: switched to without pullup
        
        util::print("InterruptInput: Configure input and output pins for ", false);
        util::print(mLabel.c_str(), true); 
        util::print("InterruptInput: Direction ", false);
        util::print_binary(*mDirectionRegister);
        util::print("InterruptInput: Config    ", false);
        util::print_binary(*mCfgRegister);
    }

    /**
     * Disarms the Pin for Pin Change Interrupts
     */
    void InterruptInput::disarm_interrupt(){
        *mInterruptRegister &= ~mBit;

        /* 
        util::print("InterruptInput: Disarmed for ", false);
        util::print(mLabel.c_str(), true); 
        util::print("InterruptInput: Interrupt ", false);
        util::print_binary(*mInterruptRegister);  
        */
    }

    /**
     * Arms the Pin for Pin Change Interrupts
     */
    void InterruptInput::arm_interrupt(){
        *mInterruptRegister |= mBit;

        /* 
        util::print("InterruptInput: Armed for ", false);
        util::print(mLabel.c_str(), true); 
        util::print("InterruptInput: Interrupt ", false);
        util::print_binary(*mInterruptRegister);  
        */
    }

    /**
     * Handles complete interrupt logic (is called by CustomisedInterruptManager)
     */
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
            } else{;
                //util::print("should not have happened.", true);
                ///reset(); // TODO: this case should never be reached in normal operation and if reached, indicates a fundamental error in the setup
            }
        } else{;}
    }

    void InterruptInput::switch_error_status(){
        if (mStatus){
            if (n_interrupt_error_switch_off <= mErrorCount){
                mStatus = false;
                mErrorCount = 0;
                mTimePeriod = mTimePeriodDefault;
                
                
                util::print("Error switch: Disabled ", false);
                util::print(mLabel, true); 
                
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

    /**
     * Collects Interrupts with same properties
    */
    InterruptGroup::InterruptGroup(){
        mIdx = 0;
    }
    /**
     * Collects Interrupts with same properties
    */
    void InterruptGroup::update_timer(){
        //util::print("INT: update timer from interrupt group", true);
        (*mInterruptActive).update_timer();
    }

    void InterruptGroup::rotate_interrupts(){
        if (util::IS_VALID(mInterruptActive->mTimePeriod) && mInterruptActive->mNewInterruptAvailable) {
            //util::print("Interrupt - Rotate: Received Interrupt Time Period successfully: ", false);
            //util::print((*mInterruptActive).mLabel, true);
            
            mInterruptActive->mErrorCount = 0.; // TODO: Decide when or how often to reset error counter, now only counting consecutive errors, reset on each valid read
            mInterruptActive->disarm_interrupt();
            util::correct_period(mInterruptActive->mTimePeriod, mInterruptActive->mTimePeriodValid);

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
            } else{;} // keep the same Interrupt
            
        } else{
            //util::print("Interrupt: Error detected : Count ", false);
            mInterruptActive->mErrorCount++;
            //util::print((*mInterruptActive).mErrorCount, true);
            mInterruptActive->switch_error_status();
            //util::print("FAILED fetching period: ", false);
            //util::print((**mInterruptActive).mLabel, true);
        } 
        mInterruptActive->arm_interrupt();
        mInterruptActive->reset();
    }

    /* append new interrupt to group */
    void InterruptGroup::append_interrupt(InterruptInput * NewInterruptInput){
        mInterrupts.push_back(NewInterruptInput);
        mInterruptActive = mInterrupts.front();
    }



    /* append new group to interrupt via pointer*/
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

    /* */
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

    /* */
    void InterruptManager::arm_interrupts(){
        util::print("Start arming all active interrupts:", true);
        for(std::vector<InterruptGroup*>::iterator it = mInterruptGroups.begin(); it != mInterruptGroups.end(); it++){
            ((**it).mInterruptActive)->arm_interrupt();
        }
    }
}