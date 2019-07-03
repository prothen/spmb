#include "spmb.h"

namespace SPMB{
    #define ms2us 1000.

    /**
     * Defines a Interrupt pin and all its properties (needs to be in global namespace during runtime)
     */
    InterruptInput::InterruptInput(){;}            
    /**
     * Defines a Interrupt pin and all its properties (needs to be in global namespace during runtime)
     */
    void InterruptInput::initialise(    std::string labelInput_,
                                        volatile uint8_t bit_,
                                        volatile uint8_t * direction_register_,
                                        volatile uint8_t * configuration_register_,
                                        volatile uint8_t * data_register_,
                                        volatile uint8_t * interrupt_register_) {
        mLabel = labelInput_;
        mBit = bit_;
        mDirectionRegister = direction_register_;
        mCfgRegister = configuration_register_; 
        mDataRegister = data_register_,
        mInterruptRegister = interrupt_register_;
        
        mStatus = true;
        mErrorCount = 0;
        mTimer = micros();
        mTimePeriod = 1500;

        reset();
        
        disarm_interrupt();
    } 
    /**
     * Takes care of setting up all the pins for input definitions and pull-up configuration of respective pin
     */
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

    /**
     * Disarms the Pin for Pin Change Interrupts
     */
    void InterruptInput::disarm_interrupt(){
        *mInterruptRegister &= ~mBit;
        mStatus = false;

        util::print("InterruptInput: Disarmed for ", false);
        util::print(mLabel.c_str(), true); 
        util::print("InterruptInput: Interrupt ", false);
        util::print_binary(*mInterruptRegister);
    }

    /**
     * Arms the Pin for Pin Change Interrupts
     */
    void InterruptInput::arm_interrupt(){
        *mInterruptRegister |= mBit;
        mStatus = true;

        util::print("InterruptInput: Armed for ", false);
        util::print(mLabel.c_str(), true); 
        util::print("InterruptInput: Interrupt ", false);
        util::print_binary(*mInterruptRegister);
    }

    /**
     * Handles complete interrupt logic (is called by CustomisedInterruptManager)
     */
    void InterruptInput::update_timer(){
        if (!mNewInterruptAvailable){
            long tmpTime = micros();
            /*util::print("update timer:", false);
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
            } else{
                //util::print("should not have happened.", true);
                ; ///reset(); // TODO: this case should never be reached in normal operation and if reached, indicates a fundamental error in the setup
            }
        } else{;}
    }

    void InterruptInput::reset(){
        mReceivedHigh = false;
        mReceivedLow = false;
        mNewInterruptAvailable = false;
        util::print("Interrupt ", false);
        util::print(mLabel, false);
        util::print(" has been reset.", true);
    }

    /**
     * Collects Interrupts with same properties
    */
    void InterruptGroup::update_timer(){
        //util::print("INT: update timer from interrupt group", true);
        (**mInterruptActive).update_timer();
    }

    void InterruptGroup::rotate_interrupts(){
        if (util::IS_VALID((**mInterruptActive).mTimePeriod) && (**mInterruptActive).mNewInterruptAvailable) {
            util::print("Disable successfully: ", true);
            util::print((**mInterruptActive).mLabel, true);
            //(**mInterruptActive).disarm_interrupt();
            
            if (mInterrupts.size() > 1) {
                std::vector<InterruptInput*>::iterator tmp_root_pointer = mInterruptActive;
                while (!(**mInterruptActive).mStatus){
                    if (mInterruptActive != mInterrupts.end()){
                        mInterruptActive++;
                    } else{
                        mInterruptActive = mInterrupts.begin();
                    }
                    if (mInterruptActive == tmp_root_pointer){
                        break; // keep the same pointer, we searched all available interrupts unsuccessfully
                    } else{;} // prepare new interrupt and reset conditions
                }
                
            } else{;} // keep the same Interrupt
            
        } else{
            (**mInterruptActive).mErrorCount++;
            //util::print("FAILED fetching period: ", false);
            //util::print((**mInterruptActive).mLabel, true);
        }
        util::print("Disable: ", true);
            
        //util::print("Enable successfully: ", false);
        //util::print((**mInterruptActive).mLabel, true);
        (**mInterruptActive).arm_interrupt();
        (**mInterruptActive).reset();
    }

    /* append new interrupt to group */
    void InterruptGroup::append_interrupt(InterruptInput * NewInterruptInput){
        mInterrupts.push_back(NewInterruptInput);
        mInterruptActive = mInterrupts.begin();
    }

    /* append new group to interrupt via pointer*/
    void InterruptManager::append_group(InterruptGroup * NewInterruptGroup){
        mInterruptGroups.push_back(NewInterruptGroup);
    }
    
    volatile uint16_t InterruptManager::get_time_period(std::string label){
        volatile uint16_t return_value;
        //util::print("start searching for label: ", false);
        //util::print(label, true);
        int i = 0;
        for(typename std::vector<InterruptGroup*>::iterator it = mInterruptGroups.begin(); it != mInterruptGroups.end(); it++){

            //util::print("    group: ", false);
            //util::print(i, true);
            i++;
            int j = 0;
            for(typename std::vector<InterruptInput*>::iterator iit = (**it).mInterrupts.begin(); iit != (**it).mInterrupts.end(); iit++){
                /* util::print("        interrupt: ", false);
                util::print(j, false);
                util::print(" ( ", false);
                util::print((**iit).mLabel, false);
                util::print(" )", true);
                 */
                j++;
                if ((**iit).mLabel == label){
                    //util::print((**iit).mLabel.c_str(), false);
                    (**iit).mTimePeriod++;
                    return_value = (**iit).mTimePeriod;
                    //(**iit).reset();
                    return return_value;
                } else{;}
            }
        }
        return return_value;
    }

    /* */
    void InterruptManager::rotate_interrupts(){
        util::print("    start rotating", true);
        int i = 0;
        for(std::vector<InterruptGroup*>::iterator it = mInterruptGroups.begin(); it != mInterruptGroups.end(); it++){
            util::print("        ... rotating ", false);
            util::print(i, true);
            i++;
            (**it).rotate_interrupts();
        }
    }

    /* */
    void InterruptManager::arm_interrupts(){
        util::print("Start arming all active interrupts:", true);
        for(std::vector<InterruptGroup*>::iterator it = mInterruptGroups.begin(); it != mInterruptGroups.end(); it++){
            (**(**it).mInterruptActive).arm_interrupt();
        }
    }
}