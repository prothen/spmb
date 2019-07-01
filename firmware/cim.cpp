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
        mErrorCount = 0;
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
        long tmpTime = micros();
        if(((* mDataRegister) & mBit) && mReceivedLow){
            mTimer = tmpTime;
            mReceivedHigh = true;
        }
        else if ( !((* mDataRegister) & mBit) && (!(mReceivedHigh))) {
                mReceivedLow = true;
        }
        else if ( !((* mDataRegister) & mBit) && mReceivedHigh && mReceivedLow && !mNewInterruptAvailable) {
                mTimePeriod = uint16_t(micros() - mTimer);
                mNewInterruptAvailable = true;
        } else{
            ; ///reset(); // TODO: this case should never be reached in normal operation and if reached, indicates a fundamental error in the setup
        }
    }

    void InterruptInput::reset(){
        mReceivedHigh = false;
        mReceivedLow = false;
        mNewInterruptAvailable = false;
    }

    /**
     * Collects Interrupts with same properties
    */
    void InterruptGroup::update_timer(){
        (**mInterruptActive).update_timer();
    }

    void InterruptGroup::rotate_interrupts(){
        if (util::IS_VALID((**mInterruptActive).mTimePeriod) && (**mInterruptActive).mNewInterruptAvailable) {
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
                    } else{
                        (**mInterruptActive).reset(); // prepare new interrupt and reset conditions
                    }
                }
                
            } else{;} // keep the same Interrupt
        }
        else{
            (**mInterruptActive).mErrorCount++;
            (**mInterruptActive).reset();
        }
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

    /* */
    void InterruptManager::rotate_interrupts(){
        for(std::vector<InterruptGroup*>::iterator it = mInterruptGroups.begin(); it != mInterruptGroups.end(); it++ ){
            (**it).rotate_interrupts();
        }
    }
}