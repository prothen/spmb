#pragma once

#include "spmb.h"

namespace SPMB{

    class StateMachine{
        public:
            InterruptManager* mInterruptManager;
            ROSInterface<myHardware>* mRosi;
            

            util::control_filtered mControlFiltered;
            bool mALIVE;
            bool mSW_CONTROL_ACTIVE;
            
            long mSMTimestamp;
            long mCTRLTimestamp;

            uint16_t mSMTimePeriod; 
            uint16_t mCTRLTimePeriod;
            
            // Switching Logic (SWL)
            uint8_t mSWLState;  // 0, 1, 2 - off - arm - switch!
            uint16_t mSWLTimePeriod;
            long mSWLTimestamp;
            boolean mS1;
            boolean mS2;
            boolean mS1_prev;
            boolean mS2_prev;

            StateMachine();

            void configure( InterruptManager* interrupt_manager_in, ROSInterface<myHardware>* ros_interface_in);
            //void configure( InterruptManager* interrupt_manager_in, ROSInterfaceNh* ros_interface_in);

            void _update_rc_signals(util::control &signals_rc);

            void _swl_execute_switching_logic(util::control* signals_rc);

            void _swl_check_idle_transition();

            void wait_for_next_cycle();
            
            void critical_error();

            void _process_rc_inputs(util::control &signals_rc);

            void _process_ros_inputs(util::control &signals_ros);

            void update_output_signals();

            void _expose_actuated_signals_to_ros();

            void actuate();
    };


}