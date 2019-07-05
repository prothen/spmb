#pragma once

#include "spmb.h"

namespace SPMB{

    class StateMachine{
        public:
            InterruptManager* mInterruptManager;
            ROSInterface* mROSInterface;
            LowPass<uint16_t> signals(uint8_t, float);

            long timestamp;
            uint16_t time_period_main_loop; 
            uint16_t time_period_control_loop; // TODO initialise in constructor
            uint16_t time_period_measurement_loop;
            
            void configure( InterruptManager * interrupt_manager_in,
                            ROSInterface * ros_interface_in);

            void update_signals(util::control &);

            void wait_for_next_cycle();
            
            void execute_main_loop();
    };


}