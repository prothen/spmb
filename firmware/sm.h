#pragma once

#include "spmb.h"

namespace SPMB{


    /**
     *  A test class
     */

    /*! \todo Add status LED class */
    class StateMachine{

        public:
            InterruptManager* mInterruptManager;        //!< Pointer to interrupt manager provides access to all interrupt results. 

            #ifdef ROS_ACTIVE
            ROSInterface<myHardware>* mRosi;            //!< Pointer to ROS interface that provides access to all ROS related actions. 
            #endif /* ROS_ACTIVE */

            OutputDriverI2C* mOutput;                   //!<  Pointer to output driver that provides access to all hardware related actuation actions. 

            util::control_filtered mControlFiltered;    //!< Filtered control output for actuation via output driver. 
            bool mALIVE;                                //!< Indicates state machine health status. 
            bool mSW_CONTROL_ACTIVE;                    //!< Activation boolean for software control loop 
            
            long mSMTimestamp;                          //!< Timestamp of last main loop iteration. 
            long mCTRLTimestamp;                        //!< Timestamp of last control actuation. 

            uint16_t mSMTimePeriod;                     //!< Time period of main logic loop 
            uint16_t mCTRLTimePeriod;                   //!< Timp period of control loop
            
            uint8_t mSWLState;                          //!< State of switching logic, see macros for states. 
            uint16_t mSWLTimePeriod;                    //!< Time period of switching logic to detect idle 
            long mSWLTimestamp;                         //!< Timestamp of switching logic 
            boolean mS1;                                //!< Channel 4 (differential_front) state 
            boolean mS2;                                //!< Channel 5 (differential_rear) state 
            boolean mS1_prev;                           //!< Previous Channel 4 (differential_front) state 
            boolean mS2_prev;                           //!< Previous Channel 5 (differential_rear) state 

            StateMachine(); //< Constructor

            #ifdef ROS_ACTIVE
            /*! configure input */
            void configure_input( InterruptManager* interrupt_manager_in, ROSInterface<myHardware>* ros_interface_in);
            #else
            /*! configure input */
            void configure_input( InterruptManager* interrupt_manager_in);
            #endif /* ROS_ACTIVE */
            
            /*! Brief description */
            void configure_output(OutputDriverI2C* output_in);

            /*! update rc signals */
            void _update_rc_signals(util::control &signals_rc);

            /*! taehu */
            void _swl_execute_switching_logic(util::control* signals_rc);

            /*! oeoauu */
            void _swl_check_idle_transition();

            /*! oeauoeu */
            void wait_for_next_cycle();
            
            /*! thaeu */
            void critical_error();

            /*! taestestetest */
            void _process_rc_inputs(util::control &signals_rc);
            
            #ifdef ROS_ACTIVE
            /*! test ros */
            void _process_ros_inputs(util::control &signals_ros);
            #endif /* ROS_ACTIVE */

            /*! update output */
            void update_output_signals();

            #ifdef ROS_ACTIVE
            /*! expose ros */
            void _expose_actuated_signals_to_ros();
            #endif /* ROS_ACTIVE */
            
            /*! actuate */
            void actuate();
    };


}