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

#pragma once

#include "spmb.h"

namespace SPMB {
    //! Specifies the Arduino hardware limitations by assigning buffer sizes and restrictions on amount of publishers and subscribers.
    /*! Typedefinition of Arduino Serial ROS interface specifying maximum transmission and buffer allocation */
    typedef ros::NodeHandle_<ArduinoHardware, 1, 1, 100, 100> myHardware;
    
    //! Provides all necessary functinos to interface and debug various datatypes via ROS.
    /*! ROS Interface class providing interface functions and facilitating publishing and subscribing */
    // \Todo Provide interface functions for int, float, double and string to parse into char * for loginfo
    // \Todo Remove util::control mSignals and provide interface based on call-by-reference with util::control to parse ROS message
    template<typename T>
    class ROSInterface{
        public:
            ROSInterface(const char * subscriber_topic, const char * publisher_topic); //!< Constructor

            //! Callback for command request via ROS
            void cb_request_(const spmb::request& msg);
             
            //! Parsing to publish control_filtered to ROS message actuated
            void publish(util::control_filtered* data);
  
            //! Print convenience function for debugging
            void debug(const char * text);
            //! Print convenience function for debugging
            void debug(std::string text);
            //! Print convenience function for debugging
            void debug(uint8_t number);
            //! Print convenience function for debugging
            void debug(uint16_t number);
            //! Print convenience function for debugging
            void debug(long number);
            //! Print convenience function for debugging
            void debug(float number);
            //! Print convenience function for debugging
            void debug(double number);

            //! Node handle variable
            T mNh;

            //! Node handle variable
            spmb::actuated mMsgPub;

            //! Node handle variable
            spmb::request mMsgSub; 
                        
            //! Node handle variable
            util::control mSignals;

            //! Node handle variable
            boolean mIsIdle;
            //! Node handle variable
            long mTimestamp;
            //! Node handle variable
            uint16_t mMinTimePeriod;

            //! Node handle variable
            ros::Subscriber<spmb::request, ROSInterface<T>> mSubscriber;
            
            //! Node handle variable
            ros::Publisher mPublisher;
    };

    template<typename T>
    void ROSInterface<T>::cb_request_(const spmb::request& msg){
        mSignals.steering = util::pwm_to_period(msg.steering);
        mSignals.velocity = util::pwm_to_period(msg.velocity);
        mSignals.transmission = util::pwm_to_period(msg.transmission);
        mSignals.differential_front = util::pwm_to_period(msg.differential_front);
        mSignals.differential_rear = util::pwm_to_period(msg.differential_rear);
        mTimestamp = micros();
    }


    template<typename T>
    void ROSInterface<T>::debug(const char * text){
        mNh.loginfo(text);
    }    
    
    template<typename T>
    void ROSInterface<T>::debug(std::string text){
        mNh.loginfo(text.c_str());
    }
    
    template<typename T>
    void ROSInterface<T>::debug(uint8_t number){
        char buffer[10];
        itoa(number, buffer, 10);
        mNh.loginfo(buffer);
    }    

    template<typename T>
    void ROSInterface<T>::debug(uint16_t number){
        char buffer[10];
        itoa(number, buffer, 10);
        mNh.loginfo(buffer);
    }    
    
    template<typename T>
    void ROSInterface<T>::debug(long number){
        char buffer[10];
        itoa(number, buffer, 10);
        mNh.loginfo(buffer);
    }    
    
    template<typename T>
    void ROSInterface<T>::debug(float number){
        char buffer[20];
        snprintf(buffer, sizeof buffer, "%f", number);
        mNh.loginfo(buffer);
    }
    
    template<typename T>
    void ROSInterface<T>::debug(double number){
        char buffer[20];
        snprintf(buffer, sizeof buffer, "%f", number);
        mNh.loginfo(buffer);
    }


    template<typename T>
    void ROSInterface<T>::publish(util::control_filtered* data){
        mMsgPub.steering = util::period_to_pwm(data->steering.mValue);
        mMsgPub.velocity = util::period_to_pwm(data->velocity.mValue);
        mMsgPub.transmission = util::period_to_pwm(data->transmission.mValue);
        mMsgPub.differential_front = util::period_to_pwm(data->differential_front.mValue);
        mMsgPub.differential_rear = util::period_to_pwm(data->differential_rear.mValue);
        mPublisher.publish(&mMsgPub);
    }
    
    template<typename T>
    ROSInterface<T>::ROSInterface(const char * subscriber_topic, const char * publisher_topic)
        :mSubscriber(subscriber_topic, &ROSInterface<T>::cb_request_, this),
        mPublisher(publisher_topic, &mMsgPub){   
        mNh.getHardware()->setBaud(BAUD_RATE);
        mNh.initNode();
        mNh.advertise(mPublisher);
        mNh.subscribe(mSubscriber); 
        mIsIdle = true;
        mTimestamp = micros();
        mMinTimePeriod =  uint16_t(s2ms * T_ROS_MIN_RATE);
    }  
}
