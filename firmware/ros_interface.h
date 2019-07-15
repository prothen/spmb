#pragma once

#include "spmb.h"

namespace SPMB {
    typedef ros::NodeHandle_<ArduinoHardware, 1, 1, 100, 100> myHardware;
    
    
    template<typename T>
    class ROSInterface{
        public:

            ROSInterface(const char * subscriber_topic, const char * publisher_topic);

            void cb_request_(const spmbv2::request& msg);
             
            void publish(util::control_filtered* data);
  
            void debug(const char * text);

            T mNh;
            spmbv2::actuated mMsgPub;
            spmbv2::request mMsgSub; 
                        
            util::control mSignals;

            boolean mIsIdle;
            long mTimestamp;
            uint16_t mMinTimePeriod;

            ros::Subscriber<spmbv2::request, ROSInterface<T>> mSubscriber;
            ros::Publisher mPublisher;
    };

    template<typename T>
    void ROSInterface<T>::cb_request_(const spmbv2::request& msg){
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
