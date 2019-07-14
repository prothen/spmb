#pragma once

#include "spmb.h"

namespace SPMB {
   /*
    class ROSInterfaceNh{
        public:
            void cb_request_(const spmbv2::request& msg);
             
            void publish(util::control_filtered* data);
  
            util::control mSignals;

            ros::NodeHandle mNh;
            volatile spmbv2::actuated mMsgPub;
            volatile spmbv2::request mMsgSub; 
            ROSInterfaceNh(const char * subscriber_topic, const char * publisher_topic);
            
            ros::Subscriber<spmbv2::request, ROSInterfaceNh> mSubscriber;
            ros::Publisher mPublisher;
    };
    */
    
    typedef ros::NodeHandle_<ArduinoHardware, 1, 1, 240, 240> myHardware;
    
    
    template<typename T>
    class ROSInterface{
        public:
            void cb_request_(const spmbv2::request& msg);
             
            void publish(util::control_filtered* data);
  
            util::control mSignals;

            T mNh;
            spmbv2::actuated mMsgPub;
            spmbv2::request mMsgSub; 
            ROSInterface(const char * subscriber_topic, const char * publisher_topic);
            
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
    }  
}
