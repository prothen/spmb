#include "spmb.h"

namespace SPMB{
    /*
    void ROSInterfaceNh::cb_request_(const spmbv2::request& msg){
        mSignals.steering = util::pwm_to_period(msg.steering);
        mSignals.velocity = util::pwm_to_period(msg.velocity);
        mSignals.transmission = util::pwm_to_period(msg.transmission);
        mSignals.differential_front = util::pwm_to_period(msg.differential_front);
        mSignals.differential_rear = util::pwm_to_period(msg.differential_rear);
    }

    
    void ROSInterfaceNh::publish(util::control_filtered* data){
        mMsgPub.steering = util::period_to_pwm(data->steering.mValue);
        mMsgPub.velocity = util::period_to_pwm(data->velocity.mValue);
        mMsgPub.transmission = util::period_to_pwm(data->transmission.mValue);
        mMsgPub.differential_front = util::period_to_pwm(data->differential_front.mValue);
        mMsgPub.differential_rear = util::period_to_pwm(data->differential_rear.mValue);
        mPublisher.publish(&mMsgPub);
    }
    

    ROSInterfaceNh::ROSInterfaceNh(const char * subscriber_topic, const char * publisher_topic)
        :mSubscriber(subscriber_topic, &ROSInterfaceNh::cb_request_, this),
        mPublisher(publisher_topic, &mMsgPub){
        mNh.getHardware()->setBaud(BAUD_RATE);
        mNh.initNode();
        mNh.advertise(mPublisher);
        mNh.subscribe(mSubscriber); 
    }
    */
}