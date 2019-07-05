#pragma once

#include "spmb.h"


//! Parameter definition for build process

/*!
   DEFINE MACROS
*/
// FLASH CONFIGURATION                  // Serial print-outs INACTIVE, ACTIVE [0,1]
#define ROS_ACTIVE 1                    // include ros publisher in build
#define MEASUREMENT 0                   // Activate current and voltage measurement
#define DIAGNOSIS 1                     // Allow serial-prints (only in combination with ROS_ACTIVE 0)
// -------------------------------------------------------------------------------

#define s2ms 1.e3
#define s2us 1.e6
#define ms2s 1.e-3
#define us2s 1.e-6
#define ms2us 1.e3
#define us2ms 1.e-3

// IDENTIFIERS - OUTPUT
#define BAUD_RATE 115200   
#define PWM_LOW 1000
#define PWM_NEUTRAL 1500
#define PWM_HIGH 2000
#define PWM_LOW_TH 900
#define PWM_HIGH_TH 2100

#define PWM_EXTERNAL_MIN_TICK 204.0  // 1ms --> 1/20 --> 0.05
#define PWM_EXTERNAL_MAX_TICK 410.0
#define PWM_EXTERNAL_RES 4096.0

// TIMER PERIODS
#define T_loop_rate (1.f / 20.f)  // in seconds

// SAFETY PERIODS
#define T_interrupt_error_switch_off .5f  // in seconds
#define T_interrupt_error_switch_on 5.f  // in seconds
#define n_interrupt_error_switch_off uint8_t(T_interrupt_error_switch_off / T_loop_rate)
#define n_interrupt_error_switch_on uint8_t(T_interrupt_error_switch_on / T_loop_rate )

// BIT MASK
#define BIT0 1
#define BIT1 2
#define BIT2 4
#define BIT3 8
#define BIT4 16
#define BIT5 32
#define BIT6 64
#define BIT7 128