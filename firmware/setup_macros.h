#pragma once

//! Parameter definition for build process

/*!
   DEFINE MACROS
*/
// FLASH CONFIGURATION                  // Serial print-outs INACTIVE, ACTIVE [0,1]
//#define ROS_ACTIVE true                   // include ros publisher in build
#define DIAGNOSIS true                     // Allow serial-prints (only in combination with ROS_ACTIVE 0)
// -------------------------------------------------------------------------------

// CONTROL SIGNALS
#define DEFAULT_PWM_STEERING 1500
#define DEFAULT_PWM_VELOCITY 1500
#define DEFAULT_PWM_TRANSMISSION 1000
#define DEFAULT_PWM_DIFFERENTIAL_FRONT 1000
#define DEFAULT_PWM_DIFFERENTIAL_REAR 2000

// OUTPUT FILTER CONSTANT
#define LP_T_STEERING .2f
#define LP_T_VELOCITY .2f
#define LP_T_TRANSMISSION .1f
#define LP_T_DIFFERENTIAL_FRONT .1f
#define LP_T_DIFFERENTIAL_REAR .1f

// IDENTIFIERS - OUTPUT
#define BAUD_RATE 57600   
#define PWM_LOW 1000
#define PWM_NEUTRAL 1500
#define PWM_HIGH 2000
#define PWM_TH 100
#define PWM_LOW_TH PWM_LOW - PWM_TH
#define PWM_HIGH_TH PWM_HIGH + PWM_TH

#define PWM_CLEAR_TH 250
#define PWM_CLEAR_TH_LOW PWM_LOW + PWM_CLEAR_TH
#define PWM_CLEAR_TH_HIGH PWM_HIGH - PWM_CLEAR_TH

#define PWM_EXTERNAL_MIN_TICK 204.0  // 1ms --> 1/20 --> 0.05
#define PWM_EXTERNAL_MAX_TICK 410.0
#define PWM_EXTERNAL_RES 4096.0

// TIMER PERIODS
#define T_LOOP_RATE (1.f / 10.f)  // in seconds
#define T_CONTROL_LOOP_RATE (1.f / 30.f)  // in seconds

// SAFETY PERIODS
#define T_interrupt_error_switch_off 1.f  // in seconds
#define T_interrupt_error_switch_on 5.f  // in seconds
#define n_interrupt_error_switch_off uint8_t(T_interrupt_error_switch_off / T_LOOP_RATE)
#define n_interrupt_error_switch_on uint8_t(T_interrupt_error_switch_on / T_LOOP_RATE )

// SWITCHING LOGIC
#define SWL_TIME_PERIOD_RESET 0.5f // in seconds
#define SWL_OFF 0
#define SWL_ARM 1
#define SWL_SWITCH 2

// IDENTIFIERS - TIME
#define s2ms 1.e3
#define s2us 1.e6
#define ms2s 1.e-3
#define us2s 1.e-6
#define ms2us 1.e3
#define us2ms 1.e-3

// BIT MASK
#define BIT0 1
#define BIT1 2
#define BIT2 4
#define BIT3 8
#define BIT4 16
#define BIT5 32
#define BIT6 64
#define BIT7 128