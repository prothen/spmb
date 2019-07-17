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

//! Parameter definition for build process

/*! DEFINE MACROS */
// FLASH CONFIGURATION                      // Comment out to deactivate 
#define ROS_ACTIVE true                     // Use ROS Interface for communication --> requires ndef of DIAGNOSIS (comment out '#define DIAGNOSIS true'!)
//#define DIAGNOSIS true                    // Allow serial-prints (only with commented out '#define ROS_ACTIVE true' )
// -------------------------------------------------------------------------------

// CONTROL SIGNALS
#define DEFAULT_PWM_STEERING 1500
#define DEFAULT_PWM_VELOCITY 1500
#define DEFAULT_PWM_TRANSMISSION 1000
#define DEFAULT_PWM_DIFFERENTIAL_FRONT 1000
#define DEFAULT_PWM_DIFFERENTIAL_REAR 2000

// OUTPUT FILTER CONSTANT
#define LP_T_STEERING .06f
#define LP_T_VELOCITY .06f
#define LP_T_TRANSMISSION .15f
#define LP_T_DIFFERENTIAL_FRONT .15f
#define LP_T_DIFFERENTIAL_REAR .15f

// OUTPUT DRIVER CONFIGURATION
#define M_OUT_STEERING 0
#define M_OUT_VELOCITY 1
#define M_OUT_TRANSMISSION 4
#define M_OUT_DIFFERENTIAL_FRONT 5
#define M_OUT_DIFFERENTIAL_REAR 6

// INPUT ROS CONFIGURATION
#define T_ROS_MIN_RATE 0.1f // minimum 10Hz 

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

// TIMER PERIODS
#define T_LOOP_RATE (1.f / 50.f)  // in seconds
#define T_CONTROL_LOOP_RATE (1.f / 50.f)  // in seconds

// SAFETY PERIODS
#define T_interrupt_error_switch_off .5f  // in seconds
#define T_interrupt_error_switch_on 2.f  // in seconds
#define n_interrupt_error_switch_off uint16_t(T_interrupt_error_switch_off / T_LOOP_RATE)
#define n_interrupt_error_switch_on uint16_t(T_interrupt_error_switch_on / T_LOOP_RATE )

// SWITCHING LOGIC \Todo move to a enumeration in state machine
#define SWL_TIME_PERIOD_RESET 0.5f // in seconds


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