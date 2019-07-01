#pragma once

// IDENTIFIERS - OUTPUT
#define BAUD_RATE 115200   
#define PWM_LOW 1000.0
#define PWM_NEUTRAL 1500.0
#define PWM_HIGH 2000.0
#define PWM_THRESHOLD 500.0
#define PWM_CTRL_THRESHOLD 200.0                          // margin around center value for clearly defined logic states
#define PWM_EMERGENCY_CTRL_SWITCH_LOW 1150.0              // if in CONTROL_SW mode acceleration or deceleration outside of these margins, switches immediately to CONTROL_REMOTE
#define PWM_EMERGENCY_CTRL_SWITCH_HIGH 1850.0
#define PWM_EXTERNAL_MIN_TICK 204.0 // 1ms --> 1/20 --> 0.05
#define PWM_EXTERNAL_MAX_TICK 410.0
#define PWM_EXTERNAL_RES 4096.0
//define PWM_INTERNAL2INTERAL //40,9-82

// TIMER PERIODS
#define T_pub 25  // [T_pub] = s 10E-3
#define T_ctrl 1000  // [T_pub] = s 10E-3 
#define T_idle_interrupt 3000  // [T_pub] = s 10E-3
#define T_out 25  // [T_pub] = s 10E-3
#define T_remote_idle 1000  // [T_remote_idle] = s 10E-3
#define T_sw_idle 1000  // [T_remote_idle] = s 10E-3
#define T_diag 100   // [T_diag] = s 10E-3  
#define T_sense 1000// [T_led] = s 10E-3
#define T_alarm 1000// [T_led] = s 10E-3

// IDENTIFIERS - INPUT
//adc -- voltage
#define ADC_VOLTAGE 0x10
#define ADC_BIT_RES 1024.0
#define ADC_GAIN 4.2082         // 13.3/3.3
#define ADC_REF 5.0               //
#define POWER_SUPPLY_VOLTAGE 20 // MAX ADC VALUE

//adc -- voltage
#define ADC_CURRENT 0x10

// alarm
#define ADC_PIN 0x10            // Port B PB4, di12
#define F_ALARM 4000            // Hz
#define T_ALARM1 100            // ms
#define T_ALARM2 500            // ms
#define T_ALARM3 1000           // ms
#define F_IO 16000000           // Hz
#define Prescaler 32            //
#define ALARM_TICK_COUNT 163 //                            // (10bit)
#define VALUE_IS(PINX,PINn) ((PINX&PINn)==PINn)


// BIT MASK
#define BIT0 1
#define BIT1 2
#define BIT2 4
#define BIT3 8
#define BIT4 16
#define BIT5 32
#define BIT6 64
#define BIT7 128

// INPUT PINS - DIGITAL         //           // (PCINT)      // timer        //common interrupt vecor       -- see setup() pin definition for interrupts
#define INP_STEER 17            // A3        // (11)         // ti 1         //2
#define INP_VEL 8               // PB0       // (0)          // ti 2         //1
#define INP_TRANS 7             // PD7       // (23)         // ti 3         //3
#define INP_DIFF_F 4            // PD4       // (20)         // ti 4         //3
#define INP_DIFF_R 2            // PD2       // (18)         // ti 5         //3
