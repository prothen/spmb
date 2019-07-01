#pragma once

//! Parameter definition for build process

/*!
   DEFINE MACROS
*/
// FLASH CONFIGURATION                  // Serial print-outs INACTIVE, ACTIVE [0,1]
#define ACTUATE_EXTERNAL 0              // Use PCA9685 - external timer chip (i2c)
#define ROS_ACTIVE 0                    // include ros publisher in build
#define MEASUREMENT 0                   // Activate current and voltage measurement
#define ALARM_ACTIVE 0                  // Activate Alarm with Buzzer in case of low voltage

#define DIAGNOSIS 1                     // Allow serial-prints (only in combination with ROS_ACTIVE 0)
#define PRINT_BITS 0                    // Output some register information (see print function)
#define PRINT_CTRL_MODE 0               // Visualise the current control mode
#define PRINT_INTERRUPT_MODE 1          // Visualise the current control mode
#define PRINT_CTRL 0                    // Print logic for control bit switching
#define PRINT_INPUT_PERIODS 0           // Show the measured input time periods and their channel dependent idle state
#define PRINT_INPUT_PPMS 0              // Show the PPM values for each channel based on the input period
#define PRINT_OUTPUT 0                  // Show the PPM values for each channel based on the input period
#define PRINT_MEASUREMENT 0             // Print Measurement data
#define PRINT_FILTER 0                  // Print Moving average debug data
