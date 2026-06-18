#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include "driver/ledc.h"

// --- MOTOR PINS ---
#define PWM_PIN_A    18  // ENB - Left Speed
#define IN_PIN_A     16  // Swapped from 15 to fix direction
#define IN_PIN_B     15  // Swapped from 16 to fix direction

#define IN_PIN_C     6   // Swapped from 7 to fix direction
#define IN_PIN_D     7   // Swapped from 6 to fix direction
#define PWM_PIN_B    8   // ENA - Right Speed

// --- ENCODER PINS ---
// #define ENCODER_PIN_A_L  4 
// #define ENCODER_PIN_B_L  5
// #define ENCODER_PIN_A_R  2
// #define ENCODER_PIN_B_R  1 
#define ENCODER_PIN_A_L  13 
#define ENCODER_PIN_B_L  14
#define ENCODER_PIN_A_R  48
#define ENCODER_PIN_B_R  47 

// --- I2C / IMU CONFIGURATION ---
#define I2C_SDA_PIN      10
#define I2C_SCL_PIN      11
#define I2C_PORT         I2C_NUM_0
#define MPU9250_ADDR     0x68


// --- PWM CONFIGURATION ---
#define PWM_FREQ        5000
#define PWM_RESOLUTION  LEDC_TIMER_12_BIT
#define PWM_TIMER       LEDC_TIMER_0
#define PWM_MODE        LEDC_LOW_SPEED_MODE
#define PWM_CHANNEL_L   LEDC_CHANNEL_0
#define PWM_CHANNEL_R   LEDC_CHANNEL_1

// --- ROBOT KINEMATICS ---
#define WHEEL_RADIUS     0.034f
#define WHEEL_SEPARATION 0.2562f
#define ENCODER_PPR      1232 // 11 pulses * 56 ratio * 2 (edge counting)

// --- CMD_VEL SCALE-BACK (phai khop voi normalized_cmd/src/main.cpp) ---
#define V_MAX            0.627f   // m/s
#define W_MAX            5.016f   // rad/s

// --- PID CONSTANTS ---
#define PID_DT          0.02f   // 50ms
#define OUT_MIN         -4095
#define OUT_MAX         4095
#define ROBOT_INT_MIN         -2000.0f
#define ROBOT_INT_MAX         2000.0f

// --- MICRO-ROS CONFIG ---
#define DOMAIN_ID       30
#define FRAME_TIME      100 // ms
#define SLEEP_TIME      10  // ms

#endif // CONFIG_H
