#ifndef CONFIG_H
#define CONFIG_H
#include <stdint.h>
#include "driver/ledc.h"
#define PWM_PIN_A    18  
#define IN_PIN_A     16  
#define IN_PIN_B     15  
#define IN_PIN_C     6   
#define IN_PIN_D     7   
#define PWM_PIN_B    8   
#define ENCODER_PIN_A_L  13 
#define ENCODER_PIN_B_L  14
#define ENCODER_PIN_A_R  48
#define ENCODER_PIN_B_R  47 
#define I2C_SDA_PIN      10
#define I2C_SCL_PIN      11
#define I2C_PORT         I2C_NUM_0
#define MPU9250_ADDR     0x68
#define PWM_FREQ        5000
#define PWM_RESOLUTION  LEDC_TIMER_12_BIT
#define PWM_TIMER       LEDC_TIMER_0
#define PWM_MODE        LEDC_LOW_SPEED_MODE
#define PWM_CHANNEL_L   LEDC_CHANNEL_0
#define PWM_CHANNEL_R   LEDC_CHANNEL_1
#define WHEEL_RADIUS     0.034f
#define WHEEL_SEPARATION 0.2562f
#define ENCODER_PPR      1232 
#define V_MAX            0.627f   
#define W_MAX            5.016f   
#define PID_DT          0.02f   
#define OUT_MIN         -4095
#define OUT_MAX         4095
#define ROBOT_INT_MIN         -2000.0f
#define ROBOT_INT_MAX         2000.0f
#define DOMAIN_ID       30

#endif 
