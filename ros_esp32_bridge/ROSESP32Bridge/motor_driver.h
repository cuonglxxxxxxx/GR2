/***************************************************************
   Motor driver function definitions - ESP32 Version
   *************************************************************/

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#ifdef L298_MOTOR_DRIVER
  // Định nghĩa chân khớp với motor_driver.ino
  #define LEFT_MOTOR_IN1  6
  #define LEFT_MOTOR_IN2  7
  #define RIGHT_MOTOR_IN3 15
  #define RIGHT_MOTOR_IN4 16
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);

#endif
