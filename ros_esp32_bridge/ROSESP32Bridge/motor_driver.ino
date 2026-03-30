/***************************************************************
   Motor driver definitions for L298N with ESP32
   Matched to NEW User Wiring:
   Left Motor  -> OUT1/OUT2 (IN1:6, IN2:7)
   Right Motor -> OUT3/OUT4 (IN3:15, IN4:16)
 ***************************************************************/

#include "motor_driver.h"

#ifdef L298_MOTOR_DRIVER
  void initMotorController() {
    pinMode(LEFT_MOTOR_IN1, OUTPUT);
    pinMode(LEFT_MOTOR_IN2, OUTPUT);
    pinMode(RIGHT_MOTOR_IN3, OUTPUT);
    pinMode(RIGHT_MOTOR_IN4, OUTPUT);
    
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    digitalWrite(RIGHT_MOTOR_IN3, LOW);
    digitalWrite(RIGHT_MOTOR_IN4, LOW);
  }

  void setMotorSpeed(int i, int spd) {
    int pin1, pin2;
    
    if (i == LEFT) {
        pin1 = LEFT_MOTOR_IN1;
        pin2 = LEFT_MOTOR_IN2;
    } else {
        pin1 = RIGHT_MOTOR_IN3;
        pin2 = RIGHT_MOTOR_IN4;
    }

    if (spd == 0) {
      analogWrite(pin1, 0);
      analogWrite(pin2, 0);
    } 
    else if (spd > 0) {
      analogWrite(pin1, spd);
      analogWrite(pin2, 0);
    } 
    else {
      analogWrite(pin1, 0);
      analogWrite(pin2, -spd);
    }
  }

  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#endif
