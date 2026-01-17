/***************************************************************
   Motor driver definitions for L298N with ESP32
   Adapted for ROSArduinoBridge
 ***************************************************************/

#ifdef L298_MOTOR_DRIVER
  // Define pins based on user configuration
  #define LEFT_MOTOR_IN1  4
  #define LEFT_MOTOR_IN2  5
  #define RIGHT_MOTOR_IN3 6
  #define RIGHT_MOTOR_IN4 7

  void initMotorController() {
    pinMode(LEFT_MOTOR_IN1, OUTPUT);
    pinMode(LEFT_MOTOR_IN2, OUTPUT);
    pinMode(RIGHT_MOTOR_IN3, OUTPUT);
    pinMode(RIGHT_MOTOR_IN4, OUTPUT);
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