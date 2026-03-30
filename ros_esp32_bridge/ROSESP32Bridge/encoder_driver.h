/* *************************************************************
   Encoder driver function definitions - ESP32-S3 Version
   ************************************************************ */
   
#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#ifdef ARDUINO_ENC_COUNTER
  // Giữ nguyên chân bạn đang dùng cho ESP32-S3
  #define LEFT_ENC_A_PIN  4 
  #define LEFT_ENC_B_PIN  5

  #define RIGHT_ENC_A_PIN 1
  #define RIGHT_ENC_B_PIN 2
#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
void initEncoders();

#endif
