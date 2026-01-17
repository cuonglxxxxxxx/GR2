/***************************************************************
   Encoder driver definitions for ESP32
   Using interrupts (attachInterrupt)
 ***************************************************************/

#ifdef ARDUINO_ENC_COUNTER
  
  // Left Encoder Pins (User provided)
  #define LEFT_ENC_A_PIN  1 
  #define LEFT_ENC_B_PIN  2
  
  // Right Encoder Pins (DEFAULT - PLEASE UPDATE IF DIFFERENT)
  #define RIGHT_ENC_A_PIN 41
  #define RIGHT_ENC_B_PIN 42
  
  volatile long left_enc_pos = 0;
  volatile long right_enc_pos = 0;

  // Interrupt Service Routines (IRAM_ATTR is required for ESP32 ISRs)
  void IRAM_ATTR doLeftEncA() {
    if (digitalRead(LEFT_ENC_B_PIN) == digitalRead(LEFT_ENC_A_PIN)) {
      left_enc_pos++;
    } else {
      left_enc_pos--;
    }
  }
  
  void IRAM_ATTR doLeftEncB() {
    if (digitalRead(LEFT_ENC_A_PIN) == digitalRead(LEFT_ENC_B_PIN)) {
        left_enc_pos++;
    } else {
        left_enc_pos--;
    }
  }

  void IRAM_ATTR doRightEncA() {
    if (digitalRead(RIGHT_ENC_B_PIN) == digitalRead(RIGHT_ENC_A_PIN)) {
      right_enc_pos++;
    } else {
      right_enc_pos--;
    }
  }

  void IRAM_ATTR doRightEncB() {
    if (digitalRead(RIGHT_ENC_A_PIN) == digitalRead(RIGHT_ENC_B_PIN)) {
        right_enc_pos++;
    } else {
        right_enc_pos--;
    }
  }

  // Initialization function
  void initEncoders() {
      pinMode(LEFT_ENC_A_PIN, INPUT_PULLUP);
      pinMode(LEFT_ENC_B_PIN, INPUT_PULLUP);
      pinMode(RIGHT_ENC_A_PIN, INPUT_PULLUP);
      pinMode(RIGHT_ENC_B_PIN, INPUT_PULLUP);

      attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A_PIN), doLeftEncA, CHANGE);
      attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B_PIN), doLeftEncB, CHANGE);
      
      attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A_PIN), doRightEncA, CHANGE);
      attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B_PIN), doRightEncB, CHANGE);
  }
  
  long readEncoder(int i) {
    if (i == LEFT) return left_enc_pos;
    else return right_enc_pos;
  }

  void resetEncoders() {
    left_enc_pos = 0;
    right_enc_pos = 0;
  }
#endif