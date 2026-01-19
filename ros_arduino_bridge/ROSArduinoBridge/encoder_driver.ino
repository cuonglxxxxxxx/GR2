/* ******************************************************************** */
/* Encoder driver function definitions - COPIED FROM WORKING TEST CODE  */
/* ******************************************************************** */

#ifdef ARDUINO_ENC_COUNTER
  
  // Cấu hình chân y hệt file test_esp32.ino
  // Lưu ý: Đảo 18, 17 để sửa chiều quay như bạn muốn
  #define LEFT_ENC_A_PIN  18 
  #define LEFT_ENC_B_PIN  17
  
  #define RIGHT_ENC_A_PIN 8
  #define RIGHT_ENC_B_PIN 3
  
  volatile long left_enc_pos = 0;
  volatile long right_enc_pos = 0;

  // Logic ngắt y hệt file test
  void IRAM_ATTR doLeftEnc() {
    if (digitalRead(LEFT_ENC_B_PIN) == digitalRead(LEFT_ENC_A_PIN)) {
      left_enc_pos++;
    } else {
      left_enc_pos--;
    }
  }

  void IRAM_ATTR doRightEnc() {
    if (digitalRead(RIGHT_ENC_B_PIN) == digitalRead(RIGHT_ENC_A_PIN)) {
      right_enc_pos++;
    } else {
      right_enc_pos--;
    }
  }

  // Khởi tạo Encoder
  void initEncoders() {
      pinMode(LEFT_ENC_A_PIN, INPUT_PULLUP);
      pinMode(LEFT_ENC_B_PIN, INPUT_PULLUP);
      pinMode(RIGHT_ENC_A_PIN, INPUT_PULLUP);
      pinMode(RIGHT_ENC_B_PIN, INPUT_PULLUP);

      // Gắn ngắt CHANGE y hệt file test
      attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A_PIN), doLeftEnc, CHANGE);
      attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A_PIN), doRightEnc, CHANGE);
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
