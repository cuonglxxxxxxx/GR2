/* ******************************************************************** */
/* Encoder driver for ESP32-S3 N16R8 - FINAL STABLE PINS               */
/* ******************************************************************** */
#ifdef ARDUINO_ENC_COUNTER

  // CHÂN AN TOÀN TUYỆT ĐỐI (Dựa trên danh sách I/O usable của bạn)
  #define LEFT_ENC_A_PIN  14 
  #define LEFT_ENC_B_PIN  15

  #define RIGHT_ENC_A_PIN 16
  #define RIGHT_ENC_B_PIN 8

  volatile long left_enc_pos = 0;
  volatile long right_enc_pos = 0;

  void IRAM_ATTR doLeftEnc() {
    if (digitalRead(LEFT_ENC_B_PIN)) left_enc_pos++;
    else left_enc_pos--;
  }

  void IRAM_ATTR doRightEnc() {
    if (digitalRead(RIGHT_ENC_B_PIN)) right_enc_pos++;
    else right_enc_pos--;
  }

  void initEncoders() {
      pinMode(LEFT_ENC_A_PIN, INPUT_PULLUP);
      pinMode(LEFT_ENC_B_PIN, INPUT_PULLUP);
      pinMode(RIGHT_ENC_A_PIN, INPUT_PULLUP);
      pinMode(RIGHT_ENC_B_PIN, INPUT_PULLUP);

      attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A_PIN), doLeftEnc, RISING);
      attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A_PIN), doRightEnc, RISING);
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
