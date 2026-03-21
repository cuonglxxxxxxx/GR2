/*
 * Author: Automatic Addison (Modified for ESP32)
 * Website: https://automaticaddison.com
 * Description: Đếm số xung encoder trên một vòng quay sử dụng ESP32.
 */

// Chọn chân GPIO phù hợp trên ESP32 (Ví dụ GPIO 18, 19, 21... tránh các chân chỉ Input như 34-39)
#define ENC_IN_RIGHT_A 10

// Sử dụng volatile để biến được cập nhật chính xác trong trình phục vụ ngắt
// Dùng IRAM_ATTR để đặt hàm ngắt vào RAM, giúp ESP32 phản hồi nhanh hơn
volatile long right_wheel_pulse_count = 0;

void IRAM_ATTR right_wheel_pulse() {
  right_wheel_pulse_count++;
}

void setup() {
  // ESP32 thường dùng baudrate cao hơn, nhưng 9600 vẫn hoạt động tốt
  Serial.begin(115200); 

  // Cấu hình chân encoder
  pinMode(ENC_IN_RIGHT_A, INPUT_PULLUP);
 
  // Thiết lập ngắt: ESP32 không cần hàm digitalPinToInterrupt() nhưng dùng vẫn an toàn
  // Ta dùng chế độ RISING (cạnh lên) giống code gốc của bạn
  attachInterrupt(ENC_IN_RIGHT_A, right_wheel_pulse, RISING);
}

void loop() {
  // In giá trị xung ra Serial Monitor mỗi 100ms để tránh làm nghẽn Buffer
  static unsigned long lastTime = 0;
  if (millis() - lastTime > 100) {
    Serial.print("Pulses: ");
    Serial.println(right_wheel_pulse_count);
    lastTime = millis();
  }
}