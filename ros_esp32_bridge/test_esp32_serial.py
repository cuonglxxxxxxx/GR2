import serial
import time

# Cấu hình cổng Serial
# Thay '/dev/ttyACM0' bằng cổng thực tế của bạn (ví dụ 'COM3' trên Windows)
SERIAL_PORT = '/dev/ttyACM0' 
BAUD_RATE = 57600
TIMEOUT = 0.1

try:
    # Khởi tạo kết nối
    esp32 = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=TIMEOUT)
    time.sleep(2) # Đợi ESP32 khởi động lại sau khi kết nối
    print(f"--- Đã kết nối với ESP32 tại {SERIAL_PORT} ---")

    def send_command(cmd_string):
        """Gửi lệnh kèm theo ký tự Carriage Return (\r)"""
        full_cmd = cmd_string + '\r'
        esp32.write(full_cmd.encode())
        time.sleep(0.05)
        # Đọc phản hồi nếu có
        response = esp32.read_all().decode().strip()
        if response:
            print(f"ESP32 phản hồi: {response}")
        return response

    # 1. Reset Encoder
    print("\n1. Đang Reset Encoder...")
    send_command('r')

    # 2. Chạy thử Motor bằng PWM (Raw PWM)
    # Lệnh 'o <pwm_trái> <pwm_phải>' (Giá trị từ -255 đến 255)
    print("\n2. Đang chạy Motor tiến (PWM 150)...")
    send_command('o 150 150')
    time.sleep(2)

    # 3. Đọc giá trị Encoder
    # Lệnh 'e' trả về: <ticks_trái> <ticks_phải>
    print("\n3. Đọc giá trị Encoder hiện tại:")
    send_command('e')

    # 4. Dừng Motor
    print("\n4. Đang dừng Motor...")
    send_command('o 0 0')

    # 5. Thử nghiệm vòng kín (PID) - Tốc độ theo ticks/vòng lặp
    # Lệnh 'm <tốc_độ_trái> <tốc_độ_phải>'
    print("\n5. Chạy thử bằng PID (Tốc độ 20 ticks/loop)...")
    send_command('m 20 20')
    time.sleep(2)
    send_command('e')
    send_command('m 0 0')

    print("\n--- Hoàn tất thử nghiệm ---")

except serial.SerialException as e:
    print(f"Lỗi kết nối Serial: {e}")
except KeyboardInterrupt:
    print("\nĐã dừng bởi người dùng.")
finally:
    if 'esp32' in locals() and esp32.is_open:
        esp32.close()
        print("Đã đóng kết nối Serial.")
