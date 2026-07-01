#include "robot_imu.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#define MPU9250_WHO_AM_I     0x75
#define MPU9250_PWR_MGMT_1   0x6B
#define MPU9250_ACCEL_XOUT_H 0x3B
#define MPU9250_GYRO_XOUT_H  0x43
RobotIMU::RobotIMU()
    : _dev_handle(NULL), _gz(0) {}

esp_err_t RobotIMU::read_registers(uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(_dev_handle, &reg_addr, 1, data, len, -1);
}
esp_err_t RobotIMU::write_register(uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(_dev_handle, write_buf, 2, -1);
}
void RobotIMU::calibrateGyro(int samples) {
    float sum_gz = 0;
    uint8_t data[6];
    for (int i = 0; i < samples; i++) {
        if (read_registers(MPU9250_GYRO_XOUT_H, data, 6) == ESP_OK) {
            int16_t raw_gz = (int16_t)((data[4] << 8) | data[5]);
            sum_gz += (float)raw_gz;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    _gyro_z_bias = sum_gz / (float)samples;
}
esp_err_t RobotIMU::update() {
    uint8_t data[14];
    if (read_registers(MPU9250_ACCEL_XOUT_H, data, 14) != ESP_OK) return ESP_FAIL;
    int16_t raw_gz = (int16_t)((data[12] << 8) | data[13]);
    float gz_corrected = (float)raw_gz - _gyro_z_bias;
    _gz = gz_corrected / 131.0f * (M_PI / 180.0f);
    return ESP_OK;
}
void RobotIMU::setHandle(i2c_master_dev_handle_t handle) {
    _dev_handle = handle;
    if (write_register(MPU9250_PWR_MGMT_1, 0x00) != ESP_OK) {
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
}
