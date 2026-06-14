#include "robot_imu.hpp"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define MPU9250_WHO_AM_I     0x75
#define MPU9250_PWR_MGMT_1   0x6B
#define MPU9250_ACCEL_XOUT_H 0x3B
#define MPU9250_GYRO_XOUT_H  0x43

static const char *TAG = "ROBOT_IMU";

RobotIMU::RobotIMU(i2c_port_t i2c_port, uint8_t dev_addr)
    : _i2c_port(i2c_port), _dev_addr(dev_addr), _dev_handle(NULL),
      _ax(0), _ay(0), _az(0), _gx(0), _gy(0), _gz(0), _yaw(0) {}

esp_err_t RobotIMU::init() {
    // Note: I2C Bus is expected to be initialized before calling this
    // We use a simplified setup here. In main, we will create the bus.
    return ESP_OK; // Initialization of dev_handle will happen in setupPins
}

// Internal method to link the device handle
void set_device_handle(i2c_master_dev_handle_t handle) {
    // This will be called from main to link the master device
}

esp_err_t RobotIMU::read_registers(uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(_dev_handle, &reg_addr, 1, data, len, -1);
}

esp_err_t RobotIMU::write_register(uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(_dev_handle, write_buf, 2, -1);
}

void RobotIMU::calibrateGyro(int samples) {
    ESP_LOGI(TAG, "Calibrating Gyro... Keep robot still");
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
    ESP_LOGI(TAG, "Gyro Z Bias: %.2f", _gyro_z_bias);
}

esp_err_t RobotIMU::update() {
    uint8_t data[14];
    if (read_registers(MPU9250_ACCEL_XOUT_H, data, 14) != ESP_OK) return ESP_FAIL;

    // Convert raw values
    int16_t raw_ax = (int16_t)((data[0] << 8) | data[1]);
    int16_t raw_ay = (int16_t)((data[2] << 8) | data[3]);
    int16_t raw_az = (int16_t)((data[4] << 8) | data[5]);
    // Skip temperature (data[6], data[7])
    int16_t raw_gx = (int16_t)((data[8] << 8) | data[9]);
    int16_t raw_gy = (int16_t)((data[10] << 8) | data[11]);
    int16_t raw_gz = (int16_t)((data[12] << 8) | data[13]);

    // Apply scales (Assuming default ±2g and ±250deg/s)
    _ax = (float)raw_ax / 16384.0f * 9.81f;
    _ay = (float)raw_ay / 16384.0f * 9.81f;
    _az = (float)raw_az / 16384.0f * 9.81f;
    
    _gx = (float)raw_gx / 131.0f * (M_PI / 180.0f);
    _gy = (float)raw_gy / 131.0f * (M_PI / 180.0f);
    
    // Corrected Gyro Z with bias and convert to rad/s
    float gz_corrected = (float)raw_gz - _gyro_z_bias;
    _gz = gz_corrected / 131.0f * (M_PI / 180.0f);

    // Integrate Yaw
    int64_t now = esp_timer_get_time();
    if (_last_update_time > 0) {
        float dt = (float)(now - _last_update_time) / 1000000.0f;
        _yaw += _gz * dt;
        // Normalize angle
        while (_yaw > M_PI) _yaw -= 2.0f * M_PI;
        while (_yaw < -M_PI) _yaw += 2.0f * M_PI;
    }
    _last_update_time = now;

    return ESP_OK;
}

// Added public method to set handle correctly
void RobotIMU::setHandle(i2c_master_dev_handle_t handle) {
    _dev_handle = handle;
    
    // 1. Reset / Wake up MPU9250
    if (write_register(MPU9250_PWR_MGMT_1, 0x00) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU9250! Check wiring/power.");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // 2. Check WHO_AM_I
    uint8_t who_am_i = 0;
    if (read_registers(MPU9250_WHO_AM_I, &who_am_i, 1) == ESP_OK) {
        ESP_LOGI(TAG, "MPU9250 connection successful. WHO_AM_I: 0x%02X", who_am_i);
    } else {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I! I2C communication error.");
    }
}
