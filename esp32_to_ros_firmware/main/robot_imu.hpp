#ifndef ROBOT_IMU_HPP
#define ROBOT_IMU_HPP

#include "driver/i2c_master.h"
#include "esp_log.h"
#include <math.h>

class RobotIMU {
public:
    RobotIMU(i2c_port_t i2c_port, uint8_t dev_addr);
    esp_err_t init();
    esp_err_t update();
    
    void setHandle(i2c_master_dev_handle_t handle);
    void calibrateGyro(int samples = 500);
    
    float getAccelX() const { return _ax; }
    float getAccelY() const { return _ay; }
    float getAccelZ() const { return _az; }
    float getGyroX() const { return _gx; }
    float getGyroY() const { return _gy; }
    float getGyroZ() const { return _gz; }
    float getYaw() const { return _yaw; }

private:
    i2c_port_t _i2c_port;
    uint8_t _dev_addr;
    i2c_master_dev_handle_t _dev_handle;

    float _ax, _ay, _az;
    float _gx, _gy, _gz;
    float _gyro_z_bias = 0;
    float _yaw = 0;
    int64_t _last_update_time = 0;

    esp_err_t read_registers(uint8_t reg_addr, uint8_t *data, size_t len);
    esp_err_t write_register(uint8_t reg_addr, uint8_t data);
};

#endif // ROBOT_IMU_HPP
