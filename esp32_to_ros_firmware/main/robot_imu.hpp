#ifndef ROBOT_IMU_HPP
#define ROBOT_IMU_HPP
#include "driver/i2c_master.h"
#include <math.h>
class RobotIMU {
public:
    RobotIMU();

    esp_err_t update();
    void setHandle(i2c_master_dev_handle_t handle);
    void calibrateGyro(int samples = 500);
    float getGyroZ() const { return _gz; }
private:
    i2c_master_dev_handle_t _dev_handle;
    float _gz;
    float _gyro_z_bias = 0;
    esp_err_t read_registers(uint8_t reg_addr, uint8_t *data, size_t len);
    esp_err_t write_register(uint8_t reg_addr, uint8_t data);
};
#endif 
