#ifndef ROBOT_ENCODER_HPP
#define ROBOT_ENCODER_HPP
#include "driver/pulse_cnt.h"
#include "config.h"
class RobotEncoder {
public:
    RobotEncoder(int pin_a, int pin_b, pcnt_unit_handle_t* handle, float gear_ratio = 46.8f);
    void init();

    float getRPM(float dt_s); 
private:
    int _pin_a, _pin_b;
    pcnt_unit_handle_t* _handle;

    float _gear_ratio;
};
#endif 
