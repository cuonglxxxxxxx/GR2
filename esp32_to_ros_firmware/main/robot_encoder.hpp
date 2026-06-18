#ifndef ROBOT_ENCODER_HPP
#define ROBOT_ENCODER_HPP

#include "driver/pulse_cnt.h"
#include "config.h"

class RobotEncoder {
public:
    RobotEncoder(int pin_a, int pin_b, pcnt_unit_handle_t* handle, float gear_ratio = 46.8f);
    void init();
    int64_t getCount();           // [TC-ENC-01] cumulative count — consumes pcnt, accumulates _total_count
    void resetCount();            // [TC-ENC-01] zero accumulator + pcnt
    float getRPM(float dt_s); // Get RPM based on count difference since last call

private:
    int _pin_a, _pin_b;
    pcnt_unit_handle_t* _handle;
    int64_t _last_count = 0;
    int64_t _total_count = 0;     // [TC-ENC-01]
    float _gear_ratio;
};

#endif // ROBOT_ENCODER_HPP
