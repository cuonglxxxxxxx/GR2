#ifndef ROBOT_CONTROL_HPP
#define ROBOT_CONTROL_HPP

#include "config.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include <math.h>

// Macros for compatibility
#ifndef constrain
#define constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

// --- PID Class Ported from Arduino ---
class RobotPID {
public:
    RobotPID(float Kp, float Ki, float Kd, float dt,
             float integratorMin, float integratorMax,
             int outputMin, int outputMax);

    int compute(float setpoint, float measurement);
    void reset();
    void setTunings(float Kp, float Ki, float Kd);
    void setDeadZone(int dz);
    void setFeedforwardParams(float a, float b);

private:
    float _Kp, _Ki, _Kd, _dt;
    float _integrator, _prevMeas;
    float _dFiltered = 0.0f;
    float _dFilterAlpha = 0.1f;
    float _intMin, _intMax;
    int _outMin, _outMax;
    int _deadZone = 0;
    float _ffA = 0.0f, _ffB = 0.0f;
};

// --- MotorDriver Class Ported from Arduino ---
class RobotMotor {
public:
    RobotMotor(int pwm_pin, int in1_pin, int in2_pin, ledc_channel_t channel);
    void init();
    void setSpeed(int speed); // Speed from -4095 to 4095
    void stop();

private:
    int _pwm_pin, _in1_pin, _in2_pin;
    ledc_channel_t _channel;
};

#endif // ROBOT_CONTROL_HPP
