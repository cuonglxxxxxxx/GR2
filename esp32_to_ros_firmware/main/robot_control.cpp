#include "robot_control.hpp"
#include "esp_log.h"

// --- RobotPID Implementation ---
RobotPID::RobotPID(float Kp, float Ki, float Kd, float dt,
                   float integratorMin, float integratorMax,
                   int outputMin, int outputMax)
    : _Kp(Kp), _Ki(Ki), _Kd(Kd), _dt(dt),
      _integrator(0), _prevMeas(0), _dFiltered(0),
      _intMin(integratorMin), _intMax(integratorMax),
      _outMin(outputMin), _outMax(outputMax) {}

int RobotPID::compute(float setpoint, float measurement) {
    float error = setpoint - measurement;
    float P = _Kp * error;

    // Derivative
    float dMeas = (measurement - _prevMeas) / _dt;
    _dFiltered = _dFilterAlpha * dMeas + (1.0f - _dFilterAlpha) * _dFiltered;
    float D = -_Kd * _dFiltered;

    // Integral with anti-windup
    float I_temp = _integrator + _Ki * error * _dt;
    _integrator = constrain(I_temp, _intMin, _intMax);

    float pid_control = P + _integrator + D;
    float ff_control = _useFeedforward ? (_ffA * setpoint + _ffB) : 0.0f;

    int raw = (int)round(pid_control + ff_control);

    if (raw > 0) raw += _deadZone;
    else if (raw < 0) raw -= _deadZone;

    _prevMeas = measurement;
    return constrain(raw, _outMin, _outMax);
}

void RobotPID::reset() {
    _integrator = 0;
    _prevMeas = 0;
    _dFiltered = 0;
}

void RobotPID::setTunings(float Kp, float Ki, float Kd) {
    _Kp = Kp; _Ki = Ki; _Kd = Kd;
}

void RobotPID::setDeadZone(int dz) {
    _deadZone = abs(dz);
}

void RobotPID::setFeedforwardParams(float a, float b) {
    _ffA = a; _ffB = b;
}

// --- RobotMotor Implementation ---
RobotMotor::RobotMotor(int pwm_pin, int in1_pin, int in2_pin, ledc_channel_t channel)
    : _pwm_pin(pwm_pin), _in1_pin(in1_pin), _in2_pin(in2_pin), _channel(channel) {}

void RobotMotor::init() {
    // Configure Direction Pins
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << _in1_pin) | (1ULL << _in2_pin);
    gpio_config(&io_conf);

    // Configure PWM Channel
    ledc_channel_config_t ledc_conf = {};
    ledc_conf.speed_mode = PWM_MODE;
    ledc_conf.channel = _channel;
    ledc_conf.timer_sel = PWM_TIMER;
    ledc_conf.intr_type = LEDC_INTR_DISABLE;
    ledc_conf.gpio_num = _pwm_pin;
    ledc_conf.duty = 0;
    ledc_conf.hpoint = 0;
    ledc_channel_config(&ledc_conf);

    stop();
}

void RobotMotor::setSpeed(int speed) {
    if (speed > 0) {
        gpio_set_level((gpio_num_t)_in1_pin, 1);
        gpio_set_level((gpio_num_t)_in2_pin, 0);
    } else if (speed < 0) {
        gpio_set_level((gpio_num_t)_in1_pin, 0);
        gpio_set_level((gpio_num_t)_in2_pin, 1);
    } else {
        gpio_set_level((gpio_num_t)_in1_pin, 0);
        gpio_set_level((gpio_num_t)_in2_pin, 0);
    }
    ledc_set_duty(PWM_MODE, _channel, abs(speed));
    ledc_update_duty(PWM_MODE, _channel);
}

void RobotMotor::stop() {
    // Coast mode: ENA=L → motor terminals floating → khong skid khi cmd dot ngot ve 0.
    // Active brake gay wheel slip lon trong teleop click-once → odom drift → RTAB-Map loop closure fail.
    // Nav2 precision OK voi controller_freq 20Hz + rotate_vel 0.4 rad/s + stateful False (overshoot per cycle ~1 deg)
    gpio_set_level((gpio_num_t)_in1_pin, 0);
    gpio_set_level((gpio_num_t)_in2_pin, 0);
    ledc_set_duty(PWM_MODE, _channel, 0);
    ledc_update_duty(PWM_MODE, _channel);
}
