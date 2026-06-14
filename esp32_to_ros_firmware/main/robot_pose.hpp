#ifndef ROBOT_POSE_HPP
#define ROBOT_POSE_HPP

#include <math.h>

class UnicycleOdometry {
public:
    UnicycleOdometry(float wheel_radius, float wheel_separation)
        : _radius(wheel_radius), _separation(wheel_separation), _x(0), _y(0), _theta(0) {}

    void update(float v_lin, float v_ang, float dt) {
        _x += v_lin * cosf(_theta) * dt;
        _y += v_lin * sinf(_theta) * dt;
        _theta += v_ang * dt;
        normalizeAngle();
    }

    float getX() { return _x; }
    float getY() { return _y; }
    float getTheta() { return _theta; }
    void setTheta(float theta) { _theta = theta; normalizeAngle(); }

private:
    float _radius, _separation;
    float _x, _y, _theta;
    void normalizeAngle() {
        while (_theta > M_PI) _theta -= 2.0f * M_PI;
        while (_theta < -M_PI) _theta += 2.0f * M_PI;
    }
};

class EncoderPoseEstimator {
public:
    EncoderPoseEstimator(float wheel_radius, float wheel_separation)
        : _radius(wheel_radius), _separation(wheel_separation), _v_lin(0.0f), _v_ang(0.0f) {}

    void update(float rpm_L, float rpm_R) {
        float w_L = (rpm_L * 2.0f * M_PI) / 60.0f;
        float w_R = (rpm_R * 2.0f * M_PI) / 60.0f;
        _v_lin = _radius * (w_R + w_L) / 2.0f;
        _v_ang = _radius * (w_R - w_L) / _separation;
    }

    float getLinearVelocity() { return _v_lin; }
    float getAngularVelocity() { return _v_ang; }

private:
    float _radius, _separation;
    float _v_lin, _v_ang;
};

#endif // ROBOT_POSE_HPP
