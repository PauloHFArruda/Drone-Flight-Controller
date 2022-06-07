#include "PIDController.h"

PIDController::PIDController(float Kp_, float Ki_, float Kd_) {
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
    _e_prev = 0;
    _i = 0;
}

float PIDController::calc(float x, float xd, float dt) {
    float e = xd - x;
    _i += dt * e;
    float d = (e - _e_prev) / dt;
    _e_prev = e;

    return Kp * e + Ki * _i + Kd * d;
}