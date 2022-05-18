#include "PIDController.h"

PIDController::PIDController(double Kp_, double Ki_, double Kd_) {
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
    _e_prev = 0;
    _i = 0;
}

double PIDController::calc(double x, double xd, double dt) {
    double e = xd - x;
    _i += dt * e;
    double d = (e - _e_prev) / dt;
    _e_prev = e;

    return Kp * e + Ki * _i + Kd * d;
}