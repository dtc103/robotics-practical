#include "pid.h"

PID::PID()
: p_gain(0.0), i_gain(0.0), d_gain(0.0),
  prev_error(0.0), integral(0.0), set_point(0.0),
  last_time_update(0.0)
{}

PID::PID(double p, double i, double d, double initial_set_point)
: p_gain(p), i_gain(i), d_gain(d),
  prev_error(0.0), integral(0.0), set_point(initial_set_point),
  last_time_update(0.0)
{}

void PID::set_p_gain(double p) { p_gain = p; }
void PID::set_i_gain(double i) { i_gain = i; }
void PID::set_d_gain(double d) { d_gain = d; }

double PID::update(double error, double dt) {
    // 1) I-Anteil (clamped)
    integral += error * dt;
    if (integral > MAX_INTEGRAL_VALUE) integral =  MAX_INTEGRAL_VALUE;
    if (integral < -MAX_INTEGRAL_VALUE) integral = -MAX_INTEGRAL_VALUE;

    // 2) D-Anteil
    double derivative = (dt > 0.0) ? (error - prev_error)/dt : 0.0;
    prev_error = error;

    // 3) PID-Ausgang
    return p_gain * error + i_gain * integral + d_gain * derivative;
}


void PID::new_set_point(double sp) {
    set_point = sp;
}

void PID::reset() {
    integral = 0.0;
    prev_error = 0.0;
}
