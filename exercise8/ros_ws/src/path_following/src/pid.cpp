#include "pid.h"

PID::PID() : p_gain(0.0), i_gain(0.0), d_gain(0.0){
    this->prev_error = 0.0;
    this->integral = 0.0;
    this->set_point = 0.0;
}

PID::PID(double p, double i, double d, double initial_set_point): p_gain(p), i_gain(i), d_gain(d){
    this->prev_error = 0.0;
    this->integral = 0.0;
    this->set_point = initial_set_point;
}

void PID::set_p_gain(double p_gain){
    this->p_gain = p_gain;
}

void PID::set_i_gain(double i_gain){
    this->i_gain = i_gain;
}

void PID::set_d_gain(double d_gain){
    this->d_gain = d_gain;
}

double PID::update(double current_value, double current_time){
    double curr_time = current_time;
    double dt = curr_time - last_time_update;
    this->last_time_update = curr_time;

    double error = this->set_point - current_value;


    // // TODO change to rad, if necessary
    // if(error > 180.0){
    //     error -= 360.0;
    // }else if(error < -180.0){
    //     error += 360.0;
    // }

    double p_value = error;
    std::cout << "P value: " << p_value << ", ";


    this->integral += (error * dt);
    if(this->integral > (double)PID::MAX_INTEGRAL_VALUE){
        this->integral = (double)PID::MAX_INTEGRAL_VALUE;
    }else if(this->integral < (double)(-PID::MAX_INTEGRAL_VALUE)){
        this->integral = (double)(-PID::MAX_INTEGRAL_VALUE);
    }
    std::cout << "I value: " << integral << ", ";

    double derivative = (error - this->prev_error) / dt;
    this->prev_error = error;
    std::cout << "D value: " << derivative << std::endl;

    return this->p_gain * p_value + /*this->i_gain * this->integral +*/ this->d_gain * derivative;
}

void PID::new_set_point(double set_point){
    this->set_point = set_point;
}

void PID::reset(){
    this->integral = 0.0;
    this->prev_error = 0.0;
}