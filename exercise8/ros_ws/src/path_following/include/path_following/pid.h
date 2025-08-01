#ifndef PID_H
#define PID_H

#include <rclcpp/rclcpp.hpp>

class PID{
    static const int MAX_INTEGRAL_VALUE = 10000;
    
    public:
        PID();
        PID(double, double, double, double);

        void set_p_gain(double);
        void set_i_gain(double);
        void set_d_gain(double);
        
        double update(double, double current_time);
        
        void new_set_point(double);
        void reset();

    private:
        double prev_error;
        double set_point;
        double integral;

        double p_gain;
        double i_gain;
        double d_gain;

        double last_time_update = 0.0;
};

#endif