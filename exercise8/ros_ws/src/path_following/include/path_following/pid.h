#ifndef PID_H
#define PID_H

#include <rclcpp/rclcpp.hpp>

class PID {
public:
    PID();
    PID(double p, double i, double d, double initial_set_point);

    void set_p_gain(double p);
    void set_i_gain(double i);
    void set_d_gain(double d);

    double update(double error, double dt);

    void new_set_point(double sp);
    void reset();

private:
    // PID-Parameter
    double p_gain;
    double i_gain;
    double d_gain;

    // interner Zustand
    double prev_error{0.0};
    double integral{0.0};
    double set_point{0.0};

    // Zeitstempel des letzten Updates (in Sekunden)
    double last_time_update{0.0};

    // Maximaler Betrag für den Integrator
    static constexpr double MAX_INTEGRAL_VALUE = 1e4;
};

#endif  // PID_H
