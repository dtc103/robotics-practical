#ifndef POTENTIAL_FIELD_H
#define POTENTIAL_FIELD_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <message_filters/subscriber.h>

#include "vec2.h"

class PotentialField{
    public:
        PotentialField(Vec2f goal_position, double k_att, double k_rep, double var_rep, double rho_0, size_t segments);
        Vec2f get_f_att(Vec2f current_position);
        std::vector<Vec2f> get_f_rep(Vec2f current_pos, double curr_yaw, std::vector<Vec2f> laser_positions);
        Vec2f get_total_force(std::vector<Vec2f> f_reps, Vec2f f_att);

    private:
        double rotational_scaling(double angle);

        Vec2f goal_position;
        double k_att;
        double k_rep;
        double var_rep;
        double rho_0;
        int segments;
};


#endif