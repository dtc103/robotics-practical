#ifndef PATH_FOLLOWING_H
#define PATH_FOLLOWING_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "vec2.h"
#include "pid.h"
#include <math.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <message_filters/subscriber.h>
#include <tf2/utils.h>

class PathFollowing: public rclcpp::Node {
    public:

        struct ProjectionData {
            size_t segment_index; //index of the path index, that is cdosest
            Vec2f projection;    // Closest point on path
            double x_n;          // Signed lateral error
            double phi_c;        // Control angle (radians)
        };


        PathFollowing();
        void process_path(const nav_msgs::msg::Path::SharedPtr msg);
        ProjectionData nearest_projection_angle(const nav_msgs::msg::Path& path, Vec2f point);
        void odomCallback(const nav_msgs::msg::Odometry &odom);
        void goal_callback(const geometry_msgs::msg::PoseStamped &goal);
        double nearest_projection_angle(nav_msgs::msg::Path &path, Vec2f point);

        void move();


    private:
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr processed_path_pub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr move_cmd_pub;

        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr driven_path_pub;
        nav_msgs::msg::Path driven_path;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription;

        nav_msgs::msg::Path processed_path;

        Vec2f curr_pos;
        double robot_yaw = 0.0;

        double k = 1.0;

        bool has_init_pos = false;
        bool received_path = false;

        PID controller;

        std::shared_ptr<tf2_ros::Buffer> tf2Buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf2Listener;

        double closest_point_on_segment(Vec2f& a, Vec2f& b, Vec2f& p, Vec2f& proj);
};

#endif