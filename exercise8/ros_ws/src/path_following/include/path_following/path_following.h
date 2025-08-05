#ifndef PATH_FOLLOWING_H
#define PATH_FOLLOWING_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "vec2.h"
#include "pid.h"
#include <math.h>
#include <cmath>
#include <chrono>
#include "path_processing.h"
#include "plot_data.h"

#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <message_filters/subscriber.h>
#include <tf2/utils.h>
#include "write_plot_data.hpp"
#include <std_msgs/msg/bool.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>


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
        // subscriber
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bumper_sub_;

        // publisher
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr processed_path_pub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr move_cmd_pub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr driven_path_pub;

        // path
        nav_msgs::msg::Path driven_path;
        nav_msgs::msg::Path processed_path;

        write_plot_data::PlotDataWriter data_writer_;



        // basics
        Vec2f curr_pos;
        double robot_yaw = 0.0;

        bool has_init_pos = false;
        bool received_path = false;

        PID controller;
        double closest_point_on_segment(Vec2f& a, Vec2f& b, Vec2f& p, Vec2f& proj);
        std::shared_ptr<tf2_ros::Buffer> tf2Buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf2Listener;

        // laser
        std::vector<Vec2f> laserPoints;
        bool laserInit_ = false;
        message_filters::Subscriber<sensor_msgs::msg::LaserScan> subLaser;
        std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> tf2MessageFilter;

        void laserCallback(const sensor_msgs::msg::LaserScan &scan);


        // path following
        double p_gain_, i_gain_, d_gain_, k_gain_;


        // obstacle avoidance
        rclcpp::Time state_start_time_;
        rclcpp::Time rotate_start_time_;

        // drive modes
        enum class State { NORMAL, REVERSING, ROTATING };
        State state_ = State::NORMAL;

        double reverse_duration_;
      
        bool obstacle_detected_ = false;
        bool   rotating_ = false;

        double length_, width_;
        int    min_obstacle_points_;   
        double near_deadband_;         
        int    consecutive_hits_;      
        int    consecutive_misses_;   
        double omega_avoid_;
        double omega_max_;
        double rotate_duration_;

        void check_save_zone();

        bool shouldStartAvoidance() const;
        void startReversing();
        void handleReversing();
        void handleRotating();
        void handleNormalDriving();

        double computeDeltaTime();
        double computeRotationalVelocity(double error, double dt);
        double computeLinearVelocity(double omega, double dt);
        void applyDeceleration(double &v);


        // velocity & accceleration
        double v_max_, a_max_;
        double omega_slow_; 
        double decel_distance_;

        double prev_v_ = 0.0;
};

#endif