#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

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

#include <vector>
#include <memory>
#include <mutex>

#include "vec2.h"

#include <potential_field/potential_field.h>

class ObstacleAvoidance: public rclcpp::Node {
    public:
        ObstacleAvoidance();

    private:
        void odomCallback(const nav_msgs::msg::Odometry &odom);
        void laserCallback(const sensor_msgs::msg::LaserScan &scan);
        void check_save_zone();
        void process();
        void visualizeMarkers();
        void publish_marker(Vec2f f_att, double r, double g, double b, std::string ns);
        void publish_markers(std::vector<Vec2f> f_rep, double r, double g, double b, std::string ns);

        std::mutex mutex;
        bool odomInit;
        bool laserInit;

        std::shared_ptr<PotentialField> pf;
        double kAtt;
        double kRep;
        int segments;

        Vec2f robotPos;
        double robotYaw;
        Vec2f goalPos;
        std::vector<Vec2f> laserPoints;

        double width;
        double length;

        bool obstacle_detected = false;
        Vec2<double> monitor_zone = Vec2<double>(2.0, 2.0);

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pubCmdVel;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;

        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr f_att_publisher;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr f_rep_publisher;

        std::shared_ptr<tf2_ros::Buffer> tf2Buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf2Listener;
        message_filters::Subscriber<sensor_msgs::msg::LaserScan> subLaser;
        std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> tf2MessageFilter;
};

#endif
