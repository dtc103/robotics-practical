#ifndef MAPPING_H
#define MAPPING_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>

#include <memory>
#include <mutex>

#include "vec2.h"

class Mapping: public rclcpp::Node {
    public:
        Mapping();

    private:
        void laserCallback(const sensor_msgs::msg::LaserScan &scan);
        void odomCallback(const nav_msgs::msg::Odometry &odom);
        int get_grid_index(Vec2f robotPos);

        tf2::Transform slerpTransforms(const tf2::Transform &a, const tf2::Transform &b, double ratio);

        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr gridPublisher;

        nav_msgs::msg::OccupancyGrid grid;

        Vec2f robotPos;
        double robotYaw;

        double grid_resolution = 0.1;
        int grid_width = 100;
        int grid_height = 100; 
        bool grid_init = false;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;

        std::mutex mutex;

        std::shared_ptr<tf2_ros::Buffer> tf2Buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf2Listener;
        message_filters::Subscriber<sensor_msgs::msg::LaserScan> subLaser;
        std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> tf2MessageFilter;
};

#endif
