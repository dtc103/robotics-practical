#ifndef PATH_FOLLOWING_H
#define PATH_FOLLOWING_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "vec2.h"

class PathFollowing: public rclcpp::Node {
    public:

        struct ProjectionData {
            size_t segment_index;
            Vec2f projection;    // Closest point on path
            double x_n;          // Signed lateral error (right-hand rule)
            double phi_c;        // Control angle (radians)
        };


        PathFollowing();
        void process_path(const nav_msgs::msg::Path::SharedPtr msg);
        ProjectionData nearest_projection_angle(const nav_msgs::msg::Path& path, Vec2f point);
        void odomCallback(const nav_msgs::msg::Odometry &odom);

        


    private:
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr processed_path_pub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;

        Vec2f curr_pos;

        double k = 1.0;

        bool has_init_pos = false;

};

#endif