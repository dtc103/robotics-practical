#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>

#include <memory>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <vector>
#include <queue>

#include "vec2.h"

class PathPlanning: public rclcpp::Node {
    public:
        PathPlanning();

    private:
        void grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        void plan_route(Vec2f start, Vec2f goal);
        void publish_cost_map();
        std::vector<int> astar(int  start_idx, int goal_idx, int width, int height);
        int get_grid_index(Vec2f &p_world);
        Vec2i odom_to_grid(Vec2f world);
        Vec2f grid_coords(int idx);
        Vec2f gridIndexToWorld(int idx);
        void create_cost_map();
        double manhatten_distance(int idx, int goal, int width);
        double shapley_distance(int a, int b, int width);
        void odomCallback(const nav_msgs::msg::Odometry &odom);
        void goal_callback(const geometry_msgs::msg::PoseStamped &goal);
        bool is_line_free_with_clearance(int start_idx, int goal_idx, const nav_msgs::msg::OccupancyGrid &grid, const std::vector<float> &dist2, double min_clearance, double resolution, int threshold);
        
        std::vector<float> computeDistanceMap();


        rclcpp::TimerBase::SharedPtr timer;
        void timer_callback();


        Vec2f goal_position;
        bool goal_active_ = false;

        Vec2f current_position_;
        bool odom_received_ = false;


        std::vector<float> dist2_;
        
        
        
        double min_clearance;
        double threshold;
        double inflation_radius_;
        double sigma;
        std::vector<double> cost_map_;

        

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalSubscription;

        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr gridSubscription;
        nav_msgs::msg::OccupancyGrid grid;

        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPublisher;
        nav_msgs::msg::Path path;

        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costMapPublisher;
        nav_msgs::msg::OccupancyGrid cost_grid;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;
        std::mutex mutex;
        std::shared_ptr<tf2_ros::Buffer> tf2Buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf2Listener;
        message_filters::Subscriber<sensor_msgs::msg::LaserScan> subLaser;
        std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> tf2MessageFilter;

    public:
        struct Node {
            int idx; // index in 1D grid
            double f;   // f = g + h
            double g;   // cost from start
        };

        struct Compare {
            bool operator()(const Node &a, const Node &b) const {
                return a.f > b.f;
            }
        };
};

#endif