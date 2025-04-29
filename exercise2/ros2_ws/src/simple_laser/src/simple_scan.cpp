#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <online_statistics/online_statistics.h>


// class SimpleScan : public rclcpp::Node
// {
// public:
//     SimpleScan() : Node("simple_scan")
//     {
//         std::cout << "Cerated sub" << std::endl;
//         subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//             "/scan", 10, std::bind(&SimpleScan::scan_callback, this, std::placeholders::_1));
//         std::cout << "Cerated sub" << std::endl;

//     }
// private:

//     void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
//     {
//         int num_measurements = msg->ranges.size();
//         float angle_increment = msg->angle_increment;
//         float first_ray_angle = msg->angle_min;
//         float last_ray_angle = msg->angle_max;

//         RCLCPP_INFO(this->get_logger(), "Number of distance measurements per scan: %d", num_measurements);
//         RCLCPP_INFO(this->get_logger(), "Angle difference btw two subsequent rays: %f", angle_increment);
//         RCLCPP_INFO(this->get_logger(), "Orientation of the first ray: %f", first_ray_angle);
//         RCLCPP_INFO(this->get_logger(), "Orientation of the last ray: %f", last_ray_angle);

//     }


//     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
// };

class SimpleScan : public rclcpp::Node
{
public:
    SimpleScan() : Node("simple_scan"), forward_ray_index_(0)
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&SimpleScan::scan_callback, this, std::placeholders::_1));
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan &msg)
    {
        float range_min = msg.range_min;
        float range_max = msg.range_max;


        // float first_ray_angle = msg->angle_min;
        // float last_ray_angle = msg->angle_max;
        // float angle_increment = msg->angle_increment;

        // RCLCPP_INFO(this->get_logger(), "first Ray angle: %.5f rad", first_ray_angle);
        // RCLCPP_INFO(this->get_logger(), "last Ray angle: %.5f rad", last_ray_angle);
        // RCLCPP_INFO(this->get_logger(), "angle increment: %.5f rad", angle_increment);

        forward_ray_index_ = msg.ranges.size() / 2;

        double forward_ray_distance = msg.ranges[388];

        if (std::isnan(forward_ray_distance) || std::isinf(forward_ray_distance) || forward_ray_distance < range_min || forward_ray_distance > range_max) {
            //RCLCPP_WARN(this->get_logger(), "Invalid distance measurement: %.4f", forward_ray_distance);
            std::cout << "nan" << std::endl;
            return;
        }
        statistics_.update(forward_ray_distance);

        RCLCPP_INFO(this->get_logger(), "forward Ray Distance: %.5f m", forward_ray_distance);
        RCLCPP_INFO(this->get_logger(), "mean Distance: %.5f m", statistics_.getMean());
        RCLCPP_INFO(this->get_logger(), "standard Deviation: %.5f m", statistics_.getStandardDeviation());
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    OnlineStatistics statistics_;
    size_t forward_ray_index_;
};




int main(int argc, char **argv)
{
    (void) argc;
    (void) argv;
    rclcpp::init(argc, argv);

    std::cout << "After init" << std::endl;
    
    //auto node = std::make_shared<SimpleScan>();
    std::cout << "consttructor" << std::endl;

    rclcpp::spin(std::make_shared<SimpleScan>());
    std::cout << "After spin" << std::endl;
    rclcpp::shutdown();
    return 0;
}



