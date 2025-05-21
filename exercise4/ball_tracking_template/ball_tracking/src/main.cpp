#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "ball_tracker.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    std::shared_ptr<BallTracker> node = std::make_shared<BallTracker>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
