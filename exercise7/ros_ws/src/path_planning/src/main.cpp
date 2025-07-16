#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "path_planning.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    std::shared_ptr<PathPlanning> node = std::make_shared<PathPlanning>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
