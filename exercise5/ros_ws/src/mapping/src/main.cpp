#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "mapping.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    std::shared_ptr<Mapping> node = std::make_shared<Mapping>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
