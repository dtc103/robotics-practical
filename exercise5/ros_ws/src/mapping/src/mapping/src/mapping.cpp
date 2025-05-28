#include "mapping.h"
#include <tf2/utils.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include "gridtraversal.h"

using namespace std::chrono_literals;

Mapping::Mapping(): Node("mapping") {
    // TODO: create and setup grid
    
    gridPublisher = create_publisher<nav_msgs::msg::OccupancyGrid>("/grid", 1);

    tf2Buffer = std::make_shared<tf2_ros::Buffer>(get_clock(), 10s);
    std::shared_ptr<tf2_ros::CreateTimerROS> timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(get_node_base_interface(), get_node_timers_interface());
    tf2Buffer->setCreateTimerInterface(timer_interface);
    tf2Listener = std::make_shared<tf2_ros::TransformListener>(*tf2Buffer);

    subLaser.subscribe(this, "/scan");
    tf2MessageFilter = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(subLaser, *tf2Buffer, "odom", 3, get_node_logging_interface(), get_node_clock_interface(), 500ms);
    tf2MessageFilter->setTolerance(100ms);
    tf2MessageFilter->registerCallback(&Mapping::laserCallback, this);
}

void Mapping::laserCallback(const sensor_msgs::msg::LaserScan &scan) {
    std::lock_guard<std::mutex> guard(mutex);

    // Improve performance slightly by only looking up the first and last transformation.
    // The intermediate transformations can be computed by using a spherical linear interpolation.
    tf2::Transform startTransform;
    tf2::Transform endTransform;
    tf2::convert(tf2Buffer->lookupTransform("odom", scan.header.frame_id, scan.header.stamp).transform, startTransform);
    tf2::convert(tf2Buffer->lookupTransform("odom", scan.header.frame_id, scan.header.stamp + rclcpp::Duration::from_seconds((scan.ranges.size() - 1) * scan.time_increment)).transform, endTransform);

    for (unsigned int i = 0; i < scan.ranges.size(); i++) {
        double angle = scan.angle_min + scan.angle_increment * i;

        if (std::abs(angle) > 1.6) {
            continue;
        }

        if (scan.ranges[i] < scan.range_min || scan.range_max < scan.ranges[i]) {
            continue;
        }
        
        // TODO: handle max range values

        Vec2f pLaser = scan.ranges[i] * Vec2f::fromAngle(angle);

        tf2::Transform interpolatedTransform = slerpTransforms(startTransform, endTransform, i / (scan.ranges.size() - 1.0));
        Vec2f viewpointOdom = interpolatedTransform(Vec2f(0, 0).toTf2Vector3());
        Vec2f pOdom = interpolatedTransform(pLaser.toTf2Vector3());
    }

    // TODO: traverse grid and update occupancy information

    grid.header.stamp = scan.header.stamp;
    gridPublisher->publish(grid);
}

// spherical linear interpolation, https://en.wikipedia.org/wiki/Slerp
tf2::Transform Mapping::slerpTransforms(const tf2::Transform &a, const tf2::Transform &b, double ratio) {
    tf2::Transform slerpedTransform;
    tf2::Vector3 translation;
    translation.setInterpolate3(a.getOrigin(), b.getOrigin(), ratio);
    slerpedTransform.setOrigin(translation);
    slerpedTransform.setRotation(tf2::slerp(a.getRotation(), b.getRotation(), ratio));

    return slerpedTransform;
}
