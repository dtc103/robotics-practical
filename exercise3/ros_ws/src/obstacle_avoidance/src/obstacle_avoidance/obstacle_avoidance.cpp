#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include <functional>
#include <obstacle_avoidance/obstacle_avoidance.h>
#include <optional>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/utils.h>
#include <algorithm>

using namespace std::chrono_literals;

ObstacleAvoidance::ObstacleAvoidance()
    : Node("obstacle_avoidance"), odomInit(false), laserInit(false),
      goalPos(20, -20)
{
    pubCmdVel = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    subOdom = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 1,
        std::bind(&ObstacleAvoidance::odomCallback, this, std::placeholders::_1));

    tf2Buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
    // Create the timer interface before call to waitForTransform, to avoid a
    // tf2_ros::CreateTimerInterfaceException exception
    std::shared_ptr<tf2_ros::CreateTimerROS> timer_interface =
        std::make_shared<tf2_ros::CreateTimerROS>(get_node_base_interface(),
                                                  get_node_timers_interface());
    tf2Buffer->setCreateTimerInterface(timer_interface);
    tf2Listener = std::make_shared<tf2_ros::TransformListener>(*tf2Buffer);

    subLaser.subscribe(this, "/base_scan");
    tf2MessageFilter =
        std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
            subLaser, *tf2Buffer, "odom", 3, get_node_logging_interface(),
            get_node_clock_interface(), 500ms);
    tf2MessageFilter->setTolerance(100ms);
    // Register a callback with tf2_ros::MessageFilter to be called when
    // transforms are available
    tf2MessageFilter->registerCallback(&ObstacleAvoidance::laserCallback, this);

    std::cout << "finished constructor" << std::endl;
}

void ObstacleAvoidance::odomCallback(const nav_msgs::msg::Odometry &odom)
{
    std::lock_guard<std::mutex> guard(mutex);

    robotPos = Vec2f(odom.pose.pose.position.x, odom.pose.pose.position.y);
    robotYaw = tf2::getYaw(odom.pose.pose.orientation);

    std::cout << "initializing odo" << std::endl;
    odomInit = true;
    if (odomInit && laserInit)
    {
        std::cout << "processing" << std::endl;
        process();
    }
}

void ObstacleAvoidance::laserCallback(const sensor_msgs::msg::LaserScan &scan)
{
    std::lock_guard<std::mutex> guard(mutex);

    std::cout << "CALLBACK CALLED" << std::endl;

    laserPoints.clear();

    for (unsigned int i = 0; i < scan.ranges.size(); i++)
    {
        if (std::isnan(scan.ranges[i]) || std::isinf(scan.ranges[i]) ||
            scan.ranges[i] < scan.range_min || scan.range_max < scan.ranges[i])
        {
            continue;
        }

        Vec2f pLaser = scan.ranges[i] *
                       Vec2f::fromAngle(scan.angle_min + scan.angle_increment * i);

        std_msgs::msg::Header header = scan.header;
        rclcpp::Time stamp(header.stamp);
        stamp += rclcpp::Duration::from_seconds(i * scan.time_increment);
        header.stamp = stamp;

        Vec2f pOdom =
            tf2Buffer->transform(pLaser.toGeometryMsgPointStamped(header), "odom");

        laserPoints.push_back(pOdom);
    }

    laserInit = true;
    if (odomInit && laserInit)
    {
        process();
    }
}

void ObstacleAvoidance::process()
{

    geometry_msgs::msg::Twist twistMsg;

    double distanceToGoal = (goalPos - robotPos).norm();
    if (distanceToGoal > 0.2)
    {
        double deltaYaw = (goalPos - robotPos).angle() - robotYaw;
        if (deltaYaw > std::numbers::pi)
        {
            deltaYaw -= 2 * std::numbers::pi;
        }
        else if (deltaYaw < -std::numbers::pi)
        {
            deltaYaw += 2 * std::numbers::pi;
        }

        twistMsg.linear.x = 0.3;
        twistMsg.angular.z = 
            std::clamp(deltaYaw, -std::numbers::pi / 8, std::numbers::pi / 8);
    }

    pubCmdVel->publish(twistMsg);
}
