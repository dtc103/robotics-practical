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
      goalPos(14, 0)
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

    this->declare_parameter<double>("length", 1.0);
    this->declare_parameter<double>("width", 1.0);
    this->declare_parameter<double>("kAtt", 1.0);
    this->declare_parameter<double>("kRep", 1.0);
    this->declare_parameter<int>("segments", 1);
    

    this->length = this->get_parameter("length").as_double();
    this->width = this->get_parameter("width").as_double();
    this->kAtt = this->get_parameter("kAtt").as_double();
    this->kRep = this->get_parameter("kRep").as_double();
    this->segments = this->get_parameter("segments").as_int();
    
    this->pf = std::make_shared<PotentialField>(this->goalPos, this->kAtt, this->kRep, this->segments);
}

void ObstacleAvoidance::odomCallback(const nav_msgs::msg::Odometry &odom)
{
    std::lock_guard<std::mutex> guard(mutex);

    robotPos = Vec2f(odom.pose.pose.position.x, odom.pose.pose.position.y);
    robotYaw = tf2::getYaw(odom.pose.pose.orientation);

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

void ObstacleAvoidance::check_save_zone(){
    
    for(auto laserpoint : this->laserPoints){
        auto laser_vec = (laserpoint - this->robotPos).rotated(-this->robotYaw);

        if(laser_vec.x > 0 && laser_vec.x < this->length && std::abs(laser_vec.y) < this->width / 2){
            this->obstacle_detected = true;
            return;
        }
    }
    this->obstacle_detected = false;

}

void ObstacleAvoidance::process()
{
    geometry_msgs::msg::Twist twistMsg;

    check_save_zone();




    double distanceToGoal = (goalPos - robotPos).norm();
    if (distanceToGoal > 0.2 && !this->obstacle_detected)
    {
        double deltaYaw = (goalPos - robotPos).angle() - robotYaw;
        RCLCPP_INFO(this->get_logger(), "Goal Pos: (%.2f, %.2f); Robot Pos: (%.2f, %.2f); Delta Yaw: %.2f; Robot Yaw: %.2f"
            , goalPos.x, goalPos.y, robotPos.x, robotPos.y, deltaYaw, robotYaw);
        
        if (deltaYaw > std::numbers::pi)
        {
            deltaYaw -= 2 * std::numbers::pi;
        }
        else if (deltaYaw < -std::numbers::pi)
        {
            deltaYaw += 2 * std::numbers::pi;
        }

        RCLCPP_INFO(this->get_logger(), "%.2f", deltaYaw);
        double k = 0.5;
        twistMsg.linear.x = 0.3;
        twistMsg.angular.z = k * deltaYaw;
    }else{
        twistMsg.linear.x = 0.0;
        twistMsg.angular.z = 0.0;
    }

    pubCmdVel->publish(twistMsg);
}
