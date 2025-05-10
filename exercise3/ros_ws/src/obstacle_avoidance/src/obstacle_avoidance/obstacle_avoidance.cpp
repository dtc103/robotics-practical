#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include <functional>
#include <obstacle_avoidance/obstacle_avoidance.h>
#include <optional>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/utils.h>
#include <algorithm>

using namespace std::chrono_literals;

ObstacleAvoidance::ObstacleAvoidance() : Node("obstacle_avoidance"), odomInit(false), laserInit(false) {
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
    this->declare_parameter<double>("kAtt", 0.5);
    this->declare_parameter<double>("kRep", 0.4);
    this->declare_parameter<double>("var_rep", 1.57079);
    this->declare_parameter<int>("segments", 10);
    this->declare_parameter<double>("d_thres", 0.4);
    this->declare_parameter<double>("x_goal", 0.0);
    this->declare_parameter<double>("y_goal", 0.0);
    this->declare_parameter<double>("rot_gain", 0.5);
    

    this->length = this->get_parameter("length").as_double();
    this->width = this->get_parameter("width").as_double();
    this->rot_gain = this->get_parameter("rot_gain").as_double();

    auto kAtt = this->get_parameter("kAtt").as_double();
    auto kRep = this->get_parameter("kRep").as_double();
    auto var_rep = this->get_parameter("var_rep").as_double();
    auto segments = this->get_parameter("segments").as_int();
    auto rho_0 = this->get_parameter("d_thres").as_double();

    auto x_goal = this->get_parameter("x_goal").as_double();
    auto y_goal = this->get_parameter("y_goal").as_double();
    this->goalPos = Vec2f(x_goal, y_goal);
    
    this->pf = std::make_shared<PotentialField>(this->goalPos, kAtt, kRep, var_rep, rho_0, segments);
    this->f_att_publisher = create_publisher<visualization_msgs::msg::Marker>("/att_marker", 10);
    this->f_rep_publisher = create_publisher<visualization_msgs::msg::MarkerArray>("/rep_marker", 10);

    this->monitor_zone = Vec2f(this->width, this->length);
}

void ObstacleAvoidance::odomCallback(const nav_msgs::msg::Odometry &odom)
{
    std::lock_guard<std::mutex> guard(mutex);

    robotPos = Vec2f(odom.pose.pose.position.x, odom.pose.pose.position.y);
    robotYaw = tf2::getYaw(odom.pose.pose.orientation);

    odomInit = true;
    if (odomInit && laserInit)
    {
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

    auto f_att = this->pf->get_f_att(this->robotPos);
    auto f_reps = this->pf->get_f_rep(this->robotPos, this->robotYaw, this->laserPoints);
    auto f_tot = this->pf->get_total_force(f_reps, f_att);

    this->publish_marker(f_att, 0.0, 1.0, 0.0, "potential", 0);
    this->publish_marker(f_tot, 0.0, 0.0, 1.0, "potential", 1);
    this->publish_markers(f_reps, 1.0, 0.0, 0.0, "potential");

    double distanceToGoal = (goalPos - robotPos).norm();
    if (distanceToGoal > 0.2 && !this->obstacle_detected)
    {
        //double deltaYaw = (goalPos - robotPos).angle() - robotYaw;
        double deltaYaw = f_tot.angle() - robotYaw;
        
        if (deltaYaw > std::numbers::pi)
        {
            deltaYaw -= 2 * std::numbers::pi;
        }
        else if (deltaYaw < -std::numbers::pi)
        {
            deltaYaw += 2 * std::numbers::pi;
        }

        twistMsg.linear.x = 0.3;
        twistMsg.angular.z = this->rot_gain * deltaYaw;
    }else{
        twistMsg.linear.x = 0.0;
        twistMsg.angular.z = 0.0;
    }

    pubCmdVel->publish(twistMsg);
}

void ObstacleAvoidance::publish_marker(Vec2f f_att, double r, double g, double b, std::string ns, int id){
    auto marker = this->create_marker(this->robotPos, f_att, r, g, b, ns, id);

    this->f_att_publisher->publish(marker);
}

void ObstacleAvoidance::publish_markers(std::vector<std::tuple<int, Vec2f>> f_rep, double r, double g, double b, std::string ns){
    visualization_msgs::msg::MarkerArray markers;

    for(size_t i = 0; i < f_rep.size(); ++i){
        markers.markers.push_back(this->create_marker(this->laserPoints[std::get<0>(f_rep[i])], std::get<1>(f_rep[i]), r, g, b, ns, i + 2));
    }

    this->f_rep_publisher->publish(markers);
}

visualization_msgs::msg::Marker ObstacleAvoidance::create_marker(Vec2f origin, Vec2f dir, double r, double g, double b, std::string ns, int id){
    auto marker = visualization_msgs::msg::Marker();
    marker.action = 0;
    marker.header.frame_id = "odom";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = ns;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.id = id;

    marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    geometry_msgs::msg::Point from, to;

    from.x = origin.x;
    from.y = origin.y;
    from.z = 0;

    to.x = origin.x + dir.x;
    to.y = origin.y + dir.y;
    to.z = 0;

    marker.points.push_back(from);
    marker.points.push_back(to);

    marker.scale.x = 0.1;  // shaft diameter
    marker.scale.y = 0.2;  // head diameter
    marker.scale.z = 0.2;  // head length

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    return marker;
}


