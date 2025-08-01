#include "path_following.h"
#include "path_processing.h"
#include "plot_data.h"

using std::placeholders::_1;

PathFollowing::PathFollowing(): Node("path_following") {
    this->declare_parameter<double>("p_gain", 5.0);
    this->declare_parameter<double>("i_gain", 0.0);
    this->declare_parameter<double>("d_gain", 0.0);

    this->move_cmd_pub = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    this->path_sub = this->create_subscription<nav_msgs::msg::Path>("/path", 10, std::bind(&PathFollowing::process_path, this, _1));
    this->processed_path_pub = this->create_publisher<nav_msgs::msg::Path>("/processed_path", 10);
    this->driven_path_pub = this->create_publisher<nav_msgs::msg::Path>("/driven_path", 10);

    this->curr_pos = Vec2f(0.0, 0.0);

    subOdom = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 1,
        std::bind(&PathFollowing::odomCallback, this, std::placeholders::_1));

    
    tf2Buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
    // Create the timer interface before call to waitForTransform, to avoid a
    // tf2_ros::CreateTimerInterfaceException exception
    std::shared_ptr<tf2_ros::CreateTimerROS> timer_interface =
        std::make_shared<tf2_ros::CreateTimerROS>(get_node_base_interface(),
                                                  get_node_timers_interface());
    tf2Buffer->setCreateTimerInterface(timer_interface);
    tf2Listener = std::make_shared<tf2_ros::TransformListener>(*tf2Buffer);

    this->goal_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&PathFollowing::goal_callback, this, _1));

    this->driven_path.header.frame_id = "odom";

    this->controller = PID(
        this->get_parameter("p_gain").as_double(), 
        this->get_parameter("i_gain").as_double(),
        this->get_parameter("d_gain").as_double(),
        0.0
    );
}

void PathFollowing::goal_callback(const geometry_msgs::msg::PoseStamped &goal)
{
    this->received_path = false;
    this->driven_path.poses.clear();
}

void PathFollowing::process_path(const nav_msgs::msg::Path::SharedPtr msg){
    auto path = processPath(*msg);

    this->processed_path_pub->publish(path);

    std::cout << "run nearest_projections" << std::endl;
    if(has_init_pos && path.poses.size() > 0){
        auto s = nearest_projection_angle(path, this->curr_pos);
        this->controller.new_set_point(-s.phi_c);
        std::cout << "Set Point: " << -s.phi_c << std::endl;
    }

    this->received_path = true;
}

void PathFollowing::move(){
    geometry_msgs::msg::Twist twistMsg;

    if(this->received_path){

        rclcpp::Time now = this->now();
        double error = this->controller.update(this->robot_yaw, now.seconds());
    
        // if (error > std::numbers::pi)
        // {
        //     error -= 2 * std::numbers::pi;
        // }
        // else if (error < -std::numbers::pi)
        // {
        //     error += 2 * std::numbers::pi;
        // }
    
        twistMsg.linear.x = 0.1;
        twistMsg.angular.z = error;

        std::cout << "ROB YAW: " << this->robot_yaw << ", PID error: " << error << std::endl;
    }else{
        twistMsg.linear.x = 0.0;
        twistMsg.angular.z = 0.0;
    }
    this->move_cmd_pub->publish(twistMsg);
}

void PathFollowing::odomCallback(const nav_msgs::msg::Odometry &odom)
{
    this->curr_pos.x = odom.pose.pose.position.x;
    this->curr_pos.y = odom.pose.pose.position.y;

    this->has_init_pos = true;

    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = this->curr_pos.x;
    pose.pose.position.y = this->curr_pos.y;

    pose.pose.orientation.w = 0;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 1;

    this->driven_path.poses.push_back(pose);

    this->driven_path_pub->publish(this->driven_path);

    this->robot_yaw = tf2::getYaw(odom.pose.pose.orientation);

    move();
}

PathFollowing::ProjectionData PathFollowing::nearest_projection_angle(const nav_msgs::msg::Path &path, Vec2f point)
{
    double min_dist = std::numeric_limits<double>::max();
    Vec2f closest_projection{0, 0};
    size_t closest_segment = 0;
    double best_xn = 0.0;
    double best_phi_c = 0.0;

    for (size_t i = 0; i < path.poses.size() - 1; ++i) {
        Vec2f A(path.poses[i].pose.position.x, path.poses[i].pose.position.y);
        Vec2f B(path.poses[i+1].pose.position.x, path.poses[i+1].pose.position.y);

        Vec2f AB = B - A;
        Vec2f AQ = point - A;
        
        double dist_aq = std::sqrt(std::pow(AQ.x, 2) + std::pow(AQ.y, 2));
        //double t = (AQ.x * AB.x + AQ.y * AB.y) / ab_squared;
        //t = std::max(0.0, std::min(1.0, t));

        //Vec2f proj = A + AB * t;
        double dist = (point - proj).norm();

        // Unit tangent
        double seg_len = std::sqrt(ab_squared);
        double tx = (seg_len > 1e-9) ? AB.x / seg_len : 1.0;
        double ty = (seg_len > 1e-9) ? AB.y / seg_len : 0.0;

        // Unit normal (right-hand rule)
        double nx = ty;
        double ny = -tx;

        Vec2f R_T = point - proj;
        double x_n = R_T.x * nx + R_T.y * ny;
        double phi_c = std::atan(-(this->k) * x_n);

        if (dist < min_dist) {
            min_dist = dist;
            closest_projection = proj;
            closest_segment = i;
            best_xn = x_n;
            best_phi_c = phi_c;
        }
    }

    return ProjectionData{closest_segment, closest_projection, best_xn, best_phi_c};
}