#include "path_following.h"
#include "path_processing.h"
#include "plot_data.h"
#include <cmath>

using std::placeholders::_1;

PathFollowing::PathFollowing(): Node("path_following"), data_writer_("/home/praktikum7/Desktop/jan/robotics-practical/exercise8/ros_ws/src/path_following/data/irl_data.txt") {
    this->declare_parameter<double>("p_gain", 5.0);
    this->declare_parameter<double>("i_gain", 0.0);
    this->declare_parameter<double>("d_gain", 0.0);
    this->declare_parameter<double>("k_gain", 3.0);

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
    (void)goal;
    this->received_path = false;
    this->driven_path.poses.clear();
}

void PathFollowing::process_path(const nav_msgs::msg::Path::SharedPtr msg){
    this->processed_path = processPath(*msg);

    this->processed_path_pub->publish(this->processed_path);


    this->received_path = true;
}

void PathFollowing::move() {
    geometry_msgs::msg::Twist twistMsg;

    if (!this->received_path) {
        // kein Pfad: stehen bleiben
        twistMsg.linear.x  = 0.0;
        twistMsg.angular.z = 0.0;
        move_cmd_pub->publish(twistMsg);
        return;
    }

    double desired_yaw = 0.0;
    if (this->has_init_pos && !this->processed_path.poses.empty()) {
        desired_yaw = nearest_projection_angle(this->processed_path, this->curr_pos);
        this->controller.new_set_point(desired_yaw);
    }

    double current_yaw = this->robot_yaw;

    // Winkel-Fehler in [–π, π] normalisieren
    double error = std::remainder(desired_yaw - current_yaw, 2.0 * M_PI);

    static double last_t = this->now().seconds();
    double t  = this->now().seconds();
    double dt = t - last_t;
    last_t     = t;

    // Drehgeschwindigkeit
    double omega = this->controller.update(error, dt);

    double v = 0.4 * std::max(1.0 - std::abs(error)/M_PI, 0.0);

    twistMsg.linear.x  = v;
    twistMsg.angular.z = omega;
    move_cmd_pub->publish(twistMsg);

    data_writer_.write(current_yaw, desired_yaw, error);

    std::cout 
      << "desired_yaw: " << desired_yaw 
      << "  current_yaw: " << current_yaw 
      << "  error: " << error 
      << "  dt: " << dt 
      << "  omega: " << omega 
      << "  v: " << v 
      << std::endl;
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


double PathFollowing::nearest_projection_angle(nav_msgs::msg::Path &path, Vec2f point)
{
    // search next segment
    double min_dist = std::numeric_limits<double>::max();
    int min_seg = 0;
    Vec2f A, B;
    for(int i = 0; i < (int)path.poses.size()-1; ++i){
        Vec2f Ai{path.poses[i].pose.position.x, path.poses[i].pose.position.y};
        Vec2f Bi{path.poses[i+1].pose.position.x, path.poses[i+1].pose.position.y};
        // project point
        Vec2f AB = Bi - Ai;
        Vec2f AP = point - Ai;
        double t = (AP.x*AB.x + AP.y*AB.y) / (AB.x*AB.x + AB.y*AB.y);
        
        t = std::clamp(t, 0.0, 1.0);
        Vec2f P = Ai + AB * t;
        // distance from point to projection
        double dist = (point - P).norm();
        if(dist < min_dist){
            min_dist = dist;
            min_seg = i;
        }
    }

    // start and end points of the best segment
    A = Vec2f{ path.poses[min_seg].pose.position.x,
               path.poses[min_seg].pose.position.y };
    B = Vec2f{ path.poses[min_seg+1].pose.position.x,
               path.poses[min_seg+1].pose.position.y };

    // tangent heading of the segment
    double segment_yaw = std::atan2(B.y - A.y, B.x - A.x);

    Vec2f AB = B - A;
    Vec2f AP = point - A;
    double t = (AP.x*AB.x + AP.y*AB.y) / (AB.x*AB.x + AB.y*AB.y);
    t = std::clamp(t, 0.0, 1.0);
    Vec2f P = A + AB * t;

    // signed cross-track error along the normal
    Vec2f normal{-AB.y, AB.x};
    normal = normal / normal.norm();  // unit normal
    double x_n = (point - P).x * normal.x + (point - P).y * normal.y;

    // stanley-correction
    double k = this->get_parameter("k_gain").as_double();
    double phi_c = std::atan(-k * x_n);

    return segment_yaw + phi_c;
}

