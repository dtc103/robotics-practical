#include "path_following.h"


using namespace std::chrono_literals;

using std::placeholders::_1;

PathFollowing::PathFollowing(): Node("path_following"), data_writer_("/home/praktikum7/Desktop/backup/robotics-practical/exercise8/ros_ws/src/path_following/data/irl_data.txt") {
    // Path-Following
    p_gain_ = this->declare_parameter<double>("p_gain", 5.0);
    i_gain_ = this->declare_parameter<double>("i_gain", 0.0);
    d_gain_ = this->declare_parameter<double>("d_gain", 0.0);
    k_gain_ = this->declare_parameter<double>("k_gain", 3.0);


    // Obstacle Avoidance
    length_ = this->declare_parameter<double>("length", 1.0);
    width_ = this->declare_parameter<double>("width",  1.0);
    near_deadband_ = this->declare_parameter<double>("near_deadband", 0.10);
    min_obstacle_points_ = this->declare_parameter<int>   ("min_obstacle_points", 5);
    rotate_duration_ = this->declare_parameter<double>("rotate_duration", 1.5);
    reverse_duration_ = this->declare_parameter<double>("reverse_duration", 0.5);
    omega_avoid_ = this->declare_parameter<double>("omega_avoid", 0.5);
    omega_max_ = this->declare_parameter<double>("omega_max", 1.0);

    state_start_time_    = this->now();
    state_              = State::NORMAL;
    consecutive_hits_ = consecutive_misses_ = 0;

    // Velocity & Acceleration
    v_max_ = this->declare_parameter<double>("v_max", 0.6);
    a_max_ = this->declare_parameter<double>("a_max", 0.2);
    omega_slow_ = this->declare_parameter<double>("omega_slow",  1.0);
    decel_distance_ = this->declare_parameter<double>("decel_distance", 0.5);
  

    // publisher
    this->move_cmd_pub = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    this->processed_path_pub = this->create_publisher<nav_msgs::msg::Path>("/processed_path", 10);
    this->driven_path_pub = this->create_publisher<nav_msgs::msg::Path>("/driven_path", 10);

    // subscriber 
    this->path_sub = this->create_subscription<nav_msgs::msg::Path>("/path", 10, std::bind(&PathFollowing::process_path, this, _1));
    this->goal_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&PathFollowing::goal_callback, this, _1));

    tf2Buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
                            get_node_base_interface(), get_node_timers_interface());
    tf2Buffer->setCreateTimerInterface(timer_interface);
    tf2Listener = std::make_shared<tf2_ros::TransformListener>(*tf2Buffer);

    subLaser.subscribe(this, "/scan");
    tf2MessageFilter = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
                        subLaser, *tf2Buffer, "odom", 3,
                        get_node_logging_interface(), get_node_clock_interface(), 100ms);
    tf2MessageFilter->registerCallback(&PathFollowing::laserCallback, this);



    this->curr_pos = Vec2f(0.0, 0.0);

    subOdom = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 1,
        std::bind(&PathFollowing::odomCallback, this, std::placeholders::_1));

    this->driven_path.header.frame_id = "odom";

    this->controller = PID(
        p_gain_,
        i_gain_,
        d_gain_,
        0.0
    );
}

void PathFollowing::laserCallback(const sensor_msgs::msg::LaserScan &scan)
{
  laserPoints.clear();
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    auto r = scan.ranges[i];
    if (std::isfinite(r) && r >= scan.range_min && r <= scan.range_max) {
      // points in laser frame
      Vec2f pLaser = Vec2f::fromAngle(scan.angle_min + i * scan.angle_increment) * r;
      // transform in odom frame
      auto stamped = pLaser.toGeometryMsgPointStamped(scan.header);
      auto pOdom = tf2Buffer->transform(stamped, "odom");
      laserPoints.push_back(Vec2f(pOdom.point.x, pOdom.point.y));
    }
  }
  laserInit_ = true;
}


void PathFollowing::goal_callback(const geometry_msgs::msg::PoseStamped &goal)
{
    (void)goal;
    this->received_path = false;
    this->driven_path.poses.clear();
}

void PathFollowing::process_path(const nav_msgs::msg::Path::SharedPtr msg){
    this->processed_path = processPath(*msg);
    if(msg->poses.size() > 0){
      this->processed_path_pub->publish(this->processed_path);

      this->received_path = true;
    }
    else{
      this->received_path = false;
    }
}

void PathFollowing::move() {
    geometry_msgs::msg::Twist twistMsg;

    if (!received_path || !laserInit_ || !has_init_pos) {
        // no path was found 
        // TODO
        twistMsg.linear.x  = 0.0;
        twistMsg.angular.z = 0.0;
        move_cmd_pub->publish(twistMsg);
        return;
    }

    // check save zone only in normal drive mode
    check_save_zone();
    if (shouldStartAvoidance()) {
      startReversing();
      return;
    }

    // Normal -> Obstacle detected -> Reverse -> Rotating -> Normal
    switch(state_) {
      case State::REVERSING: {
        handleReversing();
        return;
      }
  
      case State::ROTATING: {
        handleRotating();
        return;
      }
  
      case State::NORMAL:
      default:
        handleNormalDriving();
        return;
    }
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
    double phi_c = std::atan(-k_gain_ * x_n);

    return segment_yaw + phi_c;
}


void PathFollowing::check_save_zone()
{
  // points < deadband is ignored (propably reflection on the ground)
  const double far_limit = length_ + near_deadband_;
  int count = 0;

  for (auto &p : laserPoints) {
    auto rel = (p - curr_pos).rotated(-robot_yaw);

    // count points in danger rectangel [near_deadband_, near_deadband_+length_] × [–width_/2, width_/2]
    if (rel.x > near_deadband_ 
     && rel.x <  far_limit
     && std::abs(rel.y) < width_/2.0)
    {
      ++count;
    }
  }

  // enough points -> Hit, else Miss
  if (count >= min_obstacle_points_) {
    ++consecutive_hits_;
    consecutive_misses_ = 0;
  } else {
    ++consecutive_misses_;
    consecutive_hits_  = 0;
  }

  // 2 Hit -> stop
  if (consecutive_hits_ >= 2) {
    obstacle_detected_ = true;
  }
  // 2 Miss -> continue
  else if (consecutive_misses_ >= 2) {
    obstacle_detected_ = false;
  }
}

bool PathFollowing::shouldStartAvoidance() const {
  return consecutive_hits_ >= 2;
}

// Start the reversing 
void PathFollowing::startReversing() {
  state_ = State::REVERSING;
  state_start_time_ = now();
  geometry_msgs::msg::Twist twist;
  twist.linear.x = -0.2;
  move_cmd_pub->publish(twist);
}

void PathFollowing::handleReversing() {
  double elapsed = (now() - state_start_time_).seconds();
  geometry_msgs::msg::Twist twist;
  if (elapsed < reverse_duration_) {
      twist.linear.x = -0.2;
  } else {
      state_ = State::ROTATING;
      state_start_time_ = now();
      twist.angular.z = omega_avoid_;
  }
  move_cmd_pub->publish(twist);
}


void PathFollowing::handleRotating() {
  double elapsed = (now() - state_start_time_).seconds();
  geometry_msgs::msg::Twist twist;

  // stop any movement
  twist.linear.x = 0.0;

  // first half of rotate_duration_ -> rotate left 
  if (elapsed < 0.5 * rotate_duration_) {
    twist.angular.z =  omega_avoid_;
  }
  // second half of rotate_duration_ -> rotate right 
  else if (elapsed < 1.3 * rotate_duration_) {
    twist.angular.z = -omega_avoid_;
  }
  else {
      state_ = State::NORMAL;
      consecutive_hits_ = consecutive_misses_ = 0;
      twist = geometry_msgs::msg::Twist();  // Stop drehen
  }
  move_cmd_pub->publish(twist);
}

// Starts deceleration if distance to goal is smaller then decel_distance_
void PathFollowing::applyDeceleration(double &v) {
  const auto &end = processed_path.poses.back().pose.position;
  double dx = end.x - curr_pos.x;
  double dy = end.y - curr_pos.y;
  double dist = std::hypot(dx, dy);

  if (dist < decel_distance_) {
      // s = 1/2 * a * t², v = a * t => v = sqrt(2 * a * s)
      double v_lim = std::sqrt(2.0 * a_max_ * dist);
      v = std::min(v, v_lim);
  }
  if (dist < 0.05) {
       // completly stop when distance smaller then 5 cm 
       v = 0.0;
  }
}


double PathFollowing::computeDeltaTime() {
  static double last_t = now().seconds();
    double now_s = now().seconds();
    double dt  = now_s - last_t;
    last_t = now_s;
    return dt;
}



double PathFollowing::computeRotationalVelocity(double error, double dt) {
  double raw = controller.update(error, dt);
  return std::clamp(raw, -omega_max_, omega_max_);
}

double PathFollowing::computeLinearVelocity(double omega, double dt) {
  double abs_w = std::abs(omega);
  double x = std::min(abs_w / omega_slow_, 1.0);
  double factor = 0.5 * (1.0 + std::cos(M_PI * x));
  double v_des = v_max_ * factor;
  
  // limit acceleration
  double dv_max = a_max_ * dt;
  double dv = std::clamp(v_des - prev_v_, -dv_max, dv_max);
  prev_v_ += dv;
  return prev_v_;
}


void PathFollowing::handleNormalDriving() {
  double desired_yaw = 0.0;
    if (this->has_init_pos && !this->processed_path.poses.empty()) {
        desired_yaw = nearest_projection_angle(this->processed_path, this->curr_pos);
        this->controller.new_set_point(desired_yaw);
    }

    double current_yaw = this->robot_yaw;

    // angle error in normalized in [–pi, pi]
    double error = std::remainder(desired_yaw - current_yaw, 2.0 * M_PI);

    // compute time since last iteration
    double dt = computeDeltaTime();

    // compute rotational and linear velocities (omega, v)
    double omega = computeRotationalVelocity(error, dt);
    double v = computeLinearVelocity(omega, dt);

    // smooth stopping if distance to goal < decel_distance
    applyDeceleration(v);

    // publish calculated velocities
    geometry_msgs::msg::Twist twistMsg;
    twistMsg.linear.x  = v;
    twistMsg.angular.z = omega;
    move_cmd_pub->publish(twistMsg);

    // record data
    data_writer_.write(current_yaw, desired_yaw, error);
}