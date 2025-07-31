#include "path_following.h"
#include "path_processing.h"
#include "plot_data.h"

using std::placeholders::_1;

PathFollowing::PathFollowing(): Node("path_following") {
    this->path_sub = this->create_subscription<nav_msgs::msg::Path>("/path", 10, std::bind(&PathFollowing::process_path, this, _1));
    this->processed_path_pub = this->create_publisher<nav_msgs::msg::Path>("/processed_path", 10);

    this->curr_pos = Vec2f(0.0, 0.0);

    subOdom = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 1,
        std::bind(&PathFollowing::odomCallback, this, std::placeholders::_1));
}

void PathFollowing::process_path(const nav_msgs::msg::Path::SharedPtr msg){
    auto path = processPath(*msg);

    this->processed_path_pub->publish(path);

    std::cout << "run nearest_projections" << std::endl;
    if(has_init_pos && path.poses.size() > 0){
        auto s = nearest_projection_angle(path, this->curr_pos);
        std::cout << s.phi_c << " " << s.x_n << " " << s.segment_index << " " << s.projection.x << " " << s.projection.y << std::endl;
        std::cout << "nearest_projections did run" << std::endl;
    }
}

void PathFollowing::odomCallback(const nav_msgs::msg::Odometry &odom)
{
    this->curr_pos.x = odom.pose.pose.position.x;
    this->curr_pos.y = odom.pose.pose.position.y;

    this->has_init_pos = true;
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
        
        double ab_squared = std::pow(AB.x, 2) + std::pow(AB.y, 2);
        double t = (ab_squared > 0) ? ((AQ.x * AB.x + AQ.y * AB.y) / ab_squared) : 0.0;
        t = std::max(0.0, std::min(1.0, t));

        Vec2f proj = A + AB * t;
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