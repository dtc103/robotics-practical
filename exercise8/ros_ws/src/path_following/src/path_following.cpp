#include "path_following.h"
#include "path_processing.h"
#include "plot_data.h"

using std::placeholders::_1;

PathFollowing::PathFollowing(): Node("path_following") {
    this->path_sub = this->create_subscription<nav_msgs::msg::Path>("/path", 10, std::bind(&PathFollowing::process_path, this, _1));
    this->processed_path_pub = this->create_publisher<nav_msgs::msg::Path>("/processed_path", 10);
}

void PathFollowing::process_path(const nav_msgs::msg::Path msg){
        auto path = processPath(msg);

        this->processed_path_pub->publish(path);
}

size_t PathFollowing::nearest_projection_angle(const nav_msgs::msg::Path& path, Vec2f point, double k)
{
    double min_dist = std::numeric_limits<double>::max();
    Vec2f closest_projection{0, 0};
    size_t closest_segment = 0;
    double best_xn = 0.0;
    double best_phi_c = 0.0;

    for (size_t i = 0; i < path.poses.size() - 1; ++i) {
        const auto& poseA = path.poses[i].pose.position;
        const auto& poseB = path.poses[i+1].pose.position;

        // Segment vector
        double dx = poseB.x - poseA.x;
        double dy = poseB.y - poseA.y;

        // Vector from A to query point
        double px = point.x - poseA.x;
        double py = point.y - poseA.y;

        double ab_squared = dx * dx + dy * dy;
        double t = (ab_squared > 0) ? ((px * dx + py * dy) / ab_squared) : 0.0;
        t = std::max(0.0, std::min(1.0, t)); // Clamp t to [0,1]

        // Projection coordinates
        Vec2f proj;
        proj.x = poseA.x + t * dx;
        proj.y = poseA.y + t * dy;

        double dist = std::hypot(point.x - proj.x, point.y - proj.y);

        // Unit tangent (tx, ty)
        double seg_len = std::sqrt(ab_squared);
        double tx = (seg_len > 1e-9) ? dx / seg_len : 1.0;
        double ty = (seg_len > 1e-9) ? dy / seg_len : 0.0;

        // Unit normal (right-hand rule)
        double nx = ty;
        double ny = -tx;

        // Vector from projection to robot
        double rx = point.x - proj.x;
        double ry = point.y - proj.y;

        // Signed lateral error (xn): projection of (R - T) onto normal
        double x_n = rx * nx + ry * ny;

        // Control angle
        double phi_c = std::atan(-k * x_n);

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