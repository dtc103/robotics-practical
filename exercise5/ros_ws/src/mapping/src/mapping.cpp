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
#include <algorithm>

using namespace std::chrono_literals;

Mapping::Mapping(): Node("mapping") {
    // TODO: create and setup grid
    this->declare_parameter<int>("grid_height", 100);
    this->declare_parameter<int>("grid_width", 100);
    this->declare_parameter<double>("grid_resolution", 0.1);
    this->declare_parameter<double>("p_occ", 0.8);
    this->declare_parameter<double>("p_free", 0.45);

    this->grid_height = this->get_parameter("grid_height").as_int();
    this->grid_width= this->get_parameter("grid_width").as_int();
    this->grid_resolution = this->get_parameter("grid_resolution").as_double();
    this->p_occ = this->get_parameter("p_occ").as_double();
    this->p_free = this->get_parameter("p_free").as_double();
    

    this->grid.header.frame_id = "odom";
    this->grid.info.resolution = grid_resolution;
    this->grid.info.width = grid_width;
    this->grid.info.height = grid_height;
    this->grid.info.origin = geometry_msgs::msg::Pose();
    this->grid.info.origin.position.x = -(grid_resolution * (double)grid_width) / 2;
    this->grid.info.origin.position.y = -(grid_resolution * (double)grid_height) / 2;
    this->grid.info.origin.position.z = 0.0;
    this->grid.info.origin.orientation.w = 1.0;

    this->l0 = 0.0;
    this->l_occ = std::log(p_occ / (1.0 - p_occ));
    this->l_free = std::log(p_free / (1.0 - p_free));
    this->l_min = std::log(0.01 / 0.99);
    this->l_max = std::log(0.99/0.01);

    this->cell_log_odds.assign(grid_width * grid_height, l0);
    this->grid.data.assign(grid.info.height * grid.info.width, 50);

    //std::cout << this->grid.info.origin.position.x << " " << this->grid.info.origin.position.y << " " << this->grid.info.origin.position.z << std::endl;
    //std::cout << this->grid.info.origin.orientation.w << " " << this->grid.info.origin.orientation.x << " " << this->grid.info.origin.orientation.y << " " << this->grid.info.origin.orientation.z << std::endl;

    gridPublisher = create_publisher<nav_msgs::msg::OccupancyGrid>("/grid", 1);

    subOdom = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 1,
        std::bind(&Mapping::odomCallback, this, std::placeholders::_1));

    tf2Buffer = std::make_shared<tf2_ros::Buffer>(get_clock(), 10s);
    std::shared_ptr<tf2_ros::CreateTimerROS> timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(get_node_base_interface(), get_node_timers_interface());
    tf2Buffer->setCreateTimerInterface(timer_interface);
    tf2Listener = std::make_shared<tf2_ros::TransformListener>(*tf2Buffer);

    subLaser.subscribe(this, "/scan");
    tf2MessageFilter = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(subLaser, *tf2Buffer, "odom", 3, get_node_logging_interface(), get_node_clock_interface(), 500ms);
    tf2MessageFilter->setTolerance(100ms);
    tf2MessageFilter->registerCallback(&Mapping::laserCallback, this);


}

Vec2i Mapping::odom_to_grid(Vec2f world) {
    double gx = (world.x - this->grid.info.origin.position.x) / this->grid_resolution;
    double gy = (world.y - this->grid.info.origin.position.y) / this->grid_resolution;
    return Vec2i(gx, gy);
}

void Mapping::set_cell(int idx, double l_update){
    if (idx < 0) return;                       // out of map

    cell_log_odds[idx] = std::clamp(cell_log_odds[idx] + l_update, l_min, l_max);

    // convert once to 0…100 / -1 for publishing
    double p = 1.0 - 1.0 / (1.0 + std::exp(cell_log_odds[idx]));
    grid.data[idx] = static_cast<int8_t>( std::round(100.0 * p));
}

double Mapping::inverseSensorModel(double dist, double z, double z_max){
    if (z >= z_max || std::isnan(z))                  // no return (max-range reading)
        return l_free;

    // Wall strenght
    if (dist > z - grid_resolution * 0.7)             // hit cell
        return l_occ;

    if (dist < z)                                     // ray before hit
        return l_free;

    return 0.0;                                       // behind hit
}


int Mapping::get_grid_index(Vec2f &p_world){
    Vec2i g = odom_to_grid(p_world);

    if(g.x < 0 || g.x >= this->grid_width || g.y < 0 || g.y >= this->grid_height){
        return -1;
    }

    return g.y * this->grid_width + g.x;
}

Vec2f Mapping::grid_coords(int idx) {
    int x = idx % this->grid_width;
    int y = idx / this->grid_width;
    return Vec2f(x, y);
}

Vec2f Mapping::gridIndexToWorld(int idx) {
    int w = this->grid.info.width;
    int h = this->grid.info.height;
    int max_cells = w * h;

    if (idx < 0 || idx >= max_cells) {
        return Vec2f(-1, -1);  // invalid index
    }

    // recover integer cell‐coords
    int i = idx % w;
    int j = idx / w;

    // bottom‐left corner of the grid in odom frame:
    double ox = this->grid.info.origin.position.x;
    double oy = this->grid.info.origin.position.y;
    double r  = this->grid.info.resolution;

    double x_world = ox + (i + 0.5) * r;
    double y_world = oy + (j + 0.5) * r;

    return Vec2f(x_world, y_world);
}

void Mapping::odomCallback(const nav_msgs::msg::Odometry &odom)
{

    std::lock_guard<std::mutex> guard(mutex);

    robotPos = Vec2f(odom.pose.pose.position.x, odom.pose.pose.position.y);
    robotYaw = tf2::getYaw(odom.pose.pose.orientation);
    
    int idx = this->get_grid_index(robotPos);
    
    if(idx >= 0){
        set_cell(idx, l_free);
    }
}


void Mapping::laserCallback(const sensor_msgs::msg::LaserScan &scan) {
    std::lock_guard<std::mutex> guard(mutex);

    //std::cout << "LASER" << std::endl;

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

        if (scan.ranges[i] < scan.range_min) {
            continue;
        }

        

        // avoids large deviation (1m) of single beams
        if (i > 0 && i + 1 < scan.ranges.size()) {
            if (!validRange(scan.ranges[i - 1], scan.ranges[i], scan.ranges[i + 1], 1)) {
                continue;
            }
        }


        bool is_hit = !std::isnan(scan.ranges[i]);
        Vec2f pLaser = scan.ranges[i] * Vec2f::fromAngle(angle);

        if(scan.range_max < scan.ranges[i] || std::isnan(scan.ranges[i])){
            pLaser = 2.5 * Vec2f::fromAngle(angle);
        }
        
        tf2::Transform interpolatedTransform = slerpTransforms(startTransform, endTransform, i / (scan.ranges.size() - 1.0));
        Vec2f viewpointOdom = interpolatedTransform(Vec2f(0, 0).toTf2Vector3());
        Vec2f pOdom = interpolatedTransform(pLaser.toTf2Vector3());

        auto grid_coords_viewpointOdom = grid_coords(get_grid_index(viewpointOdom));
        auto grid_coords_pOdom = grid_coords(get_grid_index(pOdom));

        GridTraversal gt(grid_coords_viewpointOdom, grid_coords_pOdom);
        while(gt.next()){
            auto gp = gt.get();
            int idx = gp.y * grid.info.width + gp.x;

            Vec2f cw = gridIndexToWorld(idx);
            double dist = (cw - viewpointOdom).norm();

            this->set_cell(idx, inverseSensorModel(dist, scan.ranges[i], scan.range_max));
        }
    
        int idx = this->get_grid_index(pOdom);
        if(idx >= 0 && is_hit){
            set_cell(idx, l_occ);
        }
    }

    // TODO: traverse grid and update occupancy information

    grid.header.stamp = scan.header.stamp;
    gridPublisher->publish(this->grid);
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


// checks neighbours of beam to determine if beam is valid
bool Mapping::validRange(double prev, double curr, double next, double jump_thresh) {
    if (!std::isfinite(curr)) {
        return true;
        // area in front of robot
    }
    return std::abs(curr - prev) < jump_thresh
        && std::abs(curr - next) < jump_thresh;
}

  