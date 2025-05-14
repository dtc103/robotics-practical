#include "ball_tracker.h"
#include <tf2/utils.h>
#include <builtin_interfaces/msg/time.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <color_names/color_names.hpp>
#include <chrono>
#include <functional>
#include <pcl_ros/transforms.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

/// To get an understanding of the PCL API, take a quick look at these websites:
///   https://pcl.readthedocs.io/projects/tutorials/en/latest/planar_segmentation.html#planar-segmentation
///   https://pcl.readthedocs.io/projects/tutorials/en/latest/extract_indices.html?highlight=extract%20indices
///   https://pcl.readthedocs.io/projects/tutorials/en/latest/random_sample_consensus.html#random-sample-consensus

using namespace std::chrono_literals;

BallTracker::BallTracker(): Node("ball_tracker"), send_goal(false) {
    declare_parameter("ransac_plane", 0); // RANSAC method (plane)
    declare_parameter("ransac_sphere", 0); // RANSAC method (sphere)

    declare_parameter("sphere_radius_min", 0.06); // sphere radius (min)
    declare_parameter("sphere_radius_max", 0.13); // sphere radius (max)

    declare_parameter("iterations_plane", 50); // RANSAC iterations (plane)
    declare_parameter("iterations_sphere", 7000); // RANSAC iterations (sphere)

    declare_parameter("sphere_min_points", 50); // min no of point per sphere

    declare_parameter("sphere_distance_threshold", 0.06);
    declare_parameter("sphere_normal_distance_weight", 0.1);

    declare_parameter("planes", 1); // no of planes
    declare_parameter("spheres", 10); // no of spheres

    pubPlane = create_publisher<sensor_msgs::msg::PointCloud2>("/plane", 1);
    pubBalls = create_publisher<sensor_msgs::msg::PointCloud2>("/balls", 1);
    pubGoal = create_publisher<geometry_msgs::msg::PoseStamped>("/goal", 1);

    timer = create_wall_timer(50ms, std::bind(&BallTracker::tick, this));

    tf2Buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
    // Create the timer interface before call to waitForTransform, to avoid a tf2_ros::CreateTimerInterfaceException exception
    std::shared_ptr<tf2_ros::CreateTimerROS> timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(get_node_base_interface(), get_node_timers_interface());
    tf2Buffer->setCreateTimerInterface(timer_interface);
    tf2Listener = std::make_shared<tf2_ros::TransformListener>(*tf2Buffer);

    subPoints.subscribe(this, "/camera/depth_registered/points");
    tf2MessageFilter = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(subPoints, *tf2Buffer, "odom", 3, get_node_logging_interface(), get_node_clock_interface(), 500ms);
    tf2MessageFilter->setTolerance(100ms);
    // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
    tf2MessageFilter->registerCallback(&BallTracker::pointsCallback, this);
}

void BallTracker::pointsCallback(const sensor_msgs::msg::PointCloud2 &pointCloud) {
    std::lock_guard<std::mutex> guard(mutex);

    RCLCPP_INFO(get_logger(), "Point cloud received");

    sensor_msgs::msg::PointCloud2 pointCloudOdom;
    pcl_ros::transformPointCloud("odom", pointCloud, pointCloudOdom, *tf2Buffer);

    PointCloud::Ptr pclPointCloud = std::make_shared<PointCloud>();
    *pclPointCloud = pointCloud2ToPclPointCloud(pointCloudOdom);

    NormalCloud::Ptr normals = std::make_shared<NormalCloud>();
    estimateNormals(pclPointCloud, normals);

    // find the ground plane and extract it
    Eigen::Vector4f plane_model;
    if (!findAndExtractPlane(pclPointCloud, normals, plane_model)) {
        RCLCPP_INFO(get_logger(), "cannot find the plane");
    }

    //pubPlane->publish(pclPointCloudTopointCloud2(*pclPointCloud));
    //return;

    // find all balls
    PointCloud balls;
    findAndExtractBalls(pclPointCloud, normals, plane_model, balls);

    // process the found balls
    processBalls(balls);
}

bool BallTracker::findAndExtractPlane(PointCloud::Ptr cloud, NormalCloud::Ptr normals, Eigen::Vector4f &plane_model) {
    // 1. find a plane
    // 2. remove all point of that plane
    // 3. publish the extracted points

    // TAKE A LOOK AT THE LINKS PROVIDED IN THE FIRST LINES OF THIS FILE!

    int no_of_planes_to_extract = get_parameter("planes").as_int();
    for (int i = 0; i < no_of_planes_to_extract; i++) {
        // find segmentation
        // Use segmenter_plane_ to segment the cloud into a plane (you need to set the input cloud and input normals)
        // TODO...

        // Set a minimum number of points to be used as a plane
        // TODO...

        // extract all inliers and put them into 'extracted_plane'
        PointCloud::Ptr extracted_plane(new PointCloud);
        extracted_plane->header = cloud->header;
        // TODO...

        // remove the points from the point cloud
        // TODO...

        // remove the normals from the normal cloud
        // TODO...

        // update plane model
        // plane_model[0] = coefficients_plane->values[0];
        // plane_model[1] = coefficients_plane->values[1];
        // plane_model[2] = coefficients_plane->values[2];
        // plane_model[3] = coefficients_plane->values[3];

        // publish the plane
        pubPlane->publish(pclPointCloudTopointCloud2(*extracted_plane));

        return true;
    }

    return false;
}

void BallTracker::findAndExtractBalls(PointCloud::Ptr cloud, NormalCloud::Ptr normals, const Eigen::Vector4f &plane_model, PointCloud &balls) {
    pcl::ModelCoefficients::Ptr coefficients_sphere = std::make_shared<pcl::ModelCoefficients>();

    balls.header = cloud->header;

    int no_of_spheres_to_extract = get_parameter("spheres").as_int();
    for (int i = 0; i < no_of_spheres_to_extract; i++) {
        // find segmentation
        // TODO...

        // calculate sphere position
        pcl::PointXYZ sphere_center;// = TODO... (coefficients_sphere)

        // update the sphere model
        // bonus: use plane model to limit ball positions to the ground
        // TODO...

        // extract all inliers
        PointCloud::Ptr extracted_sphere_points(new PointCloud);
        // TODO...

        // analyze the ball
        extractBall(sphere_center, *extracted_sphere_points, balls);

        // remove the points from the point cloud
        // TODO...

        // remove the normals from the normal cloud
        // TODO...
    }

    // publish the spheres
    pubBalls->publish(pclPointCloudTopointCloud2(balls));
}

void BallTracker::extractBall(pcl::PointXYZ center, const PointCloud &points, PointCloud &balls) {
    // iterate all points in 'points' and calculate the mean color
    PointT ball;
    // ball has fields: ['x', 'y', 'z', 'r', 'g', 'b']
    // TODO...

    for (const PointT &p: points.points) {
        // TODO...
        // initialize 'ball' and compute it's mean color
        // write the resulting color into 'ball''s member fields
    }

    // TODO...
    // publish the new point cloud, containing all balls
    balls.points.push_back(ball);
}

void BallTracker::processBalls(PointCloud &balls) {
    send_goal = false;
    for (const PointT &ball: balls.points) {
        //if (isBlue(ball)) {
        //    // TODO...
        //}

    }
}

void BallTracker::tick() {
    std::lock_guard<std::mutex> guard(mutex);

    // TODO: update 'send_ball_pose' to be true, iff a ball is seen

    // this code runs at a higher frequency than the cloud callback (~30Hz)
    if (send_goal) {
        sendBallPoseAsGoalPose(last_goal_ball);
    }
}

void BallTracker::sendBallPoseAsGoalPose(const PointT &ball) {
    // publish the position of the ball as the goal pose
    geometry_msgs::msg::PoseStamped goal;

    // TODO...

    pubGoal->publish(goal);
}

void BallTracker::estimateNormals(PointCloud::ConstPtr cloud, NormalCloud::Ptr normals) {
    pcl::NormalEstimation<PointT, pcl::Normal> normal_estimation;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

    normal_estimation.setSearchMethod(tree);
    normal_estimation.setInputCloud(cloud);
    normal_estimation.setKSearch(50);
    normal_estimation.compute(*normals);
}

BallTracker::PointCloud BallTracker::pointCloud2ToPclPointCloud(const sensor_msgs::msg::PointCloud2 &pointCloud2) {
    pcl::PCLPointCloud2 pclPointCloud2;
    pcl_conversions::toPCL(pointCloud2, pclPointCloud2);

    PointCloud pclPointCloud;
    pcl::fromPCLPointCloud2(pclPointCloud2, pclPointCloud);

    return pclPointCloud;
}

sensor_msgs::msg::PointCloud2 BallTracker::pclPointCloudTopointCloud2(const PointCloud &pclPointCloud) {
    pcl::PCLPointCloud2 pclPointCloud2;
    pcl::toPCLPointCloud2(pclPointCloud, pclPointCloud2);

    sensor_msgs::msg::PointCloud2 pointCloud2;
    pcl_conversions::fromPCL(pclPointCloud2, pointCloud2);

    return pointCloud2;
}
