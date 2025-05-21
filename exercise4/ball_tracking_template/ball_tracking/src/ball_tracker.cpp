#include "ball_tracker.h"
#include <tf2/utils.h>
#include <builtin_interfaces/msg/time.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <color_names/color_names.hpp>
#include <chrono>
#include <functional>
#include <algorithm>
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

BallTracker::BallTracker(): Node("ball_tracker") {
    declare_parameter("floor_methodType", pcl::SAC_RANSAC); // RANSAC method (plane)
    declare_parameter("floor_maxIterations", 50); // RANSAC iterations (plane)
    declare_parameter("floor_normalDistanceWeight", 0.1);
    declare_parameter("floor_distanceThreshold", 0.035);
    declare_parameter("floor_epsAngle", 0.3);
    declare_parameter("floor_minPoints", 1000); // min no of point per plane

    declare_parameter("walls", 1); // no of planes
    declare_parameter("wall_methodType", pcl::SAC_RANSAC); // RANSAC method (plane)
    declare_parameter("wall_maxIterations", 50); // RANSAC iterations (plane)
    declare_parameter("wall_normalDistanceWeight", 0.1);
    declare_parameter("wall_distanceThreshold", 0.3);
    declare_parameter("wall_minPoints", 1000); // min no of point per plane

    declare_parameter("spheres", 10); // no of spheres
    declare_parameter("sphere_methodType", pcl::SAC_RANSAC); // RANSAC method (sphere)
    declare_parameter("sphere_maxIterations", 100); // RANSAC iterations (sphere)
    declare_parameter("sphere_normalDistanceWeight", 0.1);
    declare_parameter("sphere_distanceThreshold", 0.025);
    declare_parameter("sphere_radiusMin", 0.05); // sphere radius (min)
    declare_parameter("sphere_radiusMax", 0.13); // sphere radius (max)
    declare_parameter("sphere_minPoints", 1000); // min no of point per sphere
    declare_parameter("sphere_maxDistanceToFloor", 0.05); // min no of point per sphere

    pubCloudFloor = create_publisher<sensor_msgs::msg::PointCloud2>("/points_floor", 1);
    pubCloudObjects = create_publisher<sensor_msgs::msg::PointCloud2>("/points_objects", 1);
    pubCloudBalls = create_publisher<sensor_msgs::msg::PointCloud2>("/points_spheres", 1);
    pubBalls = create_publisher<visualization_msgs::msg::MarkerArray>("/balls", 1);
    pubGoal = create_publisher<geometry_msgs::msg::PoseStamped>("/goal", 1);

    subPoints = create_subscription<sensor_msgs::msg::PointCloud2>("/camera/depth_registered/points", 10, std::bind(&BallTracker::pointsCallback, this, std::placeholders::_1));
    timer = create_wall_timer(50ms, std::bind(&BallTracker::tick, this));

    tf2Buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf2Listener = std::make_shared<tf2_ros::TransformListener>(*tf2Buffer);

}

void BallTracker::pointsCallback(const sensor_msgs::msg::PointCloud2 &pointCloud) {
    RCLCPP_INFO(get_logger(), "Point cloud received");

    sensor_msgs::msg::PointCloud2 pointCloudBaseFootprint;
    if (!pcl_ros::transformPointCloud("base_footprint", pointCloud, pointCloudBaseFootprint, *tf2Buffer)) {
        return;
    }

    PointCloud::Ptr pclPointCloud = std::make_shared<PointCloud>();
    *pclPointCloud = pointCloud2ToPclPointCloud(pointCloudBaseFootprint);

    pcl::Indices indexMapping;
    pcl::removeNaNFromPointCloud(*pclPointCloud, *pclPointCloud, indexMapping);

    NormalCloud::Ptr normals = std::make_shared<NormalCloud>();
    estimateNormals(pclPointCloud, normals);

    // find the ground plane and extract it
    std::optional<pcl::ModelCoefficients> floorPlane = findAndRemoveFloor(pclPointCloud, normals);
    if (!floorPlane) {
        RCLCPP_INFO(get_logger(), "cannot find the floor");
    }

    findAndRemoveWalls(pclPointCloud, normals);

    pubCloudObjects->publish(pclPointCloudToPointCloud2(*pclPointCloud));

    // find all balls
    visualization_msgs::msg::MarkerArray balls = findAndExtractBalls(pclPointCloud, normals, floorPlane);

    visualization_msgs::msg::MarkerArray ballsWithDeleteAllMarker;
    visualization_msgs::msg::Marker deleteAllMarker;
    deleteAllMarker.ns = "delete_all";
    deleteAllMarker.action = visualization_msgs::msg::Marker::DELETEALL;
    ballsWithDeleteAllMarker.markers.push_back(deleteAllMarker);
    ballsWithDeleteAllMarker.markers.insert(ballsWithDeleteAllMarker.markers.end(), balls.markers.begin(), balls.markers.end());
    pubBalls->publish(ballsWithDeleteAllMarker);

    // process the found balls
    processBalls(balls);
}

std::optional<pcl::ModelCoefficients> BallTracker::findAndRemoveFloor(PointCloud::Ptr cloud, NormalCloud::Ptr normals) {
    // 1. find a horziontal plane
    // 2. publish the extracted points
    // 3. remove all points of that plane from the point cloud

    // Take a look at the links provided in the first lines of this file!

    // create segmenter with model type pcl::SACMODEL_NORMAL_PARALLEL_PLANE and set the cloud and its normals as inputs
    // TODO ...

    // find segmentation
    pcl::ModelCoefficients planeModel;
    // TODO ...

    // check if the plane has enough of points
    if (true /* TODO */) {
        return std::nullopt;
    }

    // extract all inliers and put them into planePoints
    PointCloud planePoints;
    // TODO ...

    // publish the plane
    pubCloudFloor->publish(pclPointCloudToPointCloud2(planePoints));

    // remove the points from the point cloud
    // TODO ...

    // remove the normals from the normal cloud
    // TODO ...

    return planeModel;
}

void BallTracker::findAndRemoveWalls(PointCloud::Ptr cloud, NormalCloud::Ptr normals) {
    // create segmenter with model type pcl::SACMODEL_NORMAL_PLANE and set the cloud and its normals as inputs
    // TODO ...

    for (int i = 0; i < get_parameter("walls").as_int(); i++) {
        // find segmentation
        // TODO ...

        // check if the plane has enough of points
        if (true /* TODO */) {
            break;
        }

        // remove the points from the point cloud
        // TODO ...

        // remove the normals from the normal cloud
        // TODO ...
    }
}

visualization_msgs::msg::MarkerArray BallTracker::findAndExtractBalls(PointCloud::Ptr cloud, NormalCloud::Ptr normals, const std::optional<pcl::ModelCoefficients> &groundPlane) {
    // create segmenter with model type pcl::SACMODEL_NORMAL_SPHERE and set the cloud and its normals as inputs
    // TODO ...

    visualization_msgs::msg::MarkerArray ballMarkers;
    PointCloud cloudBalls;
    cloudBalls.header = cloud->header;

    for (int i = 0; i < get_parameter("spheres").as_int(); i++) {
        // find segmentation
        pcl::PointIndices::Ptr inliers = std::make_shared<pcl::PointIndices>();
        pcl::ModelCoefficients sphereModel;
        // TODO ...

        // check if the sphere has enough of points
        if (true /* TODO */) {
            break;
        }

        // bonus: use plane model to limit ball positions to the ground
        if (!groundPlane /* || TODO */) {
            // analyze the ball
            ballMarkers.markers.push_back(createSphereMarker(*cloud, *inliers, sphereModel));
            ballMarkers.markers.back().id = i;
        }

        // extract all inliers
        PointCloud spherePoints;
        // TODO ...

        cloudBalls += spherePoints;

        // remove the points from the point cloud
        // TODO ...

        // remove the normals from the normal cloud
        // TODO ...
    }

    // publish the spheres
    pubCloudBalls->publish(pclPointCloudToPointCloud2(cloudBalls));

    return ballMarkers;
}

visualization_msgs::msg::Marker BallTracker::createSphereMarker(const PointCloud &cloud, const pcl::PointIndices &inliers, pcl::ModelCoefficients sphereModel) {
    visualization_msgs::msg::Marker sphereMarker;
    sphereMarker.header = pcl_conversions::fromPCL(cloud.header);
    sphereMarker.type = visualization_msgs::msg::Marker::SPHERE;
    sphereMarker.action = visualization_msgs::msg::Marker::ADD;
    sphereMarker.lifetime = rclcpp::Duration(5s);

    // TODO: set the position and scale of the sphere marker
    sphereMarker.pose.position.x = 0;
    sphereMarker.pose.position.y = 0;
    sphereMarker.pose.position.z = 0;
    sphereMarker.scale.x = 0;
    sphereMarker.scale.y = 0;
    sphereMarker.scale.z = 0;

    // iterate over all inliers in the cloud and calculate the mean color
    for (pcl::index_t idx: inliers.indices) {
        const PointT &p = cloud.points[idx];
        // TODO ...
    }

    // TODO: set the color of the sphere marker
    sphereMarker.color.r = 0;
    sphereMarker.color.g = 0;
    sphereMarker.color.b = 0;
    sphereMarker.color.a = 1;

    return sphereMarker;
}

void BallTracker::processBalls(const visualization_msgs::msg::MarkerArray &balls) {
    goal.reset();

    for (const visualization_msgs::msg::Marker &ball: balls.markers) {
        bool isBlue = false /* TODO */;

        if (isBlue) {
            goal.emplace();

            // TODO: set the goal pose
            //goal->header = ...;
            //goal->pose.position.x = ...;
            //goal->pose.position.y = ...;

            return;
        }
    }
}

void BallTracker::tick() {
    // publish the goal if the ball was found
    // TODO ...
}

void BallTracker::removeNaNPoints(PointCloud &cloud) {
    auto isAnyCoordinateNaN = [](PointT p) { return std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z); };
    std::ranges::remove_if(cloud.points, isAnyCoordinateNaN);
}

void BallTracker::estimateNormals(PointCloud::ConstPtr cloud, NormalCloud::Ptr normals) {
    pcl::NormalEstimation<PointT, pcl::Normal> normal_estimation;
    pcl::search::KdTree<PointT>::Ptr tree = std::make_shared<pcl::search::KdTree<PointT>>();

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

sensor_msgs::msg::PointCloud2 BallTracker::pclPointCloudToPointCloud2(const PointCloud &pclPointCloud) {
    pcl::PCLPointCloud2 pclPointCloud2;
    pcl::toPCLPointCloud2(pclPointCloud, pclPointCloud2);

    sensor_msgs::msg::PointCloud2 pointCloud2;
    pcl_conversions::fromPCL(pclPointCloud2, pointCloud2);

    return pointCloud2;
}
