#include <ball_tracker/ball_tracker.h>
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
    pubCloudWalls = create_publisher<sensor_msgs::msg::PointCloud2>("/points_walls", 1);
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
    RCLCPP_INFO(get_logger(), "found the floor");

    this->findAndRemoveWalls(pclPointCloud, normals);

    pubCloudObjects->publish(pclPointCloudToPointCloud2(*pclPointCloud));

    // find all balls
    visualization_msgs::msg::MarkerArray balls = findAndExtractBalls(pclPointCloud, normals, floorPlane);

    visualization_msgs::msg::MarkerArray ballsWithDeleteAllMarker;
    visualization_msgs::msg::Marker deleteAllMarker;
    deleteAllMarker.header.frame_id = "base_footprint";
    deleteAllMarker.header.stamp = this->now();
    deleteAllMarker.ns = "delete_all";
    deleteAllMarker.action = visualization_msgs::msg::Marker::DELETEALL;
    ballsWithDeleteAllMarker.markers.push_back(deleteAllMarker);
    ballsWithDeleteAllMarker.markers.insert(ballsWithDeleteAllMarker.markers.end(), balls.markers.begin(), balls.markers.end());
    
    RCLCPP_INFO(get_logger(),
      "DELETEALL Marker: frame_id=%s, ns=%s, id=%d, action=%d",
      deleteAllMarker.header.frame_id.c_str(),
      deleteAllMarker.ns.c_str(),
      deleteAllMarker.id,
      deleteAllMarker.action);
      

    for (const auto &m : ballsWithDeleteAllMarker.markers) {
        RCLCPP_INFO(get_logger(),
          "Marker to publish: ns=%s, id=%d, action=%d, frame_id=%s, stamp=%u.%u",
          m.ns.c_str(), m.id, m.action,
          m.header.frame_id.c_str(),
          m.header.stamp.sec, m.header.stamp.nanosec);
      }

    
    RCLCPP_INFO(get_logger(),
      "Publishing MarkerArray with %zu markers",
      ballsWithDeleteAllMarker.markers.size());
    
    pubBalls->publish(ballsWithDeleteAllMarker);

    // process the found balls
    processBalls(balls);
}

std::optional<pcl::ModelCoefficients> BallTracker::findAndRemoveFloor(PointCloud::Ptr cloud, NormalCloud::Ptr normals) {
    
    RCLCPP_INFO(this->get_logger(), "%s", "starting from floor");
    // 1. find a horziontal plane
    // 2. publish the extracted points
    // 3. remove all points of that plane from the point cloud

    // Take a look at the links provided in the first lines of this file!

    // create segmenter with model type pcl::SACMODEL_NORMAL_PARALLEL_PLANE and set the cloud and its normals as inputs
    // TODO ...

    pcl::PointIndices::Ptr inliers = std::make_shared<pcl::PointIndices>();

    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(get_parameter("floor_distanceThreshold").as_double());
    seg.setInputCloud(cloud);
    seg.setInputNormals(normals);
    seg.setMaxIterations(get_parameter("floor_maxIterations").as_int());

    // find segmentation
    pcl::ModelCoefficients::Ptr planeModel = std::make_shared<pcl::ModelCoefficients>();
    // TODO ...
    seg.segment(*inliers, *planeModel);

    // check if the plane has enough of points
    if (inliers->indices.size() <= get_parameter("floor_minPoints").as_int()) {
        return std::nullopt;
    }

    // extract all inliers and put them into planePoints
    PointCloud::Ptr planePoints(new PointCloud);
    planePoints->header = cloud->header;

    pcl::ExtractIndices<pcl::PointXYZRGB> p_extractor;
    p_extractor.setInputCloud(cloud);
    p_extractor.setIndices(inliers);
    p_extractor.setNegative(false);
    
    p_extractor.filter(*planePoints);

    // publish the plane
    pubCloudFloor->publish(pclPointCloudToPointCloud2(*planePoints));

    p_extractor.setNegative(true);

    p_extractor.filter(*cloud);

    pcl::ExtractIndices<pcl::Normal> n_extractor;
    n_extractor.setInputCloud(normals);
    n_extractor.setIndices(inliers);
    n_extractor.setNegative(true);
    n_extractor.filter(*normals);

    return *planeModel;
}

void BallTracker::findAndRemoveWalls(PointCloud::Ptr cloud, NormalCloud::Ptr normals) {
    // create segmenter with model type pcl::SACMODEL_NORMAL_PLANE and set the cloud and its normals as inputs

    pcl::ModelCoefficients::Ptr coeff = std::make_shared<pcl::ModelCoefficients>();
    pcl::PointIndices::Ptr inliers = std::make_shared<pcl::PointIndices>();
    
    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(get_parameter("wall_distanceThreshold").as_double());
    seg.setInputCloud(cloud);
    seg.setInputNormals(normals);
    seg.setMaxIterations(get_parameter("wall_maxIterations").as_int());

    for (int i = 0; i < get_parameter("walls").as_int(); i++) {
        // find segmentation
        // TODO ...
        seg.segment(*inliers, *coeff);

        // check if the plane has enough of points
        if (inliers->indices.size() <= get_parameter("wall_minPoints").as_int()) {
            break;
        }

        PointCloud::Ptr extracted_plane(new PointCloud);
        extracted_plane->header = cloud->header;

        pcl::ExtractIndices<pcl::PointXYZRGB> p_extractor;
        p_extractor.setInputCloud(cloud);
        p_extractor.setIndices(inliers);
        p_extractor.setNegative(false);
        
        p_extractor.filter(*extracted_plane);

        p_extractor.setNegative(true);

        p_extractor.filter(*cloud);

        // publish the wall
        pubCloudWalls->publish(pclPointCloudToPointCloud2(*extracted_plane));
        
        pcl::ExtractIndices<pcl::Normal> n_extractor;
        n_extractor.setInputCloud(normals);
        n_extractor.setIndices(inliers);
        n_extractor.setNegative(true);
        n_extractor.filter(*normals);
    }
}

visualization_msgs::msg::MarkerArray BallTracker::findAndExtractBalls(
    PointCloud::Ptr cloud,
    NormalCloud::Ptr normals,
    const std::optional<pcl::ModelCoefficients> &groundPlane)
{
    visualization_msgs::msg::MarkerArray ballMarkers;
    PointCloud::Ptr cloudBalls(new PointCloud);
    cloudBalls->header = cloud->header;

    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> segSphere;
    segSphere.setOptimizeCoefficients(true);
    segSphere.setModelType(pcl::SACMODEL_NORMAL_SPHERE);
    segSphere.setMethodType(get_parameter("sphere_methodType").as_int());
    segSphere.setNormalDistanceWeight(get_parameter("sphere_normalDistanceWeight").as_double());
    segSphere.setRadiusLimits(
        get_parameter("sphere_radiusMin").as_double(),
        get_parameter("sphere_radiusMax").as_double());
    segSphere.setDistanceThreshold(get_parameter("sphere_distanceThreshold").as_double());
    segSphere.setMaxIterations(get_parameter("sphere_maxIterations").as_int());
    segSphere.setInputCloud(cloud);
    segSphere.setInputNormals(normals);

    int maxSpheres = get_parameter("spheres").as_int();
    int id_counter = 0;
    for (int i = 0; i < maxSpheres; ++i) {
        // 1) Segmentiere eine Kugel:
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients sphereModel;
        segSphere.segment(*inliers, sphereModel);

        if (inliers->indices.empty()) {
            RCLCPP_INFO(get_logger(),
            "Sphere %d: keine Inlier gefunden, breche Schleife ab.", i);
            break;
        }

        RCLCPP_INFO(get_logger(),
                "Sphere-segmentation %d: inliers=%zu, center=(%.3f,%.3f,%.3f), radius=%.3f",
                i,
                inliers->indices.size(),
                sphereModel.values[0],
                sphereModel.values[1],
                sphereModel.values[2],
                sphereModel.values[3]);



        
        // 2) Abbruch, wenn zu wenige Punkte:
        if ((int)inliers->indices.size() < get_parameter("sphere_minPoints").as_int()) {
            break;
        }

        // (Optional: Einschränkung bzgl. Bodenebene)
        if (groundPlane) {
            // Ebene: a*x + b*y + c*z + d = 0
            const auto &coeffs = groundPlane.value().values;
            double a = coeffs[0], b = coeffs[1], c = coeffs[2], d = coeffs[3];
        
            double x0 = sphereModel.values[0];
            double y0 = sphereModel.values[1];
            double z0 = sphereModel.values[2];
        
            // Abstand Punkt–Ebene: |a*x0 + b*y0 + c*z0 + d| / sqrt(a^2+b^2+c^2)
            double dist = std::abs(a*x0 + b*y0 + c*z0 + d)
                          / std::sqrt(a*a + b*b + c*c);
        
            double maxDist = get_parameter("sphere_maxDistanceToFloor").as_double();
            RCLCPP_INFO(get_logger(),
                  "Ball %d: Höhe über Boden = %.3f m (max=%.3f)",
                  i, dist, maxDist);
            if (dist > maxDist) {
                RCLCPP_WARN(get_logger(),
                    "Ball %d verworfen: zu weit über dem Boden.", i);
                continue;
            }
        }

        RCLCPP_INFO(get_logger(),
                "Verarbeite Ball %d: center=(%.3f,%.3f,%.3f), radius=%.3f",
                i,
                sphereModel.values[0],
                sphereModel.values[1],
                sphereModel.values[2],
                sphereModel.values[3]);

        // 3) Marker erzeugen _vor_ dem Entfernen:
        auto marker = createSphereMarker(*cloud, *inliers, sphereModel);
        marker.id = id_counter++;
        RCLCPP_INFO(get_logger(),
            "Created marker %d at (%.3f, %.3f, %.3f)", 
            marker.id,
            marker.pose.position.x,
            marker.pose.position.y,
            marker.pose.position.z);
        ballMarkers.markers.push_back(marker);

        // 4) Extrahiere die Kugelpunkte:
        PointCloud spherePoints;
        spherePoints.header = cloud->header;
        pcl::ExtractIndices<PointT> p_extractor;
        p_extractor.setInputCloud(cloud);
        p_extractor.setIndices(inliers);
        p_extractor.setNegative(false);
        p_extractor.filter(spherePoints);

        // Füge sie zum Gesamt-Cloud der Bälle hinzu:
        *cloudBalls += spherePoints;

        // 5) Entferne die Kugelpunkte aus dem Original-Cloud:
        p_extractor.setNegative(true);
        p_extractor.filter(*cloud);

        // 6) Entferne die zugehörigen Normals:
        pcl::ExtractIndices<pcl::Normal> n_extractor;
        n_extractor.setInputCloud(normals);
        n_extractor.setIndices(inliers);
        n_extractor.setNegative(true);
        n_extractor.filter(*normals);
    }

    // 7) Publiziere alle gefundenen Kugeln als einen neuen PointCloud2:
    pubCloudBalls->publish(pclPointCloudToPointCloud2(*cloudBalls));

    RCLCPP_INFO(get_logger(),
            "Total markers to publish: %zu", ballMarkers.markers.size());


    return ballMarkers;
}

visualization_msgs::msg::Marker BallTracker::createSphereMarker(const PointCloud &cloud, const pcl::PointIndices &inliers, pcl::ModelCoefficients sphereModel) {
    visualization_msgs::msg::Marker sphereMarker;
    sphereMarker.header = pcl_conversions::fromPCL(cloud.header);
    sphereMarker.type = visualization_msgs::msg::Marker::SPHERE;
    sphereMarker.action = visualization_msgs::msg::Marker::ADD;
    sphereMarker.lifetime = rclcpp::Duration(5s);
    sphereMarker.ns = "balls";
    
    double x0 = sphereModel.values[0];
    double y0 = sphereModel.values[1];
    double z0 = sphereModel.values[2];
    double r  = sphereModel.values[3];

    // TODO: set the position and scale of the sphere marker
    sphereMarker.pose.position.x = x0;
    sphereMarker.pose.position.y = y0;
    sphereMarker.pose.position.z = z0;
    sphereMarker.pose.orientation.w = 1.0;  

    // Durchmesser = 2 * Radius
    sphereMarker.scale.x = 2.0 * r;
    sphereMarker.scale.y = 2.0 * r;
    sphereMarker.scale.z = 2.0 * r;
    


    // iterate over all inliers in the cloud and calculate the mean color
    double sum_r = 0.0, sum_g = 0.0, sum_b = 0.0;
    for (auto idx : inliers.indices) {
        const auto &p = cloud.points[idx];
        sum_r += p.r;
        sum_g += p.g;
        sum_b += p.b;
    }

    // TODO: set the color of the sphere marker
    double n = static_cast<double>(inliers.indices.size());
    sphereMarker.color.r = (sum_r / n) / 255.0;
    sphereMarker.color.g = (sum_g / n) / 255.0;
    sphereMarker.color.b = (sum_b / n) / 255.0;
    sphereMarker.color.a = 1.0;


    return sphereMarker;
}


void BallTracker::processBalls(const visualization_msgs::msg::MarkerArray &balls) {
    goal.reset();

    for (const auto &ball : balls.markers) {
        bool isOrange = (ball.color.r > ball.color.b && ball.color.g > ball.color.b);
        if (!isOrange) {
            continue;
        }

        goal.emplace();
        goal->header = ball.header;      // Timestamp und Frame-ID übernehmen
        goal->pose   = ball.pose;        // Position und Orientierung übernehmen

        return;  
    }
}




void BallTracker::tick() {
    // publish the goal if the ball was found
    if (goal) {
        pubGoal->publish(*goal);
    }
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
