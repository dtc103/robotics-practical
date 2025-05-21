#ifndef BALL_TRACKER_H
#define BALL_TRACKER_H

//#define EIGEN_MALLOC_ALREADY_ALIGNED 1
//#define PCL_NO_PRECOMPILE

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <message_filters/subscriber.h>

#include <memory>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>

class BallTracker: public rclcpp::Node {
    public:
        BallTracker();

        typedef pcl::PointXYZRGB PointT;
        typedef pcl::PointCloud<PointT> PointCloud;
        typedef pcl::PointCloud<pcl::Normal> NormalCloud;

    private:
        void pointsCallback(const sensor_msgs::msg::PointCloud2 &pointCloud);
        std::optional<pcl::ModelCoefficients> findAndRemoveFloor(PointCloud::Ptr cloud, NormalCloud::Ptr normals);
        void findAndRemoveWalls(PointCloud::Ptr cloud, NormalCloud::Ptr normals);
        visualization_msgs::msg::MarkerArray findAndExtractBalls(PointCloud::Ptr cloud, NormalCloud::Ptr normals, const std::optional<pcl::ModelCoefficients> &groundPlane);
        visualization_msgs::msg::Marker createSphereMarker(const PointCloud &cloud, const pcl::PointIndices &inliers, pcl::ModelCoefficients sphereModel);
        void processBalls(const visualization_msgs::msg::MarkerArray &balls);

        void tick();

        void removeNaNPoints(PointCloud &cloud);
        void estimateNormals(PointCloud::ConstPtr cloud, NormalCloud::Ptr normals);
        PointCloud pointCloud2ToPclPointCloud(const sensor_msgs::msg::PointCloud2 &pointCloud2);
        sensor_msgs::msg::PointCloud2 pclPointCloudToPointCloud2(const PointCloud &pclPointCloud);

        std::shared_ptr<tf2_ros::Buffer> tf2Buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf2Listener;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subPoints;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudFloor;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudObjects;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudBalls;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubBalls;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubGoal;
        rclcpp::TimerBase::SharedPtr timer;

        std::optional<geometry_msgs::msg::PoseStamped> goal;
};

#endif
