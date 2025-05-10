#include "autoserve_docking/dock_detector.hpp"

#include <chrono>
#include <thread>

#include <pcl/common/intersections.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <Eigen/Dense>
#include <cmath>

DockDetector::DockDetector() : Node("dock_detector") {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&DockDetector::scanCallback, this, std::placeholders::_1));

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", 10);
    cloud_pub2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud2", 10);
    dock_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("dock_pose", 10);

    loadDockReferencePCD();
}

void DockDetector::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    detectDock(msg);
}

void DockDetector::saveDockPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (pcl::io::savePCDFileBinary("/home/jatin/AutoServe/dock_area.pcd", *cloud) == -1) {
        std::cerr << "❌ Failed to save PCD file\n";
    } else {
        std::cout << "✅ Saved dock_area.pcd with " << cloud->size() << " points.\n";
    }
}

void DockDetector::loadDockReferencePCD() {
    dock_cloud_ref = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/jatin/AutoServe/dock_area.pcd", *dock_cloud_ref) == -1) {
        throw std::runtime_error("❌ Couldn't read file dock_area.pcd");
    }
    std::cout << "✅ Loaded dock_area.pcd with " << dock_cloud_ref->size() << " points.\n";
}

void DockDetector::detectDock(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    float angle = scan->angle_min;
    for (size_t i = 0; i < scan->ranges.size(); ++i, angle += scan->angle_increment) {
        float r = scan->ranges[i];
        float angle_deg = angle * 180.0 / M_PI;

        if ((angle_deg >= 135.0 && angle_deg <= 180.0) || (angle_deg >= -180.0 && angle_deg <= -135.0)) {
            if (std::isfinite(r) && r > scan->range_min && r < scan->range_max) {
                cloud->points.emplace_back(r * std::cos(angle), r * std::sin(angle), 0.0f);
            }
        }
    }

    // Publish cloud for debugging
    sensor_msgs::msg::PointCloud2 output_cloud;
    pcl::toROSMsg(*cloud, output_cloud);
    output_cloud.header = scan->header;
    cloud_pub_->publish(output_cloud);

    // saveDockPointCloud(cloud);
    detectDockICP(cloud, scan);
}

void DockDetector::detectDockICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr& live_cloud, const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
    // Preprocess (downsample) both clouds
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_ref(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_live(new pcl::PointCloud<pcl::PointXYZ>());
    vg.setLeafSize(0.002f, 0.002f, 0.002f);  // Downsampling leaf size

    // Downsample the reference dock cloud
    vg.setInputCloud(dock_cloud_ref);
    vg.filter(*filtered_ref);

    // Downsample the live scan cloud
    vg.setInputCloud(live_cloud);
    vg.filter(*filtered_live);

    // Apply ICP to align live cloud with reference dock cloud
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // icp.setInputSource(live_cloud);
    // icp.setInputTarget(dock_cloud_ref);
    icp.setInputSource(filtered_live);
    icp.setInputTarget(filtered_ref);
    icp.setMaximumIterations(100);
    pcl::PointCloud<pcl::PointXYZ> aligned;
    icp.align(aligned);

    if (!icp.hasConverged()) {
        std::cerr << "❌ ICP did not converge.\n";
        return;
    }
    Eigen::Matrix4f tf = icp.getFinalTransformation();

    // Extract position (translation) and orientation (rotation)
    float x = tf(0, 3);
    float y = tf(1, 3);
    float theta = std::atan2(tf(1, 0), tf(0, 0)); // Rotation in 2D
    std::cout << "Dock Location: x->" << x << " y->" << y << " theta->" << theta << std::endl;

    geometry_msgs::msg::PoseStamped dock_pose;
    dock_pose.header = scan->header;

    dock_pose.pose.position.x = x;
    dock_pose.pose.position.y = y;
    dock_pose.pose.position.z = 0.0;

    dock_pose.pose.orientation.z = std::sin(theta / 2.0);
    dock_pose.pose.orientation.w = std::cos(theta / 2.0);

    // Publish the detected pose
    dock_pose_pub_->publish(dock_pose);
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DockDetector>());
    rclcpp::shutdown();
    return 0;
}
