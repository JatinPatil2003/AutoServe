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

DockDetector::DockDetector(const rclcpp::Node::SharedPtr& node) {
    node_ = node;
    scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&DockDetector::scanCallback, this, std::placeholders::_1));

    cloud_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("docking_pc", 10);
    cloud_pub2_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("docking_aligned", 10);
    cloud_pub3_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("docking_pcd", 10);
    dock_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("dock_pose", 10);

    loadDockReferencePCD();
}

void DockDetector::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // detectDock(msg);
    std::lock_guard<std::mutex> lock(scan_cb_mutex_); 
    latest_scan_msg_ = msg;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    float angle = msg->angle_min;
    for (size_t i = 0; i < msg->ranges.size(); ++i, angle += msg->angle_increment) {
        float r = msg->ranges[i];
        float angle_deg = angle * 180.0 / M_PI;

        if ((angle_deg >= 135.0 && angle_deg <= 180.0) || (angle_deg >= -180.0 && angle_deg <= -135.0)) {
            if (std::isfinite(r) && r > msg->range_min && r < msg->range_max) {
                cloud->points.emplace_back(r * std::cos(angle), r * std::sin(angle), 0.0f);
            }
        }
    }
    sensor_msgs::msg::PointCloud2 output_cloud;
    pcl::toROSMsg(*cloud, output_cloud);
    output_cloud.header = msg->header;
    cloud_pub_->publish(output_cloud);

    pcl::toROSMsg(*dock_cloud_ref, output_cloud);
    output_cloud.header = latest_scan_msg_->header;
    cloud_pub3_->publish(output_cloud);

    latest_cloud_ = cloud;
}

void DockDetector::saveDockPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (pcl::io::savePCDFileBinary("/home/jatin/AutoServe/autoserve_docking/dock_pcd/dock_pcd.pcd", *cloud) == -1) {
        std::cerr << "❌ Failed to save PCD file\n";
    } else {
        std::cout << "✅ Saved dock_area.pcd with " << cloud->size() << " points.\n";
    }
}

void DockDetector::loadDockReferencePCD() {
    dock_cloud_ref = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/jatin/AutoServe/autoserve_docking/dock_pcd/dock_pcd.pcd", *dock_cloud_ref) == -1) {
        throw std::runtime_error("❌ Couldn't read file dock_pcd.pcd");
    }
    std::cout << "✅ Loaded dock_pcd.pcd with " << dock_cloud_ref->size() << " points.\n";
}

void DockDetector::detectDock(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    float angle = scan->angle_min;
    for (size_t i = 0; i < scan->ranges.size(); ++i, angle += scan->angle_increment) {
        float r = scan->ranges[i];
        float angle_deg = angle * 180.0 / M_PI;

        if ((angle_deg >= 145.0 && angle_deg <= 180.0) || (angle_deg >= -180.0 && angle_deg <= -145.0)) {
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
    // detectDockICP(cloud, scan);
}

bool DockDetector::detectDockICP(geometry_msgs::msg::Pose& dock_pose) {
    if (!latest_scan_msg_) {
        std::cerr << "❌ No latest scan message available.\n";
        return false;
    }
    if (!latest_cloud_) {
        std::cerr << "❌ No latest cloud available.\n";
        return false;
    }
    if (!dock_cloud_ref) {
        std::cerr << "❌ No dock reference cloud available.\n";
        return false;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
    
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(dock_cloud_ref);
    icp.setInputTarget(latest_cloud_);
    icp.setMaximumIterations(50);
    icp.align(*aligned);

    if (!icp.hasConverged()) {
        std::cerr << "❌ ICP did not converge.\n";
        return false;
    }

    if (icp.getFitnessScore() > 1e-4) {
        std::cerr << "❌ ICP Fitness Score Low.\n";
        return false;
    }  
    
    sensor_msgs::msg::PointCloud2 output_cloud;
    pcl::toROSMsg(*aligned, output_cloud);
    output_cloud.header = latest_scan_msg_->header;
    cloud_pub2_->publish(output_cloud);

    Eigen::Matrix4f tf = icp.getFinalTransformation();
    Eigen::Matrix4f extra_translation = Eigen::Matrix4f::Identity();
    extra_translation(0, 3) = -0.11f;  // Translate -0.3 meters along x-axis

    // Compose transformations: apply extra translation after ICP alignment
    tf = tf * extra_translation;

    // Extract position (translation) and orientation (rotation)
    float x = tf(0, 3);
    float y = tf(1, 3);
    float theta = std::atan2(tf(1, 0), tf(0, 0)); 
    // std::cout << "Score -> " << icp.getFitnessScore() << std::endl;
    // std::cout << "Dock Location: x->" << x << " y->" << y << " theta->" << theta << std::endl;
    geometry_msgs::msg::PoseStamped dock_pose_msg;
    dock_pose_msg.header = latest_scan_msg_->header;

    dock_pose_msg.pose.position.x = x;
    dock_pose_msg.pose.position.y = y;
    dock_pose_msg.pose.position.z = 0.0;

    dock_pose_msg.pose.orientation.z = std::sin(theta / 2.0);
    dock_pose_msg.pose.orientation.w = std::cos(theta / 2.0);


    dock_pose = dock_pose_msg.pose;

    // Publish the detected pose
    dock_pose_pub_->publish(dock_pose_msg);
    return true;
}


// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<rclcpp::Node>("dock_detector_node");
//     auto detector = std::make_shared<DockDetector>(node);
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }
