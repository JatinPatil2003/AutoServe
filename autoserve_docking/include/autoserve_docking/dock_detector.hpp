#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl_conversions/pcl_conversions.h>
#include <mutex>

class DockDetector {
public:
    explicit DockDetector(const rclcpp::Node::SharedPtr& node);
    bool detectDockICP(geometry_msgs::msg::Pose& dock_pose);
    
private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void detectDock(const sensor_msgs::msg::LaserScan::SharedPtr& scan);
    void saveDockPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void loadDockReferencePCD();

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr dock_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub2_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub3_;

    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_msg_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr dock_cloud_ref;

    std::mutex scan_cb_mutex_;
};
