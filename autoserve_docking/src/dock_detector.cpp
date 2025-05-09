#include "autoserve_docking/dock_detector.hpp"
#include <pcl/common/intersections.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>
#include <Eigen/Dense>
#include <cmath>

DockDetector::DockDetector() : Node("dock_detector") {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&DockDetector::scanCallback, this, std::placeholders::_1));

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", 10);
    cloud_pub2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud2", 10);
    dock_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("dock_pose", 10);
}

void DockDetector::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    detectDock(msg);
}

void DockDetector::detectDock(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    float angle = scan->angle_min;
    for (size_t i = 0; i < scan->ranges.size(); ++i, angle += scan->angle_increment) {
        float r = scan->ranges[i];

        // Convert angle to degrees for easier logic (optional)
        float angle_deg = angle * 180.0 / M_PI;

        // Keep only points in the 120° sector centered at 180°
        if ((angle_deg >= 145.0 && angle_deg <= 180.0) || (angle_deg >= -180.0 && angle_deg <= -145.0)) {
            if (std::isfinite(r) && r > scan->range_min && r < scan->range_max) {
                cloud->points.emplace_back(r * std::cos(angle), r * std::sin(angle), 0.0f);
            }
        }
    }

    sensor_msgs::msg::PointCloud2 output_cloud;
    pcl::toROSMsg(*cloud, output_cloud);
    output_cloud.header.stamp = scan->header.stamp;
    output_cloud.header.frame_id = scan->header.frame_id;
    cloud_pub_->publish(output_cloud);

    // Apply RANSAC to find two dominant lines
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_line(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_line);
    ransac.setDistanceThreshold(0.1);
    ransac.computeModel();

    std::vector<int> inliers1;
    ransac.getInliers(inliers1);

    if (inliers1.size() < 10) return;

    Eigen::VectorXf coeff1;
    ransac.getModelCoefficients(coeff1);

    // Remove inliers of first line
    pcl::PointCloud<pcl::PointXYZ>::Ptr remaining(new pcl::PointCloud<pcl::PointXYZ>());
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        if (std::find(inliers1.begin(), inliers1.end(), i) == inliers1.end()) {
            remaining->points.push_back(cloud->points[i]);
        }
    }

    // sensor_msgs::msg::PointCloud2 output_cloud;
    pcl::toROSMsg(*remaining, output_cloud);
    output_cloud.header.stamp = scan->header.stamp;
    output_cloud.header.frame_id = scan->header.frame_id;
    cloud_pub2_->publish(output_cloud);

    // Fit second line on remaining points
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_line2(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(remaining));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac2(model_line2);
    ransac2.setDistanceThreshold(0.1);
    ransac2.computeModel();

    std::vector<int> inliers2;
    ransac2.getInliers(inliers2);

    if (inliers2.size() < 10) return;

    Eigen::VectorXf coeff2;
    ransac2.getModelCoefficients(coeff2);

    // Compute intersection point
    Eigen::Vector2f p1(coeff1[0], coeff1[1]);
    Eigen::Vector2f d1(coeff1[3], coeff1[4]);

    Eigen::Vector2f p2(coeff2[0], coeff2[1]);
    Eigen::Vector2f d2(coeff2[3], coeff2[4]);

    Eigen::Matrix2f A;
    A << d1, -d2;
    if (std::abs(A.determinant()) < 0.1) return; // parallel lines

    Eigen::Vector2f t = A.inverse() * (p2 - p1);
    Eigen::Vector2f intersection = p1 + t[0] * d1;

    // Compute bisector for orientation
    Eigen::Vector2f dir1 = d1.normalized();
    Eigen::Vector2f dir2 = d2.normalized();
    Eigen::Vector2f bisector = (dir1 + dir2).normalized();

    // Publish the dock pose
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = scan->header.stamp;
    pose.header.frame_id = scan->header.frame_id;
    pose.pose.position.x = intersection.x();
    pose.pose.position.y = intersection.y();
    pose.pose.position.z = 0;

    float previous_yaw = 0.0;  // Store previous yaw value
    float yaw_filter_coefficient = 0.1;  // Smoothing factor (higher value = more smoothing)

    float yaw = std::atan2(bisector.y(), bisector.x());
    yaw = previous_yaw * (1.0 - yaw_filter_coefficient) + yaw * yaw_filter_coefficient;
    previous_yaw = yaw;  // Update the previous yaw value
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);

    dock_pose_pub_->publish(pose);
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DockDetector>());
    rclcpp::shutdown();
    return 0;
}
