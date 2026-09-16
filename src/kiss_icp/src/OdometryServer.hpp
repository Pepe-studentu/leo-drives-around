// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#pragma once

// KISS-ICP
#include "kiss_icp/pipeline/KissICP.hpp"

// ROS 2
#include <deque>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/empty.hpp>
#include <string>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace kiss_icp_ros {

class OdometryServer : public rclcpp::Node {
public:
    /// OdometryServer constructor
    OdometryServer() = delete;
    explicit OdometryServer(const rclcpp::NodeOptions &options);

private:
    /// Declare ROS parameters and set the associated variables (in this class and in the provided
    /// config object)
    void initializeParameters(kiss_icp::pipeline::KISSConfig &config);

    /// Register new frame
    void RegisterFrame(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

    /// IMU callback for relative rotation prior
    void ImuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg);

    /// Stream the estimated pose to ROS
    void PublishOdometry(const Sophus::SE3d &kiss_pose, const std_msgs::msg::Header &header);

    /// Stream the debugging point clouds for visualization (if required)
    void PublishClouds(const std::vector<Eigen::Vector3d> &frame,
                       const std::vector<Eigen::Vector3d> &keypoints,
                       const std_msgs::msg::Header &header);
    void ResetService(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                      std::shared_ptr<std_srvs::srv::Empty::Response> response);

private:
    /// Tools for broadcasting TFs.
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
    bool invert_odom_tf_;
    bool publish_odom_tf_;
    bool publish_debug_clouds_;

    /// Data subscribers.
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub_;

    /// Motion priors
    bool use_imu_{true};
    bool use_wheel_odom_{false};
    double wheel_vx_{0.0};
    std::deque<sensor_msgs::msg::Imu> imu_queue_;
    std::mutex imu_mutex_;
    rclcpp::Time last_cloud_stamp_{0, 0, RCL_ROS_TIME};

    /// Data publishers.
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr frame_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr kpoints_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;

    /// Service servers.
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;

    /// KISS-ICP
    std::unique_ptr<kiss_icp::pipeline::KissICP> kiss_icp_;

    /// Global/map coordinate frame.
    std::string lidar_odom_frame_{"odom_lidar"};
    std::string base_frame_{};

    /// Covariance diagonal
    double position_covariance_;
    double orientation_covariance_;

    /// Short-Window (Rolling Window) ICP to prevent map poisoning during spins
    int sliding_window_size_{20};
    struct WindowFrame {
        Sophus::SE3d pose;
        std::vector<Eigen::Vector3d> points;
    };
    std::deque<WindowFrame> sliding_window_;
    double last_dt_{0.1};
};

}  // namespace kiss_icp_ros
