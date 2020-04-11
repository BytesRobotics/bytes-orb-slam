//
// Created by michael on 4/4/20.
//

#ifndef BR_SLAM_SYSTEM_H
#define BR_SLAM_SYSTEM_H

//Standard C++
#include <string>
#include <iostream>
#include <chrono>

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <image_geometry/stereo_camera_model.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped__struct.hpp>
#include <nav_msgs/msg/odometry__struct.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2/time.h>

#include "ORBExtractor.h"

class System {
public:
    explicit System(rclcpp::Node *node);
    // Process the given stereo frame. Images must be synchronized and rectified. Input images: grayscale (CV_8U).
    void Track(const cv::Mat &left_img, const cv::Mat &right_image, image_geometry::StereoCameraModel &stereo_camera_model,
          const std::shared_ptr<sensor_msgs::msg::Imu>& imu_data, const std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped>& altimeter,
          const std::shared_ptr<nav_msgs::msg::Odometry>& wheel_odom);
private:
    ORBExtractor orb_extractor_;

    /// Last Frame for the motion model
    std::shared_ptr<Frame> last_frame_;

    /// This will be for using the odom as the motion model
    tf2::BufferCore tf_buffer_;
    tf2_ros::TransformListener transform_listener_;
};


#endif //BR_SLAM_SYSTEM_H
