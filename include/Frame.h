//
// Created by michael on 4/6/20.
//

#ifndef BR_SLAM_FRAME_H
#define BR_SLAM_FRAME_H

// Standard C++
#include <chrono>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>

//ROS
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>

/// See http://nghiaho.com/?p=936 for matrix framework speed comparisons

class Frame {
public:
    Frame(int max_features);

    /// Variable used for storing all the matches from the last track
    std::vector<std::array<cv::KeyPoint, 2>> match_features; //! The pairs of features matched with each other nx2
    std::vector<std::array<cv::Mat, 2>> match_descriptors; //! The rBRIEF descriptor for each feature nx2x32
    std::vector<double> match_distances; //! The distance between the descriptor (lower number means closer feature), nx1
    std::vector<cv::Point3d> match_xyz; //! The actual point in 3D space references from the left camera optical link, nx4

    /// Reduce memory footprint once all features are loaded and computed
    void shrink_frame();

    /// For debugging frames, this shows matches and disparity/depth of points
    void draw_frame(cv::Mat& img_left, cv::Mat& img_right);

    /// Transforms keypoints in R3 from the Frame in this object to the frame that is passed in
    void transform_keypoints(const std::shared_ptr<Frame> &old_frame, std::vector<cv::Point3d>& transformed_points);

    /// Include data to model the location of the frame relative to other sensor information including IMU, altimeter, and wheel odom
    /// this data can be used for storing keyframes and estimating the motion of the camera. Everything is implemented on rclcpp tf2
    tf2::Quaternion imu_orientation; //! Absolutely referenced orientation (Ric)
    tf2::Transform wheel_odom_to_camera; //! Odom references to starting position based on wheel movements (Toc)
    tf2::Vector3 altimeter_value; //! Only the z is used

    double latitude, longitude; //! GPS data corresponding to the frame
};


#endif //BR_SLAM_FRAME_H
