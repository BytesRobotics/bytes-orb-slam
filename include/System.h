//
// Created by michael on 4/4/20.
//

#ifndef BR_SLAM_SYSTEM_H
#define BR_SLAM_SYSTEM_H

#include <string>
#include <iostream>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
//#include <opencv2/cudafeatures2d.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_geometry/stereo_camera_model.h>


#include "ORBExtractor.h"

class System {
public:
    explicit System(rclcpp::Node *node, const std::string& strVocFile);

    // Process the given stereo frame. Images must be synchronized and rectified. Input images: grayscale (CV_8U).
    void Track(const cv::Mat &left_img, const cv::Mat &right_image, image_geometry::StereoCameraModel& stereo_camera_model, const double &timestamp);

private:

    ORBExtractor orb_extractor_;
};


#endif //BR_SLAM_SYSTEM_H
