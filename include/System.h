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

#include "ORBExtractor.h"

class System {
public:
    explicit System(rclcpp::Node *node, const std::string& strVocFile);

    // Process the given stereo frame. Images must be synchronized and rectified. Input images: grayscale (CV_8U).
    void Track(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp);

    // Use Epipolar geometry of cameras to reduce the number of keypoints we care about
    void FindRelevantFeatures(const cv::Mat& leftDescriptors, std::vector<cv::KeyPoint>& leftFeatures, cv::Mat& rightDescriptors,
            std::vector<cv::KeyPoint> &rightFeatures,  std::vector<std::vector<cv::KeyPoint>>& outputMatches,
            std::vector<std::vector<cv::Mat>>& outputDescriptors, std::vector<double>& outputDistance,  float tolerance=0);

    void ComputeDisparity(std::vector<std::vector<cv::KeyPoint>>& matches, std::vector<int>& disparities);

    void DrawPoints(const cv::Mat& imLeft, const cv::Mat& imRight, std::vector<std::vector<cv::KeyPoint>>& matches, std::vector<double>& distances, std::vector<int>& disparities, bool visualize_disparity = false, bool relative = true);

private:

    ORBExtractor orb_extractor_;
    std::vector<cv::KeyPoint> mvKeysLeft, mvKeysRight;
    cv::Mat mDescriptorsLeft, mDescriptorsRight;
};


#endif //BR_SLAM_SYSTEM_H
