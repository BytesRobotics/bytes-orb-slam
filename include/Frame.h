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


/// See http://nghiaho.com/?p=936 for matrix framework speed comparisons

class Frame {
public:
    Frame(int max_features);
    ~Frame();

    /// Variable for storing the number of features currently in use

    /// Variable used for storing all the matches from the last track
    std::vector<std::array<cv::KeyPoint, 2>> match_features; //! The pairs of features matched with each other nx2
    std::vector<std::array<cv::Mat, 2>> match_descriptors; //! The rBRIEF descriptor for each feature nx2x32
    std::vector<double> match_distances; //! The distance between the descriptor (lower number means closer feature), nx2
    std::vector<double> match_xyz; //! The actual point in 3D space references from the left camera optical link, nx3

    void shrink_frame();

    void draw_frame(cv::Mat& img_left, cv::Mat& img_right);
};


#endif //BR_SLAM_FRAME_H
