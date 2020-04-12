//
// Created by michael on 4/5/20.
// Property of Bytes Robotics, LLC
//

#ifndef BR_SLAM_ORBEXTRACTOR_H
#define BR_SLAM_ORBEXTRACTOR_H

// Standard C++
#include <chrono>
#include <thread>
#include <iostream>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_geometry/stereo_camera_model.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>


// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core/cvstd.hpp>

// Other
#include "Frame.h"


/**
 * The ORB Extractor class is an easy interface that allows stereo images to be processed with access to tools
 * for debugging and optimizing the ORB parameters. The typical output from the extractor is a nx3 cv::Mat that
 * contains 3D coordinates references from the left camera and a  nx32 list of descriptor for each point.
 *
 * The following represent the coordinate system used for extracting to ORB features in 3D from a stereo pair. Basically
 * everything is referenced from the left cameras optical center
 *
 *\        --------* /            /
 * \      |   x    /            /
 *  \    z|      \/            /
 *   \____|_____/ \__________/
 *  |          |  |          |
 *  |          |  |          |
 *  |   left   |  |   right  |
 *  |   camera |  |   camera |
 *
 *   y is the unseen axis and is more positive when moving down the left camera's plane
 *   x is more positive when moving more toward the right
 *   z is more positive when the pixel is farther away from the camera
 *
 *  the units of x, y, and z are provided in meters and therefore require camera info to compute the z = focal_length*baseline/disparity
 *
 *  This class is also ROS2 native and takes advantage of the ROS2 parameter server to make all parameters dynamically reconfigurable
 *  through the ROS2 parameter API
 */
class ORBExtractor{
public:
    /**
     * Initialize the ORB extractor and
     *
     * @param node A reference to the byes SLAM node
     * @param nfeatures The maximum number of features to retain.
     * @param Pyramid decimation ratio, greater than 1. scaleFactor==2 means the classical pyramid, where each next
     * level has 4x less pixels than the previous, but such a big scale factor will degrade feature matching scores
     * dramatically. On the other hand, too close to 1 scale factor will mean that to cover certain scale range you
     * will need more pyramid levels and so the speed will suffer.
     * @param nlevels The number of pyramid levels. The smallest level will have linear size equal to
     * input_image_linear_size/pow(scaleFactor, nlevels - firstLevel).
     * @param edgeThreshold This is size of the border where the features are not detected. It should roughly match the patchSize parameter.
     * @param firstLevel The level of pyramid to put source image to. Previous layers are filled with upscaled source image.
     * @param WTA_K The number of points that produce each element of the oriented BRIEF descriptor. The default value 2
     * means the BRIEF where we take a random point pair and compare their brightnesses, so we get 0/1 response. Other
     * possible values are 3 and 4. For example, 3 means that we take 3 random points (of course, those point coordinates
     * are random, but they are generated from the pre-defined seed, so each element of BRIEF descriptor is computed
     * deterministically from the pixel rectangle), find point of maximum brightness and output index of the winner (0, 1 or 2).
     * Such output will occupy 2 bits, and therefore it will need a special variant of Hamming distance, denoted as NORM_HAMMING2
     * (2 bits per bin). When WTA_K=4, we take 4 random points to compute each bin (that will also occupy 2 bits with
     * possible values 0, 1, 2 or 3).
     * @param scoreType The default HARRIS_SCORE means that Harris algorithm is used to rank features (the score is
     * written to KeyPoint::score and is used to retain best nfeatures features); FAST_SCORE is alternative value of
     * the parameter that produces slightly less stable keypoints, but it is a little faster to compute.
     * @param patchSize size of the patch used by the oriented BRIEF descriptor. Of course,
     * on smaller pyramid layers the perceived image area covered by a feature will be larger.
     * @param fastThreshold the fast threshold
     * @param debug enables printing the time stamps of the algorithm and other meta information
     */
    ORBExtractor(std::shared_ptr<rclcpp::Node> node, int nfeatures=1000, float scaleFactor=1.5f, int nlevels=5, int edgeThreshold=19, int firstLevel=0,
                    int WTA_K=2, cv::ORB::ScoreType scoreType=cv::ORB::FAST_SCORE, int patchSize=19, int fastThreshold=30, bool debug=false,
                    float tolerance=1, int matchThreshold=30);

/**
     * This is the main function to be called for converting two grayscale images into
     * a matrix of x,y,z points + descriptions
     *
     * @param imLeft Left grayscale image
     * @param imRight Right grayscale image
     */
    std::shared_ptr<Frame> extract(const cv::Mat &img_left, const cv::Mat &img_right, image_geometry::StereoCameraModel& stereo_camera_model);

private:
    /// For this ORB extractor we use the default Opencv cv::ORB algorithm (Eventually implement cv::cuda::ORB)
    cv::Ptr<cv::ORB> orb_;

    /// Allow for important ORB related parameters to be dynamically configured
    rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

    /// Hold onto the main node pointer for logging and parameters
    std::shared_ptr<rclcpp::Node> node_;

    /// State variables that update each time extract is called
    std::vector<cv::KeyPoint> left_key_pts_, right_key_pts_; //! key features for each image prior to cross check (n)
    cv::Mat left_descriptors_, right_descriptors_; //! descriptors for each key feature (nx32)

    /// Variables for storing all the parameters from the ROS2
    int n_features_;
    float scale_factor_;
    int n_levels_;
    int edge_threshold_;
    int first_level_;
    int WTA_K_;
    cv::ORB::ScoreType score_type_;
    int patch_size_;
    int fast_threshold_;
    bool debug_;
    double match_threshold_; // The maximum distance between two matching points
    float match_tolerance_; // The vertical space to check for points

    /** Use Epipolar geometry of cameras to reduce the number of keypoints we care about
     * @param frame The frame that stores all the match information can be used for tracking the motion of the camera
     * @param threshold The match threshold to determine if the point is close enough to the one in the other image
     **/
    void find_relevant_features_(const std::shared_ptr<Frame>& frame);

    void compute_xyz_(const std::shared_ptr<Frame>& frame, image_geometry::StereoCameraModel& stereo_camera_model);

    /// Variables for monitoring the time of each step in the ORB extractor
    std::chrono::steady_clock::time_point orb_start_, orb_stop_, match_start_, match_stop_, xyz_start_, xyz_stop_;
    double aggregate_total_time_;
    int iterations_;

    /// We can publish a point cloud for visualization in RVIZ during debug mode with this publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;



};

#endif //BR_SLAM_ORBEXTRACTOR_H
