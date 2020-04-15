//
// Created by michael on 4/10/20.
//

#ifndef BR_SLAM_TRACKING_H
#define BR_SLAM_TRACKING_H

// Standard C++
#include <chrono>
#include <iostream>
#include <math.h>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <image_geometry/stereo_camera_model.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2/buffer_core.h>

// OpenCv
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>


// Other
#include "Frame.h"
#include "Map.h"

/**
 * Tracking class manages maps by taking in frames, determining when to create keyframes, cleaning up keyframes
 * and managing the motion model so an error can be estimated after motion is received by odometry. The existing implementation
 * just uses wheel odometry but will eventually fuse in the altimeter and IMU in a more effective way that
 * is more productive than just using them for augmented frame search fo loop closure and relocalization. This is what
 * manages the graph and the main system maps and will implment our nonlinear least squares optimizer and map point filtering algorithms.
 */

class Tracker{
public:
    /**
     * This handles the addition and subtraction of keyframes within a map object and manages the pose graph. This is the
     * primary front end node to the SLAM algorithm. Once frames are processed here a new graph optimization may be called in which
     * case the local map will be run through a nonlinear least squares optimizer
     * @param node A subnode for the tracker
     * @param frame_rigidity This is the number of features that must exist in the last frame. A smaller number the great the likelyhood that a frame is
     * not linked to the previous one and that the optimization is not performed to the best ability in the long run. Decreasing
     * this parameter increases the number of keyframes stored in the map. Note that everytime a new frame is added
     * a small optimization framework is run to reduce drift between keyframe collection.
     */
    Tracker(std::shared_ptr<rclcpp::Node> node, int frame_rigidity, bool debug=false, int min_coord_dist=5, int min_feature_dist=1000, int tracking_period=1);

    /// Update odometry and potentially add a new keyframe to the map
    void track_with_new_frame(const std::shared_ptr<Frame>& new_frame, image_geometry::StereoCameraModel &stereo_camera_model, const cv::Mat &new_img);

    /// This is the main function for debugging the short run optimizer and frame tracking
    void show_optical_flow(const cv::Mat &new_img, std::vector<cv::Point3d>& transformed_points, image_geometry::StereoCameraModel &stereo_camera_model);

private:

    /// Hold onto the main node pointer for logging and parameters
    std::shared_ptr<rclcpp::Node> node_;

    /// Allow for important Trakcer related parameters to be dynamically configured
    rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

    /// The 3D point matches between the two most recent frames
    std::vector<std::array<cv::Point3d, 2>> matches_; // 3D points that match
    // For visualizing and computing errors in the odometry source
    std::vector<int> transformed_point_mapping_;
    cv::RNG rng_; // Used for generating random distributions for search


    /// Dyanmic configuration variable
    int frame_rigidity_, min_coord_dist_, min_feature_dist_, tracking_period_;
    bool debug_;

    /// Holds the last frame that was passed into the track to help with optical flow
    std::shared_ptr<Frame> last_frame_;

    /// Timing variables for algorithm analysis
    std::chrono::steady_clock::time_point match_start_, match_stop_;
    int iterations_;
    double aggregate_total_time_;
};

#endif //BR_SLAM_TRACKING_H
