//
// Created by michael on 4/13/20.
//

#ifndef BR_SLAM_MAP_H
#define BR_SLAM_MAP_H

// Standard C++
#include <chrono>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <image_geometry/stereo_camera_model.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2/buffer_core.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>


// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

// Other
#include "Frame.h"

class Map{
public:
    /**
     * This creates a new map object, maps are individually finite based on a storage capacity with # of keyframes
     * Other parameters available such as
     * @param map_size this is the maximum number of frames that a map can contain, the tracking node will be responsible
     * for removing extraneous keyframes, routinely running the cleanup routines, and tracking the long
     * term motion of the robot including realizations, database searches, and both short and long term graph optimizations.
     */
    explicit Map(std::shared_ptr<rclcpp::Node> node, int map_size=1000, bool debug=false);

    std::shared_ptr<Frame> get_most_recent_keyframe();

    void add_keyframe(const std::shared_ptr<Frame>& frame, image_geometry::StereoCameraModel &stereo_camera_model);

private:

    /// Hold onto the main node pointer for logging and parameters
    std::shared_ptr<rclcpp::Node> node_;

    /// Parameters
    bool debug_, publish_odom_, broadcast_tf_;
    std::string odom_frame_id_, odom_child_frame_id;

    /// This will be for using the odom as the motion model
    tf2::BufferCore tf_buffer_;
    tf2_ros::TransformListener transform_listener_;

    /// This is the map that is build over over time
    std::vector<std::shared_ptr<Frame>> map_;
    //std::shared_ptr<Frame> base_frame_;

    /// Allow for important Trakcer related parameters to be dynamically configured
    rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

    /// Here we work through the entire map to get the current odom of the last keyframe
    /// Note that this only updates when a new keyframe is inserted to preserve computational efficiency
    void publish_tf_odom_(image_geometry::StereoCameraModel &stereo_camera_model, tf2::TimePoint time);
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    /// Standard odom nav message publisher
    void publish_odometry_(image_geometry::StereoCameraModel &stereo_camera_model, tf2::TimePoint time);
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    geometry_msgs::msg::Pose last_pose_msg_;
    double last_pub_time_; // used for getting the velocity for the odometry



    /// Main debug tool for observing maps
    void publish_pointcloud();
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;

};

#endif //BR_SLAM_MAP_H
