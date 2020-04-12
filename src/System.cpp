//
// Created by michael on 4/4/20.
//

#include "System.h"

System::System(rclcpp::Node *node): orb_extractor_(node->create_sub_node("orb")), transform_listener_(tf_buffer_) {
    std::stringstream cv_version;
    cv_version << "OpenCV Version: " << CV_VERSION;
    RCLCPP_INFO(node->get_logger(), cv_version.str().c_str());
}

void System::Track(const cv::Mat &left_img, const cv::Mat &right_image, image_geometry::StereoCameraModel &stereo_camera_model,
                   const std::shared_ptr<sensor_msgs::msg::Imu>& imu_data, const std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped>& altimeter,
                   const std::shared_ptr<nav_msgs::msg::Odometry>& wheel_odom) {

    // Get the necessary transformations of the robot static body for computing everything for the new frame
    // Tbc
    geometry_msgs::msg::TransformStamped base_to_camera_msg = tf_buffer_.lookupTransform(stereo_camera_model.left().tfFrame(), wheel_odom->child_frame_id, tf2::TimePoint(std::chrono::seconds(0)));
    tf2::Stamped<tf2::Transform> base_to_camera;
    tf2::fromMsg(base_to_camera_msg, base_to_camera);

    // Data to create Ric
    geometry_msgs::msg::TransformStamped imu_to_camera_msg = tf_buffer_.lookupTransform(stereo_camera_model.left().tfFrame(), imu_data->header.frame_id, tf2::TimePoint(std::chrono::seconds(0)));
    tf2::Stamped<tf2::Transform> imu_to_camera;
    tf2::fromMsg(imu_to_camera_msg, imu_to_camera);

    // Create a new frame
    std::shared_ptr<Frame> new_frame = orb_extractor_.extract(left_img, right_image, stereo_camera_model);

    // Ric
    new_frame->imu_orientation = tf2::Quaternion(imu_data->orientation.x, imu_data->orientation.y, imu_data->orientation.z, imu_data->orientation.w)*imu_to_camera.getRotation();

    new_frame->wheel_odom_to_camera = tf2::Transform();
    new_frame->wheel_odom_to_camera.setOrigin(tf2::Vector3(wheel_odom->pose.pose.position.x, wheel_odom->pose.pose.position.y, wheel_odom->pose.pose.position.z));
    new_frame->wheel_odom_to_camera.setRotation(tf2::Quaternion(wheel_odom->pose.pose.orientation.x, wheel_odom->pose.pose.orientation.y, wheel_odom->pose.pose.orientation.z, wheel_odom->pose.pose.orientation.w));
    // Toc = Tob*Tbc
    new_frame->wheel_odom_to_camera = new_frame->wheel_odom_to_camera * base_to_camera; // Turn the wheel from odom->base_link to odom to left_camera_optical_link

    new_frame->altimeter_value = tf2::Vector3(0,0,altimeter->pose.pose.position.z);

    if(last_frame_ != nullptr){
        std::vector<cv::Point3d> transformed_points;
        new_frame->transform_keypoints(last_frame_, transformed_points);
        /// Initialize the bgr image variables for creating the debug views
        cv::Mat bgr_left_reprojected_pts;
        cv::cvtColor(left_img, bgr_left_reprojected_pts, cv::COLOR_GRAY2BGR);
        for(const auto& point : transformed_points){
            cv::circle(bgr_left_reprojected_pts, stereo_camera_model.left().project3dToPixel(point), 3, cv::Scalar(0, 0, 255));
        }
        for(auto point : new_frame->match_features){
            // New frame features are red
            cv::circle(bgr_left_reprojected_pts, point[0].pt, 3, cv::Scalar(0, 255, 0));
        }
        cv::resize(bgr_left_reprojected_pts, bgr_left_reprojected_pts, cv::Size(), 2, 2, cv::INTER_CUBIC);
        cv::imshow("reprojected points", bgr_left_reprojected_pts);
        cv::waitKey(1);
    } else {
        last_frame_ = new_frame;
    }
}