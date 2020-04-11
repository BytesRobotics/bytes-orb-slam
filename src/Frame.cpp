//
// Created by michael on 4/6/20.
//

#include "Frame.h"

Frame::Frame(int max_features) {
    match_features.reserve(max_features);
    match_descriptors.reserve(max_features);
    match_distances.reserve(max_features);
    match_xyz.reserve(max_features);
}

void Frame::draw_frame(cv::Mat &img_left, cv::Mat &img_right) {

    /// Initialize the bgr image variables for creating the debug views
    cv::Mat bgr_left_distances;
    cv::cvtColor(img_left, bgr_left_distances, cv::COLOR_GRAY2BGR);

    cv::Mat disparity_image;
    bgr_left_distances.copyTo(disparity_image);

    cv::Mat bgr_right_distances;
    cv::cvtColor(img_right, bgr_right_distances, cv::COLOR_GRAY2BGR);

    /// Merge the left and right images together to get combined debug view
    cv::Mat distance_image;
    cv::hconcat(bgr_left_distances, bgr_right_distances, distance_image);

    /// Draw the matches and use color to indicate confidence in match
    for(unsigned int i=0; i<match_features.size(); i++){
        // left image
        cv::circle(distance_image, match_features[i][0].pt, 3, cv::Scalar(0, 255 - match_distances[i], match_distances[i]));
        // right image
        cv::Point2f right_point = match_features[i][1].pt;
        right_point.x += img_left.size().width; // shift the point since we shifted the image
        cv::circle(distance_image, right_point, 3, cv::Scalar(0, 255 - match_distances[i], match_distances[i]));
        //line connecting match
        cv::line(distance_image, match_features[i][0].pt, right_point, cv::Scalar(0, 255 - match_distances[i], match_distances[i]));
    }

    cv::resize(distance_image, distance_image, cv::Size(), 2, 2, cv::INTER_CUBIC);

    cv::imshow("Matches", distance_image);
    cv::waitKey(1);


    for(unsigned int i=0; i<match_features.size(); i++){
        cv::circle(disparity_image, match_features[i][0].pt, 3, cv::Scalar(match_xyz[i].z*20, 0, 255 - match_xyz[i].z*20));
    }

    cv::resize(disparity_image, disparity_image, cv::Size(), 2, 2, cv::INTER_CUBIC);

    cv::imshow("Disparities", disparity_image);
    cv::waitKey(1);
}

void Frame::shrink_frame() {
    match_features.shrink_to_fit();
    match_descriptors.shrink_to_fit();
    match_distances.shrink_to_fit();
    match_xyz.shrink_to_fit();
}

std::shared_ptr<std::vector<cv::Point3d>> Frame::transform_keypoints(const std::shared_ptr<Frame> &transforming_frame) {
    std::shared_ptr<std::vector<cv::Point3d>> transformed_points;
    transformed_points->reserve(match_xyz.size());
    for(const auto& point : match_xyz){
//        tf2::Quaternion d_orientation =  transforming_frame->imu_orientation*imu_orientation.inverse(); // Get the change in the rotation between the frames
        tf2::Transform d_transform = transforming_frame->wheel_odom*wheel_odom.inverse(); // Change in rotation/translation according to wheel odom
        auto new_point = d_transform*tf2::tf2Vector4(point.x, point.y, point.z, 1); // temporarily storing the var in tf2::transform
        transformed_points->push_back(cv::Point3d(new_point.x(), new_point.y(), new_point.z()));
    }
    return transformed_points;
}
