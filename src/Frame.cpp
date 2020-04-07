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
    cv::Mat bgr_left_distances;
    cv::cvtColor(img_left, bgr_left_distances, cv::COLOR_GRAY2BGR);

    cv::Mat bgr_right_distances;
    cv::cvtColor(img_right, bgr_right_distances, cv::COLOR_GRAY2BGR);

    cv::Mat bgr_left_disparity;
    bgr_left_distances.copyTo(bgr_left_disparity);

    cv::Mat bgr_right_disparity;
    bgr_right_distances.copyTo(bgr_right_disparity);


    //Normalize distances to 0.0-1.0 and multiple by 255 to show how closely related the points are for visualization
    if(relative){
        double distance_min = 99999;
        double distance_max = 0;
        double distance_mean = 0;
        for(auto & distance : distances){
            if(distance > distance_max){
                distance_max = distance;
            }
            if(distance  < distance_min){
                distance_min = distance;
            }
            distance_mean += distance;
        }
        distance_mean /= distances.size();
        std::cout << "Max dist: " << distance_max << " Min dist: " << distance_min << " Mean dist: " << distance_mean << std::endl;
        for(int i=0; i<distances.size();i++){
            auto norm_dist = ((distances[i] - distance_min)/(distance_max-distance_min))*255;
            cv::circle(bgr_left_distances, matches[i][0].pt, 3, cv::Scalar(0, 255 - norm_dist, norm_dist));
            cv::circle(bgr_right_distances, matches[i][1].pt, 3, cv::Scalar(0, 255 - norm_dist, norm_dist));
        }
    } else {
        for(int i=0; i<matches.size(); i++){
            cv::circle(bgr_left_distances, matches[i][0].pt, 3, cv::Scalar(0, 255 - distances[i], distances[i]));
            cv::circle(bgr_right_distances, matches[i][1].pt, 3, cv::Scalar(0, 255 - distances[i], distances[i]));
        }
    }

    cv::Mat distance_image;
    cv::hconcat(bgr_left_distances, bgr_right_distances, distance_image);

    cv::imshow("Distances", distance_image);
    cv::waitKey(1);

    //Normalize disparities to 0.0-1.0 and multiple by 255 to show how closely related the points are for visualization
    if(relative){
        double disparity_min = 99999;
        double disparity_max = 0;
        double mean_disp = 0;
        for(auto & disparity : disparities){
            if(disparity > disparity_max){
                disparity_max = disparity;
            }
            if(disparity  < disparity_min){
                disparity_min = disparity;
            }
            mean_disp += disparity;
        }
        mean_disp/=disparities.size();
        std::cout << "Max disp: " << disparity_max << " Min disp: " << disparity_min << " Mean disp: " << mean_disp << std::endl;
        for(int i=0; i<disparities.size();i++){
            auto norm_disp = ((disparities[i] - disparity_min)/(disparity_max-disparity_min))*255;
            cv::circle(bgr_left_disparity, matches[i][0].pt, 3, cv::Scalar(norm_disp, 0, 255 - norm_disp));
            cv::circle(bgr_right_disparity, matches[i][1].pt, 3, cv::Scalar(norm_disp, 0, 255 - norm_disp));
        }
    } else {
        for(int i=0; i<matches.size(); i++){
            cv::circle(bgr_left_disparity, matches[i][0].pt, 3, cv::Scalar(disparities[i]*10, 0, 255-disparities[i]*10));
            cv::circle(bgr_right_disparity, matches[i][1].pt, 3, cv::Scalar(disparities[i]*10, 0, 255-disparities[i]*10));
        }
    }

    cv::Mat disparity_image;
    cv::hconcat(bgr_left_disparity, bgr_right_disparity, disparity_image);

    cv::imshow("Disparities", disparity_image);
    cv::waitKey(1);
}

void Frame::shrink_frame() {
    match_features.shrink_to_fit();
    match_descriptors.shrink_to_fit();
    match_distances.shrink_to_fit();
    match_xyz.shrink_to_fit();
}
