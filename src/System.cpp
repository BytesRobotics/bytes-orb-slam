//
// Created by michael on 4/4/20.
//

#include "System.h"

System::System(rclcpp::Node *node, const std::string& strVocFile): orb_extractor_(node->create_sub_node("orb")) {
    std::stringstream cv_version;
    cv_version << "OpenCV Version: " << CV_VERSION;
    RCLCPP_INFO(node->get_logger(), cv_version.str().c_str());
}

void System::Track(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp) {
//    orb_extractor(imLeft, mvKeysLeft, mDescriptorsLeft);
    //const int PATCH_SIZE = 31;
    //const int HALF_PATCH_SIZE = 15;
    //const int EDGE_THRESHOLD = 19;
//    auto detector = cv::ORB::create(800, 1.6f, 5, 19, 0, 2, cv::ORB::HARRIS_SCORE, 19, 30);

//    auto start = std::chrono::steady_clock::now();

//    detector->detect(imLeft, mvKeysLeft);
//    detector->compute(imLeft, mvKeysLeft, mDescriptorsLeft);
//
//    detector->detect(imRight, mvKeysRight);
//    detector->compute(imRight, mvKeysRight, mDescriptorsRight);
//
//    std::vector<std::vector<cv::KeyPoint>> matches;
//    std::vector<std::vector<cv::Mat>> match_descriptions;
//    std::vector<double> match_distances;
//    FindRelevantFeatures(mDescriptorsLeft, mvKeysLeft, mDescriptorsRight, mvKeysRight, matches, match_descriptions, match_distances, 0);
//
//    std::vector<int> match_disparities;
//    ComputeDisparity(matches, match_disparities);

//    cv::BFMatcher matcher(cv::NORM_L2, false);
//    std::vector< cv::DMatch > matches;
//    matcher.match(mDescriptorsLeft, mDescriptorsRight, matches);

//    auto end = std::chrono::steady_clock::now();

//    std::cout << "Elapsed time in milliseconds : " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;
//
//    std::cout << "Number of matches: " << matches.size() << std::endl;

    std::shared_ptr<Frame> new_frame = orb_extractor_.extract(imLeft, imRight);

//    DrawPoints(imLeft, imRight, matches, match_distances, match_disparities, false, false);


//    std::cout << timestamp << std::endl;
//    std::cout << mvKeysLeft.size() << std::endl;
//    std::cout << mDescriptorsLeft.row(0) << std::endl;
//    std::cout << "Match rate: " << static_cast<double>(matches.size())/mDescriptorsLeft.size[0] << std::endl << std::endl;


}


void System::FindRelevantFeatures(const cv::Mat& leftDescriptors, std::vector<cv::KeyPoint>& leftFeatures, cv::Mat& rightDescriptors,
                                    std::vector<cv::KeyPoint> &rightFeatures,  std::vector<std::vector<cv::KeyPoint>>& outputMatches,
                                    std::vector<std::vector<cv::Mat>>& outputDescriptors, std::vector<double>& outputDistance, float tolerance) {

    for(int i=0; i<leftFeatures.size(); i++){
        double minDist{150}; // Eventually this will be tunable
        std::vector<cv::KeyPoint> match_feature;
        std::vector<cv::Mat> match_description;
        double distance;
        for(int j=0; j<rightFeatures.size(); j++){
            // If the point in on the same line +- 2 pixels and has an x that fits with the epipolar geometry of the stereo setup then count it good (ie x in left > x in right)
            // ### we will need to handle point that are at the same x later, those wont have a calculable depth since their is no disparity
            if(leftFeatures.at(i).pt.x >= rightFeatures.at(j).pt.x && leftFeatures.at(i).pt.y >= rightFeatures.at(j).pt.y - tolerance && leftFeatures.at(i).pt.y <= rightFeatures.at(j).pt.y + tolerance){
                std::vector<cv::KeyPoint> feature_left{leftFeatures.at(i)};
                std::vector<cv::KeyPoint> feature_right{rightFeatures.at(j)};
                match_feature = {leftFeatures[i], rightFeatures[j]};
                distance = cv::norm(leftDescriptors.row(i), rightDescriptors.row(j),cv::NORM_HAMMING);
                if(distance < minDist) {
                    match_feature = {leftFeatures[i], rightFeatures[j]};
                    match_description = {leftDescriptors.row(i), rightDescriptors.row(j)};
                }
            }
        }
        if(!match_feature.empty()){
            // Good points are points that have a probable match in the other image
            outputMatches.push_back(match_feature);
            outputDescriptors.push_back(match_description);
            outputDistance.push_back(distance);

        }
    }
}

void System::ComputeDisparity(std::vector<std::vector<cv::KeyPoint>> &matches, std::vector<int>& disparities) {
    for(auto &match : matches){
        disparities.push_back(match[0].pt.x - match[1].pt.x);
    }
}

void System::DrawPoints(const cv::Mat& imLeft, const cv::Mat& imRight, std::vector<std::vector<cv::KeyPoint>> &matches, std::vector<double>& distances,
                        std::vector<int>& disparities, bool visualize_disparity, bool relative) {

    cv::Mat bgr_left_distances;
    cv::cvtColor(imLeft, bgr_left_distances, cv::COLOR_GRAY2BGR);

    cv::Mat bgr_right_distances;
    cv::cvtColor(imRight, bgr_right_distances, cv::COLOR_GRAY2BGR);

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
