//
// Created by michael on 4/5/20.
// Property of Bytes Robotics, LLC
//

#include "ORBExtractor.h"

#include <utility>

using namespace cv;
using namespace std::chrono_literals;

// Constructor
ORBExtractor::ORBExtractor(std::shared_ptr<rclcpp::Node>  node, int nfeatures, float scaleFactor, int nlevels, int edgeThreshold, int firstLevel,
                           int WTA_K, ORB::ScoreType scoreType, int patchSize, int fastThreshold, bool debug, float tolerance,
                           int matchThreshold): node_(std::move(node)) {

    node_->declare_parameter(node_->get_sub_namespace() + ".nfeatures", nfeatures);
    n_features_ = nfeatures;
    node_->declare_parameter(node_->get_sub_namespace() + ".scaleFactor", scaleFactor);
    scale_factor_ = scaleFactor;
    node_->declare_parameter(node_->get_sub_namespace() + ".nlevels", nlevels);
    n_levels_ = nlevels;
    node_->declare_parameter(node_->get_sub_namespace() + ".edgeThreshold", edgeThreshold);
    edge_threshold_ = edge_threshold_;
    node_->declare_parameter(node_->get_sub_namespace() + ".firstLevel", firstLevel);
    first_level_ = firstLevel;
    node_->declare_parameter(node_->get_sub_namespace() + ".WTA_K", WTA_K);
    WTA_K_ = WTA_K;
    node_->declare_parameter(node_->get_sub_namespace() + ".scoreType", 1); //1 is FAST, 0 is HARRIS
    score_type_ = scoreType;
    node_->declare_parameter(node_->get_sub_namespace() + ".patchSize", patchSize);
    patch_size_ = patchSize;
    node_->declare_parameter(node_->get_sub_namespace() + ".fastThreshold", fastThreshold);
    fast_threshold_ = fastThreshold;
    node_->declare_parameter(node_->get_sub_namespace() + ".debug", debug);
    debug_ = debug;
    node_->declare_parameter(node_->get_sub_namespace() + ".tolerance", tolerance); //Stereo tolerance for image matching
    match_tolerance_ = tolerance;
    node_->declare_parameter(node_->get_sub_namespace() + ".matchThreshold", matchThreshold); //Stereo threshold for image matching
    match_threshold_ = matchThreshold;

    orb_ = cv::ORB::create(n_features_, scale_factor_, n_levels_, edge_threshold_, first_level_, WTA_K_, score_type_, patch_size_, fast_threshold_);

    // Update parameters dynamically
    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_);

    // Function definition for the async parameters callback
    // From https://github.com/ros2/demos/blob/master/demo_nodes_cpp/src/parameters/parameter_events_async.cpp
    auto on_parameter_event_callback = [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
            {
                std::stringstream ss;
                ss << "[" + node_->get_sub_namespace() + "]: \nParameter event:\n changed parameters:";
                for (auto & changed_parameter : event->changed_parameters) {
                    ss << "\n  " << changed_parameter.name;
                    if(changed_parameter.name == node_->get_sub_namespace() + ".nfeatures"){
                        n_features_ = changed_parameter.value.integer_value;
                    } else if(changed_parameter.name == node_->get_sub_namespace() + ".scaleFactor"){
                        scale_factor_ = changed_parameter.value.double_value;
                    } else if(changed_parameter.name == node_->get_sub_namespace() + ".nlevels"){
                        n_levels_ = changed_parameter.value.integer_value;
                    } else if(changed_parameter.name == node_->get_sub_namespace() + ".edgeThreshold"){
                        edge_threshold_ = changed_parameter.value.integer_value;
                    } else if(changed_parameter.name == node_->get_sub_namespace() + ".firstLevel"){
                        first_level_ = changed_parameter.value.integer_value;
                    } else if(changed_parameter.name == node_->get_sub_namespace() + ".WTA_K"){
                        WTA_K_ = changed_parameter.value.integer_value;
                    } else if(changed_parameter.name == node_->get_sub_namespace() + ".scoreType"){
                        score_type_ = ORB::ScoreType(changed_parameter.value.integer_value);
                    } else if(changed_parameter.name == node_->get_sub_namespace() + ".patchSize"){
                        patch_size_ = changed_parameter.value.integer_value;
                    } else if(changed_parameter.name == node_->get_sub_namespace() + ".fastThreshold"){
                        fast_threshold_ = changed_parameter.value.integer_value;
                    } else if(changed_parameter.name == node_->get_sub_namespace() + ".debug"){
                        debug_ = changed_parameter.value.bool_value;
                    } else if(changed_parameter.name == node_->get_sub_namespace() + ".tolerance"){
                        match_tolerance_ = changed_parameter.value.double_value;
                    } else if(changed_parameter.name == node_->get_sub_namespace() + ".matchThreshold"){
                        match_threshold_ = changed_parameter.value.double_value;
                    }
                }
                ss << "\n";
                RCLCPP_DEBUG(node_->get_logger(), ss.str().c_str());
                orb_ = cv::ORB::create(n_features_, scale_factor_, n_levels_, edge_threshold_, first_level_, WTA_K_, score_type_, patch_size_, fast_threshold_);
            };

    // Setup callback for changes to parameters.
    parameter_event_sub_ = parameters_client_->on_parameter_event(on_parameter_event_callback);

    // Initialize some variables for debugging use
    aggregate_total_time_ = 0;
    iterations_ = 0;

}

std::shared_ptr<Frame> ORBExtractor::extract(const cv::Mat &img_left, const cv::Mat &img_right, image_geometry::StereoCameraModel& stereo_camera_model) {
    if(debug_){orb_start_ = std::chrono::steady_clock::now();iterations_++;}
    orb_->detect(img_left, left_key_pts_);
    orb_->compute(img_left, left_key_pts_, left_descriptors_);
    orb_->detect(img_right, right_key_pts_);
    orb_->compute(img_right, right_key_pts_, right_descriptors_);
    if(debug_){orb_stop_ = std::chrono::steady_clock::now();}

    // This is the frame that will contain all match info for the two ORB extracted images
    auto frame = std::make_shared<Frame>(left_key_pts_.size());

    if(debug_){match_start_ = std::chrono::steady_clock::now();}
    find_relevant_features_(frame);
    if(debug_){match_stop_ = std::chrono::steady_clock::now();}

    if(debug_){xyz_start_ = std::chrono::steady_clock::now();}
    compute_xyz_(frame, stereo_camera_model);
    if(debug_){xyz_stop_ = std::chrono::steady_clock::now();}

    frame->shrink_frame();

    if(debug_){
        // Draw the debug images
        frame->draw_frame(const_cast<Mat &>(img_left), const_cast<Mat &>(img_right));
        double orb_time = std::chrono::duration_cast<std::chrono::microseconds>(match_stop_ - match_start_).count()/1000.0;
        double match_time = std::chrono::duration_cast<std::chrono::microseconds>(orb_stop_ - orb_start_).count()/1000.0;
        double xyz_time = std::chrono::duration_cast<std::chrono::microseconds>(xyz_stop_ - xyz_start_).count()/1000.0;
        double total_time = orb_time + match_time + xyz_time;
        aggregate_total_time_ += total_time;

        //Print out timing for the ORB round
        std::stringstream ss;
        ss << "ORB Diagnostics: iteration #" << iterations_ << std::endl;
        ss << "Time to complete ORB detect and compute features for left and right images: " << orb_time << " ms" << std::endl;
        ss << "Time to complete feature matching: " << match_time << " ms" << std::endl;
        ss << "Time to convert image points to xyz: " << xyz_time << " ms" << std::endl;
        ss << "Total time: " << total_time << " ms" << std::endl;
        ss << "Average total time: " << aggregate_total_time_/iterations_ << " ms" << std::endl;
        ss << "Left image feature match rate: " << static_cast<double>(frame->match_features.size())/left_key_pts_.size() << std::endl;
        RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
    }

    return frame;
}

void ORBExtractor::find_relevant_features_(const std::shared_ptr<Frame>& frame){
    for(unsigned int i=0; i<left_key_pts_.size(); i++){
        int match = -1;
        double distance = 0;
        for(unsigned int j=0; j<right_key_pts_.size(); j++){
            // If the point in on the same line +- 2 pixels and has an x that fits with the epipolar geometry of the stereo setup then count it good (ie x in left > x in right)
            // ### we will need to handle point that are at the same x later, those wont have a calculable depth since their is no disparity
            double min_match = match_threshold_; //this is how we used the feature with the least distance from the point, stores minimum distance
            if(left_key_pts_.at(i).pt.x >= right_key_pts_.at(j).pt.x && left_key_pts_.at(i).pt.y >= right_key_pts_.at(j).pt.y - match_tolerance_
            && left_key_pts_.at(i).pt.y <= right_key_pts_.at(j).pt.y + match_tolerance_){
                distance = cv::norm(left_descriptors_.row(i), right_descriptors_.row(j),cv::NORM_HAMMING);
                if(distance < min_match) {
                    min_match = distance;
                    match = static_cast<int>(j);
                }
            }
        }
        // Add a new value into the frame if a match was found
        if(match != -1){
            frame->match_distances.push_back(distance);
            frame->match_features.push_back({left_key_pts_[i], right_key_pts_[match]});
            frame->match_descriptors.push_back({left_descriptors_.row(i), right_descriptors_.row(match)});
        }
    }
}

void ORBExtractor::compute_xyz_(const std::shared_ptr<Frame>& frame, image_geometry::StereoCameraModel& stereo_camera_model) {
    frame->match_xyz.resize(frame->match_features.size());
    for(unsigned int i=0; i<frame->match_features.size(); i++){
        float disparity = frame->match_features[i][0].pt.x - frame->match_features[i][1].pt.x;
        stereo_camera_model.projectDisparityTo3d(frame->match_features[i][0].pt, disparity, frame->match_xyz[i]);
    }
}