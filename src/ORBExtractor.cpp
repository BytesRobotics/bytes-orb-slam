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
                           int WTA_K, ORB::ScoreType scoreType, int patchSize, int fastThreshold, bool debug, int tolerance,
                           int matchThreshold): node_(std::move(node)) {

    orb_->create(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize, fastThreshold);

    node_->declare_parameter(node_->get_sub_namespace() + ".nfeatures", nfeatures);
    n_features_ = nfeatures;
    node_->declare_parameter(node_->get_sub_namespace() + ".scaleFactor", scaleFactor);
    node_->declare_parameter(node_->get_sub_namespace() + ".nlevels", nlevels);
    node_->declare_parameter(node_->get_sub_namespace() + ".edgeThreshold", edgeThreshold);
    node_->declare_parameter(node_->get_sub_namespace() + ".firstLevel", firstLevel);
    node_->declare_parameter(node_->get_sub_namespace() + ".WTA_K", WTA_K);
    node_->declare_parameter(node_->get_sub_namespace() + ".scoreType", 1); //1 is FAST, 0 is HARRIS
    node_->declare_parameter(node_->get_sub_namespace() + ".patchSize", patchSize);
    node_->declare_parameter(node_->get_sub_namespace() + ".fastThreshold", fastThreshold);

    node_->declare_parameter(node_->get_sub_namespace() + ".debug", debug);

    node_->declare_parameter(node_->get_sub_namespace() + ".tolerance", tolerance); //Stereo tolerance for image matching
    node_->declare_parameter(node_->get_sub_namespace() + ".matchThreshold", matchThreshold); //Stereo threshold for image matching


    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_);

    // Function definition for the async parameters callback
    auto on_parameter_event_callback = [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
            {
                std::stringstream ss;
                ss << "[" + node_->get_sub_namespace() + "]: ";
                ss << "\nParameter event:\n new parameters:";
                for (auto & new_parameter : event->new_parameters) {
                    ss << "\n  " << new_parameter.name;
                }
                ss << "\n changed parameters:";
                for (auto & changed_parameter : event->changed_parameters) {
                    ss << "\n  " << changed_parameter.name;
                }
                ss << "\n deleted parameters:";
                for (auto & deleted_parameter : event->deleted_parameters) {
                    ss << "\n  " << deleted_parameter.name;
                }
                ss << "\n";
                RCLCPP_DEBUG(node_->get_logger(), ss.str().c_str());
            };

    // Setup callback for changes to parameters.
    parameter_event_sub_ = parameters_client_->on_parameter_event(on_parameter_event_callback);
}

std::shared_ptr<Frame> ORBExtractor::extract(const cv::Mat &img_left, const cv::Mat &img_right) {
    orb_->detect(img_left, left_key_pts_);
    orb_->compute(img_left, left_key_pts_, left_descriptors_);
    orb_->detect(img_right, right_key_pts_);
    orb_->compute(img_right, right_key_pts_, right_descriptors_);

    auto frame = std::make_shared<Frame>(left_key_pts_.size()); // This is the frame that will contain all match info for the two ORB extracted images
    find_relevant_features_(frame);
    frame->shrink_frame();
    return frame;
}

void ORBExtractor::find_relevant_features_(std::shared_ptr<Frame> frame){
    for(int i=0; i<left_key_pts_.size(); i++){
        for(int j=0; j<right_key_pts_.size(); j++){
            // If the point in on the same line +- 2 pixels and has an x that fits with the epipolar geometry of the stereo setup then count it good (ie x in left > x in right)
            // ### we will need to handle point that are at the same x later, those wont have a calculable depth since their is no disparity
            if(left_key_pts_.at(i).pt.x >= right_key_pts_.at(j).pt.x && left_key_pts_.at(i).pt.y >= right_key_pts_.at(j).pt.y - match_tolerance_ && left_key_pts_.at(i).pt.y <= right_key_pts_.at(j).pt.y + match_tolerance_){
                double distance = cv::norm(left_descriptors_.row(i), right_descriptors_.row(j),cv::NORM_HAMMING);
                if(distance < match_threshold_) {
                    frame->match_distances[i] = distance;
                    frame->match_features[i] = std::array<cv::KeyPoint, 2>{left_key_pts_[i], right_key_pts_[j]};
                    frame->match_descriptors[i] = std::array<cv::Mat, 2>{left_descriptors_.row(i), right_descriptors_.row(j)};
                }
            }
        }
    }
}

void ORBExtractor::compute_disparity_() {

}