//
// Created by michael on 4/4/20.
//

#include "System.h"

System::System(rclcpp::Node *node, const std::string& strVocFile): orb_extractor_(node->create_sub_node("orb")) {
    std::stringstream cv_version;
    cv_version << "OpenCV Version: " << CV_VERSION;
    RCLCPP_INFO(node->get_logger(), cv_version.str().c_str());
}

void System::Track(const cv::Mat &left_img, const cv::Mat &right_image, image_geometry::StereoCameraModel& stereo_camera_model, const double &timestamp) {
    //orb_extractor(imLeft, mvKeysLeft, mDescriptorsLeft);
    //const int PATCH_SIZE = 31;
    //const int HALF_PATCH_SIZE = 15;
    //const int EDGE_THRESHOLD = 19l

    std::shared_ptr<Frame> new_frame = orb_extractor_.extract(left_img, right_image, stereo_camera_model);
}