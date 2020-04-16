//
// Created by michael on 4/10/20.
//

#include "Tracking.h"

Tracker::Tracker(std::shared_ptr<rclcpp::Node> node, int frame_rigidity, bool debug, int min_coord_dist, int min_feature_dist, int tracking_period):
                node_(std::move(node)),  rng_(cv::getCPUTickCount()), last_frame_(nullptr) {

    node_->declare_parameter(node_->get_sub_namespace() + ".frameRigidity", frame_rigidity);
    frame_rigidity_ = frame_rigidity;
    node_->declare_parameter(node_->get_sub_namespace() + ".debug", debug);
    debug_ = debug;
    node_->declare_parameter(node_->get_sub_namespace() + ".minCoordDist", min_coord_dist);
    min_coord_dist_ = min_coord_dist;
    node_->declare_parameter(node_->get_sub_namespace() + ".minFeatureDist", min_feature_dist);
    min_feature_dist_ = min_feature_dist;
    node_->declare_parameter(node_->get_sub_namespace() + ".trackingPeriod", tracking_period);
    tracking_period_ = tracking_period;

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
            if(changed_parameter.name == node_->get_sub_namespace() + ".frameRigidity"){
                frame_rigidity_ = changed_parameter.value.integer_value;
            } else if(changed_parameter.name == node_->get_sub_namespace() + ".debug") {
                debug_ = changed_parameter.value.bool_value;
            } else if(changed_parameter.name == node_->get_sub_namespace() + ".minCoordDist") {
                min_coord_dist_ = changed_parameter.value.integer_value;
            } else if(changed_parameter.name == node_->get_sub_namespace() + ".minFeatureDist") {
                min_feature_dist_ = changed_parameter.value.integer_value;
            } else if(changed_parameter.name == node_->get_sub_namespace() + ".trackingPeriod") {
                tracking_period_ = changed_parameter.value.integer_value;
            }
        }
        ss << "\n";
        RCLCPP_DEBUG(node_->get_logger(), ss.str().c_str());
    };

    // Setup callback for changes to parameters.
    parameter_event_sub_ = parameters_client_->on_parameter_event(on_parameter_event_callback);

}

void Tracker::track_with_new_frame(const std::shared_ptr<Frame>& new_frame, image_geometry::StereoCameraModel &stereo_camera_model, const cv::Mat &new_img) {
        /** STEP 1: First we need to find feature correspondences and compute an error vector that
         * can be used to update the short term to have a better estimate of current motion.
         * The first function takes in the new frame and comapres it to the last to figure out which point are the same.
         * In step 2 that information is used to create an optical flow like prediction about the motion of the robot.
         * That prediction is used to update the new frame's position before.
         */
        // These are all the points from that last frame that is transformed into the new frame
        if (last_frame_ != nullptr) {

            if (debug_) {match_start_ = std::chrono::steady_clock::now();iterations_++;transformed_point_mapping_.clear();}
            std::vector<cv::Point3d> transformed_points;
            // Reserve the size of the last frame since that frame is the one used as reference for matching
            transformed_points.reserve(last_frame_->match_xyz.size());

            // Get ready for a new round of frame to frame matching for short baseline visual odometry
            matches_.clear();
            transformed_point_mapping_.clear();
            matches_.reserve(last_frame_->match_xyz.size());
            transformed_point_mapping_.reserve(last_frame_->match_xyz.size());

            // Updated matches and transformed point mappings to reflect them
            find_matching_points(new_frame, stereo_camera_model, transformed_points);
            // the initial error is the L1 distance error between the transformed points from the old frame and their matches in the new frame
            // This error reflects the error in the input odometry relative to the short-baseline visual odometry

            if (debug_) { match_stop_ = std::chrono::steady_clock::now(); }

            /**
             * STEP 2: Now we are going to take the feature correspondences and compute the average error vector in 3D space.
             * An uncertainty in the prediction will be based on the strength of the consensus and the number of points.
             * This part of the algorithm requires a well developed set of matching features and the transform
             * of the features that match in the new frame from the last frame. The result will be stored in two vectors
             * the first will contain the mean error vector and the second will contain the variance deviations for each
             * axis.
             */

            // max variable value should be where 95.4% of data lies (2 standard deviations) - OR MAYBE NOT
            // The transform used by the new frame is used to set the initial mean and is continually updated to reduce the
            // error in the system

            if (debug_) {flow_optimization_start_ = std::chrono::steady_clock::now();}
            tf2::Transform best_guess = new_frame->wheel_odom_to_camera.inverse() * last_frame_->wheel_odom_to_camera;
            std::vector<cv::Point3d> masked_transformed_points;
            new_frame->transform_keypoints(last_frame_, best_guess, transformed_point_mapping_,masked_transformed_points);
            double min_error = compute_error(masked_transformed_points);
            double starting_error = min_error; // For debugging
            for(int iteration=0;iteration<10;iteration++){
                tf2::Transform mean = best_guess;
                for(int i=0;i<10;i++){
                    tf2::Transform new_guess = new_frame->wheel_odom_to_camera.inverse() * last_frame_->wheel_odom_to_camera;
                    int scalar = (iteration+1)*(iteration+1);
                    new_guess.getOrigin().setX(mean.getOrigin().getX()+rng_.gaussian(0.1/scalar));
                    new_guess.getOrigin().setY(mean.getOrigin().getY()+rng_.gaussian(0.1/scalar));
                    new_guess.getOrigin().setZ(mean.getOrigin().getZ()+rng_.gaussian(0.1/scalar));
                    new_guess.getRotation().setX(mean.getRotation().getX()+rng_.gaussian(0.1/scalar));
                    new_guess.getRotation().setY(mean.getRotation().getY()+rng_.gaussian(0.1/scalar));
                    new_guess.getRotation().setZ(mean.getRotation().getZ()+rng_.gaussian(0.1/scalar));
                    new_guess.getRotation().setW(sqrt(1-new_guess.getRotation().getX()*new_guess.getRotation().getX()-
                                                         new_guess.getRotation().getY()*new_guess.getRotation().getY()-
                                                         new_guess.getRotation().getZ()*new_guess.getRotation().getZ()));
                    masked_transformed_points.clear();
                    new_frame->transform_keypoints(last_frame_, new_guess, transformed_point_mapping_,masked_transformed_points);
                    double error = compute_error(masked_transformed_points);
                    if(error < min_error){
                        best_guess = new_guess;
                        min_error = error;
                    }
                }
            }
            double stopping_error = min_error;
            if (debug_) {flow_optimization_stop_ = std::chrono::steady_clock::now();}

            // Debugging the tracker
            if (debug_) {
                // Draw the debug images
                show_optical_flow(new_img, transformed_points, stereo_camera_model);
                double match_time = std::chrono::duration_cast<std::chrono::microseconds>(match_stop_ - match_start_).count() / 1000.0;
                double flow_optimization_time =  std::chrono::duration_cast<std::chrono::microseconds>(flow_optimization_stop_ - flow_optimization_start_).count() / 1000.0;
                double total_time = match_time + flow_optimization_time;
                aggregate_total_time_ += total_time;

                //Print out timing for the ORB round
                std::stringstream ss;
                ss << "Tracker Diagnostics: iteration #" << iterations_ << std::endl;
                ss << "Time to complete interframe feature matching: " << match_time << " ms" << std::endl;
                ss << "Time to complete optical flow (interframe transform) optimization: " << flow_optimization_time << " ms" << std::endl;
                ss << "Total time: " << total_time << " ms" << std::endl;
                ss << "Average total time: " << aggregate_total_time_ / iterations_ << " ms" << std::endl;
                ss << "Tracker match Rate: " << matches_.size() / static_cast<double>(transformed_points.size()) << std::endl;
                ss << "Odom error reduction : " << starting_error - stopping_error << std::endl;
                RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
            }
        }
    last_frame_ = new_frame;
}

void Tracker::show_optical_flow(const cv::Mat &new_img, std::vector<cv::Point3d>& transformed_points, image_geometry::StereoCameraModel &stereo_camera_model) {
    cv::Mat flow_img;
    cv::cvtColor(new_img, flow_img, cv::COLOR_GRAY2BGR);
    for(unsigned int i=0;i<matches_.size(); i++){
        auto old_point = stereo_camera_model.left().project3dToPixel(matches_[i][0]);
        auto new_point = stereo_camera_model.right().project3dToPixel(matches_[i][1]);
        if(!isnan(old_point.x*old_point.y*new_point.x*new_point.y) && !isinf(old_point.x*old_point.y*new_point.x*new_point.y)){
            cv::arrowedLine(flow_img, old_point, new_point, cv::Scalar(0, 255, 0), 1); // This is the motion of the image
            cv::arrowedLine(flow_img, stereo_camera_model.left().project3dToPixel(transformed_points[transformed_point_mapping_[i]]),
                    new_point, cv::Scalar(0, 0, 255), 1); // This is the error in the odometry wrt the visual odom
            cv::circle(flow_img, old_point, 1, cv::Scalar(255, 0, 0)); // This denotes the old point
        }
    }
    cv::resize(flow_img, flow_img, cv::Size(), 4, 4, cv::INTER_CUBIC);
    cv::imshow("Optical Flow", flow_img);
    cv::waitKey(tracking_period_);
}

void Tracker::find_matching_points(const std::shared_ptr<Frame> &new_frame, image_geometry::StereoCameraModel &stereo_camera_model,
                                   std::vector<cv::Point3d>& transformed_points) {

    // Found the points transformed into the new frame
    new_frame->transform_keypoints(last_frame_, transformed_points);

    // Find the matches based on description and proximity to reprojected point
    // Brute force search
    for (unsigned i = 0; i < transformed_points.size(); i++) {
        cv::Point2d reprojected_point = stereo_camera_model.left().project3dToPixel(transformed_points[i]);
        double min_feature_distance = min_feature_dist_; //this stores the feature distance from the point with the nearest distance
        int match_i = -1, match_j = -1;
        for (unsigned j = 0; j < new_frame->match_descriptors.size(); j++) {
            // Get the squared taxi-cab (l1) distance between the reprojected point and each point in the new frame
            int coord_dist = static_cast<int>(abs(new_frame->match_features[j].data()->pt.x - reprojected_point.x) +
                                              abs(new_frame->match_features[j].data()->pt.y - reprojected_point.y));
            if (coord_dist < min_coord_dist_) {
                // The point must be close to the reprojected point
                // Get the average of the distances to make up final distance
                double feature_dist = cv::norm(new_frame->match_descriptors[j][0],
                        last_frame_->match_descriptors[i][0], cv::NORM_HAMMING);
                if (feature_dist < min_feature_distance) {
                    min_feature_distance = feature_dist;
                    match_i = static_cast<int>(i); // last frame
                    match_j = static_cast<int>(j); // new frame
                }
            }
        }
        if (match_i != -1) {
            // {last frame, new frame}
            matches_.push_back({last_frame_->match_xyz[match_i], new_frame->match_xyz[match_j]});
            transformed_point_mapping_.push_back(match_i);
       }
    }
    matches_.shrink_to_fit();
}

double Tracker::compute_error(std::vector<cv::Point3d> &masked_transformed_points) {
    double error = 0;
    for (unsigned int i=0; i<matches_.size(); i++) {
        // Get the squared taxi-cab (l1) distance between the reprojected point and each point in the new frame
        error += abs(matches_[i][1].x - masked_transformed_points[i].x) +
                 abs(matches_[i][1].y - masked_transformed_points[i].y) +
                 abs(matches_[i][1].z - masked_transformed_points[i].z);
    }
    return error;
}
