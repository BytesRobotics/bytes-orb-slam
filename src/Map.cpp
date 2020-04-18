//
// Created by michael on 4/13/20.
//

#include "Map.h"

Map::Map(std::shared_ptr<rclcpp::Node> node, int map_size, bool debug): node_(std::move(node)),transform_listener_(tf_buffer_),
tf_broadcaster_(node_){

    node_->declare_parameter(node_->get_sub_namespace() + ".debug", debug);
    debug_ = debug;
    node_->declare_parameter(node_->get_sub_namespace() + ".publishOdom", true);
    publish_odom_ = true;
    node_->declare_parameter(node_->get_sub_namespace() + ".broadcastTf", true);
    broadcast_tf_ = true;
    node_->declare_parameter(node_->get_sub_namespace() + ".odomFrameId", "map");
    odom_frame_id_ = "map";
    node_->declare_parameter(node_->get_sub_namespace() + ".childFrameId", "odom");
    odom_child_frame_id = "odom";

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
            if(changed_parameter.name == node_->get_sub_namespace() + ".debug") {
                debug_ = changed_parameter.value.bool_value;
            } else if(changed_parameter.name == node_->get_sub_namespace() + ".publishOdom") {
                publish_odom_ = changed_parameter.value.bool_value;
            } else if(changed_parameter.name == node_->get_sub_namespace() + ".broadcastTf") {
                broadcast_tf_ = changed_parameter.value.bool_value;
            } else if(changed_parameter.name == node_->get_sub_namespace() + ".odomFrameId") {
                odom_frame_id_ = changed_parameter.value.string_value;
            } else if(changed_parameter.name == node_->get_sub_namespace() + ".childFrameId") {
                odom_child_frame_id = changed_parameter.value.string_value;
            }
        }
        ss << "\n";
        RCLCPP_DEBUG(node_->get_logger(), ss.str().c_str());
    };

    // Setup callback for changes to parameters.
    parameter_event_sub_ = parameters_client_->on_parameter_event(on_parameter_event_callback);

    /// Point cloud for modeling the features on the map
    point_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("map_points", 1);

    /// Ready the map to recieve frames, since it is fixed size we shouldnt need to
    /// perform any more allocation beyond this point
    map_.reserve(map_size);
}

std::shared_ptr<Frame> Map::get_most_recent_keyframe() {
    if(!map_.empty()) {
        return map_[map_.size() - 1];
    }
    // return a null frame if the map does not have a keyframe yet
    std::shared_ptr<Frame> frame = std::make_shared<Frame>(Frame(0));
    return frame;
}

void Map::add_keyframe(const std::shared_ptr<Frame>& frame, image_geometry::StereoCameraModel &stereo_camera_model) {
    map_.push_back(frame);

    // Get time stamp from current ROS time
    auto stamp = std_msgs::msg::Header().stamp;
    stamp.nanosec = node_->now().nanoseconds();
    stamp.sec = node_->now().seconds();
    tf2::TimePoint time = tf2_ros::fromMsg(stamp);

    if(broadcast_tf_){publish_tf_odom_(stereo_camera_model, time);}
    if(publish_odom_){publish_odometry_(stereo_camera_model, time);}
    if(debug_){publish_pointcloud();}
}

void Map::publish_tf_odom_(image_geometry::StereoCameraModel &stereo_camera_model, tf2::TimePoint time) {
    if(!map_.empty()) {

        geometry_msgs::msg::TransformStamped base_to_camera_msg = tf_buffer_.lookupTransform(odom_child_frame_id,
                stereo_camera_model.left().tfFrame(), tf2::TimePointZero);
        tf2::Stamped<tf2::Transform> base_to_camera;
        tf2::fromMsg(base_to_camera_msg, base_to_camera);

        tf2::Stamped<tf2::Transform> transform_stamped(map_[map_.size()-1]->wheel_odom_to_camera*base_to_camera.inverse(),
                time, odom_frame_id_);
        geometry_msgs::msg::TransformStamped transform_stamped_msg = tf2::toMsg(transform_stamped);
        transform_stamped_msg.child_frame_id = odom_child_frame_id;
        tf_broadcaster_.sendTransform(tf2::toMsg(transform_stamped));
    }
}

void Map::publish_odometry_(image_geometry::StereoCameraModel &stereo_camera_model, tf2::TimePoint time) {
    if(!map_.empty()) {
        geometry_msgs::msg::Pose pose_msg;
        tf2::toMsg(map_[map_.size()-1]->wheel_odom_to_camera, pose_msg);

        if(last_pub_time_ == 0){
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.child_frame_id = stereo_camera_model.left().tfFrame();
            odom_msg.header.stamp = tf2_ros::toMsg(time);
            odom_msg.header.frame_id = odom_frame_id_;

            odom_msg.pose.pose = pose_msg;

            double dt = node_->now().seconds() + node_->now().nanoseconds()/1000000000.0 - last_pub_time_;
            odom_msg.pose.pose.position.x = (pose_msg.position.x - last_pose_msg_.position.x)/dt;
            odom_msg.pose.pose.position.y = (pose_msg.position.y - last_pose_msg_.position.y)/dt;
            odom_msg.pose.pose.position.z = (pose_msg.position.z - last_pose_msg_.position.z)/dt;

            // Need to implement quaternion orientation velocity (twist angular)

            odom_pub_->publish(odom_msg);
        }
        last_pub_time_ = node_->now().seconds() + node_->now().nanoseconds()/1000000000.0;
        last_pose_msg_ = pose_msg;
    }
}

void Map::publish_pointcloud() {
    sensor_msgs::msg::PointCloud2 point_cloud;
    point_cloud.header.frame_id = odom_frame_id_;
    point_cloud.header.stamp = node_->now();
    point_cloud.height = 1;
    point_cloud.is_bigendian = false;
    point_cloud.point_step = 12; // 4x3
    point_cloud.is_dense = true;

    // Create the point fields for doubles
    auto point_field = sensor_msgs::msg::PointField();
    point_field.name = "x";
    point_field.offset = 0;
    point_field.datatype = point_field.FLOAT32;
    point_field.count = 1;
    point_cloud.fields.push_back(point_field);
    point_field.name = "y";
    point_field.offset = 4;
    point_cloud.fields.push_back(point_field);
    point_field.name = "z";
    point_field.offset = 8;
    point_cloud.fields.push_back(point_field);
    for(const auto& frame : map_){
        point_cloud.width += frame->match_xyz.size();
        for(const auto& point : frame->match_xyz){
            // Each point is a float 32 (ie 4 x unit8) x 3
            for(int i=0;i<12;i++){
                // For the point referenced to odometry
                tf2::Vector3 point_o_tf(point.x, point.y, point.z);
                point_o_tf = frame->wheel_odom_to_camera*point_o_tf; // Transform into odom reference
                if(i/4<1){
                    auto x = static_cast<float>(point_o_tf.x());
                    char* byteArray = reinterpret_cast<char*>(&x);
                    point_cloud.data.push_back(byteArray[i%4]);
                } else if(i/4<2){
                    auto y = static_cast<float>(point_o_tf.y());
                    char* byteArray = reinterpret_cast<char*>(&y);
                    point_cloud.data.push_back(byteArray[i%4]);
                } else {
                    auto z = static_cast<float>(point_o_tf.z());
                    char* byteArray = reinterpret_cast<char*>(&z);
                    point_cloud.data.push_back(byteArray[i%4]);
                }
            }
        }
    }
    point_cloud.row_step = point_cloud.data.size();
    point_cloud_pub_->publish(point_cloud);
}
