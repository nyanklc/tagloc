#include <tagloc/tagloc.h>

namespace tagloc {
void TagLoc::initialize(tf2_ros::Buffer *tf2_buf, std::string map_frame_id) {
    tf2_buffer_ = tf2_buf;
    map_frame_id_ = map_frame_id;
}

void TagLoc::detectionsCallback(const tag_master::TagPose &msg) {
    // transform into map frame
    geometry_msgs::PoseStamped pose_in_map;
    uint8_t trials = 0;
    ros::Rate r(40);
    while (ros::ok()) {
        try {
            tf2::Transform header_to_pose;
            tf2::fromMsg(msg.pose.pose, header_to_pose);
            auto map_to_header_msg = tf2_buffer_->lookupTransform(msg.pose.header.frame_id, map_frame_id_, ros::Time(0));
            tf2::Transform map_to_header;
            tf2::fromMsg(map_to_header_msg.transform, map_to_header);
            tf2::Transform map_to_pose = map_to_header * header_to_pose;
            tf2::toMsg(map_to_pose, pose_in_map.pose);
            pose_in_map.header = msg.pose.header;
            pose_in_map.header.frame_id = map_frame_id_;
            break;
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            if (trials > 9)
                return;
            trials++;
            r.sleep();
        }
    }

    // put into map
    TagKey key(msg.id, msg.shape, msg.type);
    // if this is an object detection
    if (msg.shape == -1 && msg.type == -1) {
        std::lock_guard<std::mutex> lock(object_map_mutex_);
        // uncomment for constant updates on the table, we're just saving the first received pose for now
        /*
        if (object_detections_map_.count(key) > 0)
            object_detections_map_[key] = pose_in_map;
        else
            object_detections_map_.emplace(key, pose_in_map);
        */
        object_detections_map_.emplace(key, pose_in_map);
        // ROS_INFO("object detection after emplacing: %f %f %f", object_detections_map_[key].pose.position.x, object_detections_map_[key].pose.position.y, object_detections_map_[key].pose.position.z);
    }
    // if this is a tag detection
    else {
        std::lock_guard<std::mutex> lock(tag_map_mutex_);
        // uncomment for constant updates on the table, we're just saving the first received pose for now
        /*
        if (tag_detections_map_.count(key) > 0)
            tag_detections_map_[key] = pose_in_map;
        else
            tag_detections_map_.emplace(key, pose_in_map);
        */
        tag_detections_map_.emplace(key, pose_in_map);
        // ROS_INFO("tag detection after emplacing: %f %f %f", tag_detections_map_[key].pose.position.x, tag_detections_map_[key].pose.position.y, tag_detections_map_[key].pose.position.z);
    }
}

void TagLoc::publishRobotPose(ros::Publisher &map_pub, ros::Publisher &odom_pub) {
    // TODO:
}
}  // namespace tagloc