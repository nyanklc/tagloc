#include <tagloc/tagloc.h>

namespace tagloc {

void TagLoc::initialize(tf2_ros::Buffer *tf2_buf, std::string map_frame_id) {
    tf2_buffer_ = tf2_buf;
    map_frame_id_ = map_frame_id;

    state_ = TaglocStates::SAVE_TAG_LOCATIONS;
}

void TagLoc::detectionsCallback(const tag_master::TagPose &msg) {
    state_mutex_.lock();
    switch (state_) {
        case TaglocStates::SAVE_TAG_LOCATIONS: {
            state_mutex_.unlock();

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
                } catch (tf2::TransformException &ex) {
                    ROS_WARN("%s", ex.what());
                    if (trials > 9)
                        return;
                    trials++;
                    r.sleep();
                }
            }

            TagKey key(msg.id, msg.shape, msg.type, msg.object_name);
            // if this is an object detection
            if (msg.shape == -1 && msg.type == -1) {
                std::lock_guard<std::mutex> lock(object_map_mutex_);

                if (object_detections_map_.count(key) > 0) {
                    ROS_INFO("updating object\n");
                    object_detections_map_[key] = pose_in_map;
                } else {
                    ROS_INFO("saving object\n");
                    object_detections_map_.emplace(key, pose_in_map);
                }
            }
            // if this is a tag detection
            else {
                std::lock_guard<std::mutex> lock(tag_map_mutex_);

                if (tag_detections_map_.count(key) > 0) {
                    tag_detections_map_[key] = pose_in_map;
                } else {
                    tag_detections_map_.emplace(key, pose_in_map);
                }
            }
        } break;  // case TaglocStates::SAVE_TAG_LOCATIONS

        case TaglocStates::PUBLISH_ROBOT_POSE: {
            state_mutex_.unlock();
            // TODO: update current detections
        } break;  // case TaglocStates::PUBLISH_ROBOT_POSE

        default: {
            state_mutex_.unlock();
        } break;  // case default
    }
}

void TagLoc::setState(int newState) {
    state_mutex_.lock();
    if (newState != state_) {
        state_ = newState;
    }
    state_mutex_.unlock();
}

void TagLoc::printSaved() {
    tag_map_mutex_.lock();
    ROS_INFO("tagloc printing tag map");
    for (auto const &tag : tag_detections_map_) {
        ROS_INFO("id %d \t position %.2f %.2f %.2f", tag.first.id(), tag.second.pose.position.x, tag.second.pose.position.y, tag.second.pose.position.z);
    }
    tag_map_mutex_.unlock();
    object_map_mutex_.lock();
    ROS_INFO("tagloc printing object map");
    for (auto const &tag : object_detections_map_) {
        ROS_INFO("name %s \t position %.2f %.2f %.2f", tag.first.name().c_str(), tag.second.pose.position.x, tag.second.pose.position.y, tag.second.pose.position.z);
    }
    object_map_mutex_.unlock();
}

}  // namespace tagloc