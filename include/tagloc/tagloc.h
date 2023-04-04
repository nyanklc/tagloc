#ifndef __TAGLOC_H_
#define __TAGLOC_H_

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tag_master/TagPose.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace tagloc {
struct TagKey {
   public:
    TagKey(uint32_t id, int32_t shape, int32_t type, std::string obj_name) {
        id_ = id;
        shape_ = shape;
        type_ = type;
        obj_name_ = obj_name;
    }

    uint32_t id() const {
        return id_;
    }

    int32_t shape() const {
        return shape_;
    }

    int32_t type() const {
        return type_;
    }

    std::string name() const {
        return obj_name_;
    }

    bool operator==(const TagKey &other) const {
        if (other.id_ == id_ && other.shape_ == shape_ && other.type_ == type_)
            return true;
        return false;
    }

    void operator=(const TagKey &other) {
        id_ = other.id_;
        shape_ = other.shape_;
        type_ = other.type_;
    }

   private:
    uint32_t id_;
    int32_t shape_;
    int32_t type_;
    std::string obj_name_;
};

// hash for TagKey (we have unique ids, so we simply use the ids as the hash function)
struct TagKeyHasher {
    size_t operator()(const TagKey &key) const {
        return (size_t)key.id();
    }
};

enum TaglocStates {
    SAVE_TAG_LOCATIONS,
    PUBLISH_ROBOT_POSE,
};

class TagLoc {
   public:
    TagLoc() {}

    void initialize(tf2_ros::Buffer *tf2_buf, std::string map_frame_id);

    void detectionsCallback(const tag_master::TagPose &msg);

    void setState(int newState);

    void printSaved();

   private:
    tf2_ros::Buffer *tf2_buffer_;

    std::unordered_map<TagKey, geometry_msgs::PoseStamped, TagKeyHasher> tag_detections_map_;
    std::mutex tag_map_mutex_;
    std::unordered_map<TagKey, geometry_msgs::PoseStamped, TagKeyHasher> object_detections_map_;
    std::mutex object_map_mutex_;

    std::string map_frame_id_;

    std::mutex state_mutex_;
    int state_;
};
}  // namespace tagloc

#endif