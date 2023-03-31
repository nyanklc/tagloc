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

#include "../src/etc.h"

namespace tagloc {
struct TagKey {
   public:
    TagKey(uint32_t id, int32_t shape, int32_t type) {
        id_ = id;
        shape_ = shape;
        type_ = type;
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
};

// hash for TagKey (we have unique ids, so we simply use the ids as the hash function)
struct TagKeyHasher {
    size_t operator()(const TagKey &key) const {
        return (size_t)key.id();
    }
};

class TagLoc {
   public:
    TagLoc() {}

    void initialize(tf2_ros::Buffer *tf2_buf, std::string map_frame_id);

    void detectionsCallback(const tag_master::TagPose &msg);

    void publishRobotPose(ros::Publisher &map_pub, ros::Publisher &odom_pub);

   private:
    tf2_ros::Buffer *tf2_buffer_;

    std::unordered_map<TagKey, geometry_msgs::PoseStamped, TagKeyHasher> tag_detections_map_;
    std::mutex tag_map_mutex_;
    std::unordered_map<TagKey, geometry_msgs::PoseStamped, TagKeyHasher> object_detections_map_;
    std::mutex object_map_mutex_;

    std::string map_frame_id_;
};
}  // namespace tagloc

#endif