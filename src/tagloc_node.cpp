#include <tagloc/tagloc.h>
#include <tagloc/TaglocStateSrv.h>

using namespace tagloc;

TagLoc tagl;

bool saveTagLocationsService(TaglocStateSrv::Request &req, TaglocStateSrv::Request &res) {
    tagl.setState(TaglocStates::SAVE_TAG_LOCATIONS);
    return true;
}

bool publishRobotPoseService(TaglocStateSrv::Request &req, TaglocStateSrv::Request &res) {
    tagl.setState(TaglocStates::PUBLISH_ROBOT_POSE);
    return true;
}

bool printSavedService(TaglocStateSrv::Request &req, TaglocStateSrv::Request &res) {
    tagl.printSaved();
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tagloc_node");
    ros::NodeHandle nh;

    tf2_ros::Buffer tf2_buffer;
    tf2_ros::TransformListener tf2_listener(tf2_buffer);
    ros::Duration(5).sleep();

    tagl.initialize(&tf2_buffer, "camera_link");

    ros::Subscriber tag_pose_sub = nh.subscribe("tag_master_detections", 5, &TagLoc::detectionsCallback, &tagl);
    ros::Subscriber object_pose_sub = nh.subscribe("tag_master_object_detections", 5, &TagLoc::detectionsCallback, &tagl);

    ros::Publisher robot_pose_map_pub = nh.advertise<geometry_msgs::PoseStamped>("tagloc_robot_position_map", 10, true);
    ros::Publisher robot_pose_odom_pub = nh.advertise<geometry_msgs::PoseStamped>("tagloc_robot_position_odom", 10, true);

    ros::ServiceServer save_tag_locations_service = nh.advertiseService("tagloc_save_tag_locations", saveTagLocationsService);
    ros::ServiceServer publish_robot_pose_service = nh.advertiseService("tagloc_publish_robot_pose", publishRobotPoseService);
    ros::ServiceServer print_saved_service = nh.advertiseService("tagloc_print_saved", printSavedService);

    ros::Rate r(10);
    while (ros::ok()) {
        // TODO: publish robot pose
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}