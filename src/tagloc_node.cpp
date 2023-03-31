#include <tagloc/tagloc.h>

using namespace tagloc;

int main(int argc, char **argv) {
    ros::init(argc, argv, "tagloc_node");
    ros::NodeHandle nh;

    tf2_ros::Buffer tf2_buffer;
    tf2_ros::TransformListener tf2_listener(tf2_buffer);
    ros::Duration(5).sleep();

    TagLoc tagloc;
    tagloc.initialize(&tf2_buffer, "camera_link");

    ros::Subscriber tag_pose_sub = nh.subscribe("tag_master_detections", 5, &TagLoc::detectionsCallback, &tagloc);
    ros::Subscriber object_pose_sub = nh.subscribe("tag_master_object_detections", 5, &TagLoc::detectionsCallback, &tagloc);

    ros::Publisher robot_pose_map_pub = nh.advertise<geometry_msgs::PoseStamped>("tagloc_robot_position_map", 10, true);
    ros::Publisher robot_pose_odom_pub = nh.advertise<geometry_msgs::PoseStamped>("tagloc_robot_position_odom", 10, true);

    ros::Rate r(10);
    while (ros::ok()) {
        tagloc.publishRobotPose(robot_pose_map_pub, robot_pose_odom_pub);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}