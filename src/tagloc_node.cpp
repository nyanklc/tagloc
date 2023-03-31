#include <tagloc/tagloc.h>

using namespace tagloc;

int main(int argc, char **argv) {
    ros::init(argc, argv, "tagloc_node");
    ros::NodeHandle nh;

    ros::Rate r(20);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}