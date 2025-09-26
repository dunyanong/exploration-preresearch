#include <ros/ros.h>
#include "explorer.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "explorer_node");
    ros::NodeHandle nh("~");
    Explorer explorer(nh);
    ros::spin();
    return 0;
}

