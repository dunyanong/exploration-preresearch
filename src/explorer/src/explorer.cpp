#include "explorer.h"

Explorer::Explorer(ros::NodeHandle nh_) : nh_(nh_) {
    r.odom_sub = nh_.subscribe("/agv1/odom", 10, &Explorer::odomCallback, this);

    map_sub = nh_.subscribe("/agv_map", 1, &Explorer::mapCallback, this);

}

// printout current pose
void Explorer::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    if (!msg) {
        return;
    }
    // ROS_INFO_STREAM("Current Pose: " << msg->pose.pose);

}

void Explorer::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    if (!msg) {
        return;
    }
    // ROS_INFO_STREAM("Received Map: " << msg->header.stamp);
}