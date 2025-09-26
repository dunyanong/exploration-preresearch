#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

// english only 

struct Robot {
  std::string name;
  ros::Subscriber odom_sub; 
  geometry_msgs::Pose current_pose;
  ros::Subscriber path_sub;
  ros::Publisher cloud_pub;
  ros::Publisher local_path_pub;
  nav_msgs::Path a_star_path;
  ros::Publisher adjust_pub;
  int target_index;
  std::vector<geometry_msgs::Point> discrete_path;
};


class Explorer {
public:
    Explorer(ros::NodeHandle nh_);

private:
    ros::NodeHandle nh_;
    Robot r;

    // 订阅地图
    ros::Subscriber map_sub;
    nav_msgs::OccupancyGrid map, costmap;
    bool map_received;
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

};