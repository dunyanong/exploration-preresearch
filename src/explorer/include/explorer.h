#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <queue>
#include <cmath>

// Frontier exploration structures
struct FrontierCell {
    int x, y;          // Grid coordinates
    double world_x, world_y;  // World coordinates
    
    FrontierCell(int x_, int y_, double wx, double wy) : x(x_), y(y_), world_x(wx), world_y(wy) {}
};

struct FrontierCluster {
    std::vector<FrontierCell> cells;
    geometry_msgs::Point centroid;
    double distance_to_robot;
    int size;
    
    FrontierCluster() : distance_to_robot(std::numeric_limits<double>::max()), size(0) {}
};

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

    // Subscribers
    ros::Subscriber map_sub;
    nav_msgs::OccupancyGrid map;
    bool map_received;
    
    // Publishers
    ros::Publisher goal_pub;
    ros::Publisher frontier_marker_pub;
    ros::Publisher cluster_marker_pub;
    
    // Parameters
    int min_frontier_size_;
    double max_exploration_distance_;
    double frontier_search_radius_;
    
    // Frontier exploration variables
    std::vector<FrontierCluster> frontier_clusters_;
    geometry_msgs::Point current_goal_;
    bool has_current_goal_;
    
    // Callback functions
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    
    // Frontier detection methods
    std::vector<FrontierCell> detectFrontiers(const nav_msgs::OccupancyGrid& grid);
    std::vector<FrontierCluster> clusterFrontiers(const std::vector<FrontierCell>& frontiers);
    FrontierCluster selectBestFrontier(const std::vector<FrontierCluster>& clusters);
    
    // Utility methods
    bool isFrontierCell(const nav_msgs::OccupancyGrid& grid, int x, int y);
    bool isFree(const nav_msgs::OccupancyGrid& grid, int x, int y);
    bool isUnknown(const nav_msgs::OccupancyGrid& grid, int x, int y);
    bool isOccupied(const nav_msgs::OccupancyGrid& grid, int x, int y);
    bool isValidCell(const nav_msgs::OccupancyGrid& grid, int x, int y);
    double distanceToRobot(const geometry_msgs::Point& point);
    void gridToWorld(const nav_msgs::OccupancyGrid& grid, int gx, int gy, double& wx, double& wy);
    
    // Visualization methods
    void publishFrontierMarkers(const std::vector<FrontierCell>& frontiers);
    void publishClusterMarkers(const std::vector<FrontierCluster>& clusters);
    
    // Goal publishing
    void publishGoal(const geometry_msgs::Point& goal);
    
    // Main exploration logic
    void exploreStep();
};