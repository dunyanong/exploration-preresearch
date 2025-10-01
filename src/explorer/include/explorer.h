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
#include <ctime>
#include <cstdlib>

struct FrontierCell {
    int x, y;
    double world_x, world_y; 
    
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
    
    ros::Publisher goal_pub;
    ros::Publisher frontier_marker_pub;
    ros::Publisher cluster_marker_pub;
    
    int min_frontier_size;
    double max_exploration_distance;
    double frontier_search_radius;

    // Frontier exploration variables
    std::vector<FrontierCluster> frontier_clusters_;
    geometry_msgs::Point current_goal;
    bool has_current_goal;
    
    double stuck_timeout;
    ros::Time last_progress_time;
    std::vector<geometry_msgs::Point> inaccessible_frontiers;
    enum FallbackMode { NONE, RETREAT, RANDOM_WALK };
    FallbackMode fallback_mode;
    std::vector<geometry_msgs::Point> position_history;
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    
    // Frontier detection stuffs
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
    
    // Stuck detection and handling methods
    bool isStuck();
    void markFrontierInaccessible(const geometry_msgs::Point& frontier);
    void initiateFallbackBehavior();
    bool isFrontierAccessible(const geometry_msgs::Point& frontier);
    void retreatToHistory();
    void performRandomWalk();
    
    // Goal validation and setting
    bool setNewGoal(const geometry_msgs::Point& goal);
    bool isGoalReachable(const geometry_msgs::Point& goal);
    bool isPointInMap(const geometry_msgs::Point& point);
    double distanceBetween(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
    
    // Main exploration logic
    void exploreStep();
};