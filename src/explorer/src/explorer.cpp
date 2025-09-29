#include "explorer.h"

Explorer::Explorer(ros::NodeHandle nh_) : nh_(nh_), map_received(false), has_current_goal_(false) {
    // Initialize parameters
    nh_.param<int>("min_frontier_size", min_frontier_size_, 10);
    nh_.param<double>("max_exploration_distance", max_exploration_distance_, 10.0);
    nh_.param<double>("frontier_search_radius", frontier_search_radius_, 1.0);
    
    // Subscribers
    r.odom_sub = nh_.subscribe("/agv1/odom", 10, &Explorer::odomCallback, this);
    map_sub = nh_.subscribe("/agv_map", 1, &Explorer::mapCallback, this);
    
    // Publishers
    goal_pub = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    frontier_marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("/frontier_markers", 1);
    cluster_marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("/cluster_markers", 1);
    
    ROS_INFO("Frontier Explorer initialized");
    ROS_INFO("Parameters - min_frontier_size: %d, max_exploration_distance: %.2f", 
             min_frontier_size_, max_exploration_distance_);
}

void Explorer::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    if (!msg) {
        return;
    }
    
    r.current_pose = msg->pose.pose;
    
    // If we have a map, perform exploration step
    if (map_received) {
        exploreStep();
    }
}

void Explorer::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    if (!msg) {
        return;
    }
    
    map = *msg;
    map_received = true;
    ROS_INFO_THROTTLE(5.0, "Map received: %dx%d, resolution: %.3f", 
                      map.info.width, map.info.height, map.info.resolution);
}

std::vector<FrontierCell> Explorer::detectFrontiers(const nav_msgs::OccupancyGrid& grid) {
    std::vector<FrontierCell> frontiers;
    
    for (int y = 1; y < (int)grid.info.height - 1; y++) {
        for (int x = 1; x < (int)grid.info.width - 1; x++) {
            if (isFrontierCell(grid, x, y)) {
                double world_x, world_y;
                gridToWorld(grid, x, y, world_x, world_y);
                frontiers.emplace_back(x, y, world_x, world_y);
            }
        }
    }
    
    ROS_INFO_THROTTLE(5.0, "Detected %zu frontier cells", frontiers.size());
    return frontiers;
}

bool Explorer::isFrontierCell(const nav_msgs::OccupancyGrid& grid, int x, int y) {
    // A frontier cell must be free space
    if (!isFree(grid, x, y)) {
        return false;
    }
    
    // Check 8-connected neighbors for unknown space
    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            if (dx == 0 && dy == 0) continue;
            
            int nx = x + dx;
            int ny = y + dy;
            
            if (isValidCell(grid, nx, ny) && isUnknown(grid, nx, ny)) {
                return true;
            }
        }
    }
    
    return false;
}

std::vector<FrontierCluster> Explorer::clusterFrontiers(const std::vector<FrontierCell>& frontiers) {
    std::vector<FrontierCluster> clusters;
    std::vector<bool> visited(frontiers.size(), false);
    
    for (size_t i = 0; i < frontiers.size(); i++) {
        if (visited[i]) continue;
        
        FrontierCluster cluster;
        std::queue<size_t> queue;
        queue.push(i);
        visited[i] = true;
        
        double sum_x = 0, sum_y = 0;
        
        while (!queue.empty()) {
            size_t idx = queue.front();
            queue.pop();
            
            const FrontierCell& cell = frontiers[idx];
            cluster.cells.push_back(cell);
            sum_x += cell.world_x;
            sum_y += cell.world_y;
            
            // Find neighboring frontier cells
            for (size_t j = 0; j < frontiers.size(); j++) {
                if (visited[j]) continue;
                
                const FrontierCell& neighbor = frontiers[j];
                double dist = sqrt(pow(cell.world_x - neighbor.world_x, 2) + 
                                 pow(cell.world_y - neighbor.world_y, 2));
                
                if (dist <= frontier_search_radius_) {
                    visited[j] = true;
                    queue.push(j);
                }
            }
        }
        
        // Only keep clusters above minimum size
        if (cluster.cells.size() >= (size_t)min_frontier_size_) {
            cluster.size = cluster.cells.size();
            cluster.centroid.x = sum_x / cluster.size;
            cluster.centroid.y = sum_y / cluster.size;
            cluster.centroid.z = 0.0;
            
            cluster.distance_to_robot = distanceToRobot(cluster.centroid);
            clusters.push_back(cluster);
        }
    }
    
    ROS_INFO_THROTTLE(5.0, "Found %zu frontier clusters", clusters.size());
    return clusters;
}

FrontierCluster Explorer::selectBestFrontier(const std::vector<FrontierCluster>& clusters) {
    FrontierCluster best_cluster;
    double min_distance = std::numeric_limits<double>::max();
    
    for (const auto& cluster : clusters) {
        // Skip clusters that are too far away
        if (cluster.distance_to_robot > max_exploration_distance_) {
            continue;
        }
        
        // Select nearest cluster (could be extended with more sophisticated selection)
        if (cluster.distance_to_robot < min_distance) {
            min_distance = cluster.distance_to_robot;
            best_cluster = cluster;
        }
    }
    
    return best_cluster;
}

bool Explorer::isFree(const nav_msgs::OccupancyGrid& grid, int x, int y) {
    if (!isValidCell(grid, x, y)) return false;
    int idx = y * grid.info.width + x;
    return grid.data[idx] >= 0 && grid.data[idx] < 50;  // Free space threshold
}

bool Explorer::isUnknown(const nav_msgs::OccupancyGrid& grid, int x, int y) {
    if (!isValidCell(grid, x, y)) return false;
    int idx = y * grid.info.width + x;
    return grid.data[idx] == -1;  // Unknown space
}

bool Explorer::isOccupied(const nav_msgs::OccupancyGrid& grid, int x, int y) {
    if (!isValidCell(grid, x, y)) return false;
    int idx = y * grid.info.width + x;
    return grid.data[idx] >= 50;  // Occupied space threshold
}

bool Explorer::isValidCell(const nav_msgs::OccupancyGrid& grid, int x, int y) {
    return x >= 0 && x < (int)grid.info.width && y >= 0 && y < (int)grid.info.height;
}

double Explorer::distanceToRobot(const geometry_msgs::Point& point) {
    return sqrt(pow(point.x - r.current_pose.position.x, 2) + 
                pow(point.y - r.current_pose.position.y, 2));
}

void Explorer::gridToWorld(const nav_msgs::OccupancyGrid& grid, int gx, int gy, double& wx, double& wy) {
    wx = grid.info.origin.position.x + (gx + 0.5) * grid.info.resolution;
    wy = grid.info.origin.position.y + (gy + 0.5) * grid.info.resolution;
}

void Explorer::publishFrontierMarkers(const std::vector<FrontierCell>& frontiers) {
    visualization_msgs::MarkerArray marker_array;
    
    // Clear previous markers
    visualization_msgs::Marker clear_marker;
    clear_marker.header.frame_id = "map";
    clear_marker.header.stamp = ros::Time::now();
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
    
    // Add frontier points
    for (size_t i = 0; i < frontiers.size(); i++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "frontiers";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.pose.position.x = frontiers[i].world_x;
        marker.pose.position.y = frontiers[i].world_y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        
        marker_array.markers.push_back(marker);
    }
    
    frontier_marker_pub.publish(marker_array);
}

void Explorer::publishClusterMarkers(const std::vector<FrontierCluster>& clusters) {
    visualization_msgs::MarkerArray marker_array;
    
    for (size_t i = 0; i < clusters.size(); i++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "clusters";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.pose.position = clusters[i].centroid;
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.8;
        
        marker_array.markers.push_back(marker);
    }
    
    cluster_marker_pub.publish(marker_array);
}

void Explorer::publishGoal(const geometry_msgs::Point& goal) {
    geometry_msgs::PoseStamped goal_msg;
    goal_msg.header.frame_id = "map";
    goal_msg.header.stamp = ros::Time::now();
    
    goal_msg.pose.position = goal;
    goal_msg.pose.orientation.w = 1.0;  // No specific orientation required
    
    goal_pub.publish(goal_msg);
    ROS_INFO("Published exploration goal: (%.2f, %.2f)", goal.x, goal.y);
}

void Explorer::exploreStep() {
    // Detect frontiers
    std::vector<FrontierCell> frontiers = detectFrontiers(map);
    if (frontiers.empty()) {
        ROS_INFO_THROTTLE(10.0, "No frontiers found - exploration complete!");
        return;
    }
    
    // Publish frontier visualization
    publishFrontierMarkers(frontiers);
    
    // Cluster frontiers
    std::vector<FrontierCluster> clusters = clusterFrontiers(frontiers);
    if (clusters.empty()) {
        ROS_INFO_THROTTLE(10.0, "No valid frontier clusters found");
        return;
    }
    
    // Publish cluster visualization
    publishClusterMarkers(clusters);
    
    // Select best frontier
    FrontierCluster best_cluster = selectBestFrontier(clusters);
    if (best_cluster.size == 0) {
        ROS_INFO_THROTTLE(10.0, "No accessible frontiers within range");
        return;
    }
    
    // Check if we need to set a new goal
    if (!has_current_goal_ || 
        distanceToRobot(current_goal_) < 1.0 ||  // Close to current goal
        distanceToRobot(best_cluster.centroid) < distanceToRobot(current_goal_) * 0.8) {  // Better goal found
        
        current_goal_ = best_cluster.centroid;
        has_current_goal_ = true;
        publishGoal(current_goal_);
    }
}