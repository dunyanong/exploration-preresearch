#include "explorer.h"

Explorer::Explorer(ros::NodeHandle nh_) : nh_(nh_), map_received(false), has_current_goal_(false), fallback_mode_(NONE) {
    // Initialize parameters
    nh_.param<int>("min_frontier_size", min_frontier_size_, 10);
    nh_.param<double>("max_exploration_distance", max_exploration_distance_, 10.0);
    nh_.param<double>("frontier_search_radius", frontier_search_radius_, 1.0);
    nh_.param<double>("stuck_timeout", stuck_timeout_, 20.0);  // 20 seconds timeout

    // Initialize stuck detection
    last_progress_time_ = ros::Time::now();
    
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
    
    geometry_msgs::Point current_position;
    current_position.x = msg->pose.pose.position.x;
    current_position.y = msg->pose.pose.position.y;
    
    // Track position history for retreat fallback
    position_history_.push_back(current_position);
    if (position_history_.size() > 100) {  // Keep last 100 positions
        position_history_.erase(position_history_.begin());
    }
    
    // Check for progress towards goal
    if (has_current_goal_ && distanceToRobot(current_goal_) > 1.0) {
        double dist_to_goal = distanceToRobot(current_goal_);
        // If we're making progress (getting closer to goal), update last_progress_time
        if (dist_to_goal < distanceToRobot(current_goal_) + 0.1) {  // Allow small movements
            last_progress_time_ = ros::Time::now();
        }
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
    // First, check if we're stuck and handle it
    if (isStuck()) {
        ROS_WARN("Robot appears to be stuck! Initiating recovery procedures.");
        markFrontierInaccessible(current_goal_);
        performSensorSweep();
        initiateFallbackBehavior();
        return;  // Don't proceed with normal exploration until unstuck
    }
    
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
    
    // Filter out inaccessible frontiers
    std::vector<FrontierCluster> accessible_clusters;
    for (const auto& cluster : clusters) {
        if (isFrontierAccessible(cluster.centroid)) {
            accessible_clusters.push_back(cluster);
        }
    }
    
    if (accessible_clusters.empty()) {
        ROS_WARN("No accessible frontiers found - initiating fallback behavior");
        initiateFallbackBehavior();
        return;
    }
    
    // Publish cluster visualization
    publishClusterMarkers(accessible_clusters);
    
    // Select best frontier
    FrontierCluster best_cluster = selectBestFrontier(accessible_clusters);
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
        last_progress_time_ = ros::Time::now();  // Reset progress timer
        publishGoal(current_goal_);
    }
}

bool Explorer::isStuck() {
    if (!has_current_goal_) {
        return false;  // Can't be stuck if no goal
    }
    
    ros::Duration time_since_progress = ros::Time::now() - last_progress_time_;
    return time_since_progress.toSec() > stuck_timeout_;
}

void Explorer::markFrontierInaccessible(const geometry_msgs::Point& frontier) {
    if (has_current_goal_) {
        inaccessible_frontiers_.push_back(current_goal_);
        ROS_INFO("Marked frontier (%.2f, %.2f) as inaccessible", current_goal_.x, current_goal_.y);
        has_current_goal_ = false;  // Clear current goal
    }
}

void Explorer::performSensorSweep() {
    // In simulation, we can trigger a map update by requesting more sensor data
    // For now, just log that we're performing a sensor sweep
    ROS_INFO("Performing 360° sensor sweep to update evidence grid");
    // In a real system, this would command the robot to rotate in place
    // For simulation, we assume the map gets updated through other means
}

void Explorer::initiateFallbackBehavior() {
    if (fallback_mode_ == NONE) {
        // Start with retreat to known location
        fallback_mode_ = RETREAT;
        ROS_INFO("Initiating fallback behavior: Retreat to known location");
        
        if (!position_history_.empty()) {
            // Retreat to a position from history (e.g., 20 steps back)
            size_t retreat_index = position_history_.size() > 20 ? position_history_.size() - 20 : 0;
            geometry_msgs::Point retreat_point = position_history_[retreat_index];
            
            current_goal_ = retreat_point;
            has_current_goal_ = true;
            last_progress_time_ = ros::Time::now();
            publishGoal(current_goal_);
        } else {
            // No history, try wall following
            fallback_mode_ = WALL_FOLLOW;
            ROS_INFO("No position history available, switching to wall following");
            // Wall following would require additional implementation
            // For now, just switch to random walk
            fallback_mode_ = RANDOM_WALK;
            ROS_INFO("Switching to random walk fallback");
        }
    } else if (fallback_mode_ == RETREAT) {
        // If retreat didn't work, try wall following
        fallback_mode_ = WALL_FOLLOW;
        ROS_INFO("Retreat failed, switching to wall following");
        // Wall following implementation would go here
        // For now, switch to random walk
        fallback_mode_ = RANDOM_WALK;
        ROS_INFO("Wall following not implemented, switching to random walk");
    } else if (fallback_mode_ == WALL_FOLLOW) {
        // If wall following didn't work, try random walk
        fallback_mode_ = RANDOM_WALK;
        ROS_INFO("Wall following failed, switching to random walk");
    } else {
        // Random walk is the last resort
        ROS_WARN("All fallback behaviors exhausted. Robot may be truly stuck.");
        fallback_mode_ = NONE;  // Reset for next attempt
    }
}

bool Explorer::isFrontierAccessible(const geometry_msgs::Point& frontier) {
    for (const auto& inaccessible : inaccessible_frontiers_) {
        double dist = sqrt(pow(frontier.x - inaccessible.x, 2) + pow(frontier.y - inaccessible.y, 2));
        if (dist < 2.0) {  // Consider frontiers within 2m as the same
            return false;
        }
    }
    return true;
}