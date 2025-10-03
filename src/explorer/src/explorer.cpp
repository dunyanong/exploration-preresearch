#include "explorer.h"

Explorer::Explorer(ros::NodeHandle nh_) : nh_(nh_), map_received(false), has_current_goal(false), fallback_mode(NONE) {
    srand(time(NULL));
    
    // Create some parameters
    nh_.param("min_frontier_size", min_frontier_size, 2);
    nh_.param("max_exploration_distance", max_exploration_distance, 3.0);
    nh_.param("frontier_search_radius", frontier_search_radius, 0.8);
    nh_.param("stuck_timeout", stuck_timeout, 20.0);

    last_progress_time = ros::Time::now();

    r.odom_sub = nh_.subscribe("/agv1/odom", 10, &Explorer::odomCallback, this);
    map_sub = nh_.subscribe("/agv_map", 1, &Explorer::mapCallback, this);

    goal_pub = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

}

// printout current pose
void Explorer::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    if (!msg) {
        return;
    }
    
    geometry_msgs::Point current_position;
    current_position.x = msg->pose.pose.position.x;
    current_position.y = msg->pose.pose.position.y;

    position_history.push_back(current_position);
    if (position_history.size() > 100) {
        position_history.erase(position_history.begin());
    }

    if (has_current_goal) {
        last_progress_time = ros::Time::now();
    }
    r.current_pose = msg->pose.pose;

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
    // ROS_INFO_STREAM("Received Map: " << msg->header.stamp);
}

double Explorer::distanceToRobot(const geometry_msgs::Point& point) {
    return sqrt(
        pow(point.x - r.current_pose.position.x, 2) +
        pow(point.y - r.current_pose.position.y, 2)
    );
}

void Explorer::exploreStep() {
    if (isStuck()) {
        ROS_WARN("Robot is stuck! Initiating fallback behavior.");
        markFrontierInaccessible(current_goal);
        initiateFallbackBehavior();
        return;
    }

    // Detect Frontier
    std::vector<FrontierCell> frontiers = detectFrontiers(map);

    if (frontiers.empty()) {
        ROS_INFO_THROTTLE(10.0, "No frontiers found - exploration complete!");
        return;
    }

    std::vector<FrontierCluster> clusters = clusterFrontiers(frontiers);
    if (clusters.empty()) {
        ROS_INFO_THROTTLE(10.0, "No valid frontier clusters found.");
        return;
    }

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
    
    FrontierCluster best_cluster = selectBestFrontier(accessible_clusters);

    // Check if we need to set a new goal
    if (!has_current_goal || 
        distanceToRobot(current_goal) < 1.0 ||
        distanceToRobot(best_cluster.centroid) < distanceToRobot(current_goal) * 0.8) {
        setNewGoal(best_cluster.centroid);
    }

}

bool Explorer::isStuck() {
    if (!has_current_goal) {
        return false;
    }
    ros::Duration time_since_last_progress = ros::Time::now() - last_progress_time;
    return time_since_last_progress.toSec() > stuck_timeout;
}

// It's triggered when the robot gets stuck: it marks the current goal as inaccessible and initiates fallback behavior.
void Explorer::markFrontierInaccessible(const geometry_msgs::Point& frontier) {
    if (has_current_goal) {
        inaccessible_frontiers.push_back(frontier);
        ROS_INFO("Marked frontier (%.2f, %.2f) as inaccessible", frontier.x, frontier.y);
        has_current_goal = false;
    }
}

void Explorer::initiateFallbackBehavior() {
    switch (fallback_mode) {
        case NONE:
            if (!position_history.empty()) {
                fallback_mode = RETREAT;
                retreatToHistory();
            } else {
                fallback_mode = RANDOM_WALK;
                performRandomWalk();
            }
            break;
        case RETREAT:
            // If we're still in RETREAT mode, it means retreat failed
            fallback_mode = RANDOM_WALK;

            performRandomWalk();
            ROS_INFO("Retreat failed, switching to random walk");
            break;
        case RANDOM_WALK:
            performRandomWalk();
            ROS_WARN("Random walk fallback active");
            break;
    }

}

void Explorer::retreatToHistory() {
    if (!position_history.empty()) {
        geometry_msgs::Point retreat_point = position_history.back();
        if (setNewGoal(retreat_point)) {
            ROS_INFO("Retreating to historical position");
        } else {
            ROS_WARN("Retreat position is unreachable");
        }
    }
}

void Explorer::performRandomWalk() {
    // Try directions away from inaccessible frontiers first
    std::vector<double> preferred_angles = {0, M_PI/2, M_PI, 3*M_PI/2}; // N, E, S, W
    
    for (int attempt = 0; attempt < 10; attempt++) {
        double angle, dist;
        
        if (attempt < 4) {
            // Try preferred directions first
            angle = preferred_angles[attempt];
            dist = 2.0; 
        } else {
            angle = (rand() / (RAND_MAX + 1.0)) * 2.0 * M_PI;
            dist = 1.0 + (rand() / (RAND_MAX + 1.0)) * 2.0;
        
        geometry_msgs::Point random_goal;
        random_goal.x = r.current_pose.position.x + dist * cos(angle);
        random_goal.y = r.current_pose.position.y + dist * sin(angle);
        random_goal.z = 0;
        
        if (setNewGoal(random_goal)) {
            ROS_INFO("Random walk goal set (attempt %d)", attempt + 1);
            return;
        }
    }
    ROS_WARN("Could not find valid random walk goal after 10 attempts");
}
}

bool Explorer::setNewGoal(const geometry_msgs::Point& goal) {
    // Check if goal is reachable before setting
    if (isGoalReachable(goal)) {
        current_goal = goal;
        has_current_goal = true;
        last_progress_time = ros::Time::now();
        publishGoal(goal);
        return true;
    }
    return false;
}

bool Explorer::isGoalReachable(const geometry_msgs::Point& goal) {
    // 1. Check if goal is within map bounds
    if (!isPointInMap(goal)) {
        return false;
    }
    
    // 2. Check if goal is too close to inaccessible frontiers
    for (const auto& inaccessible : inaccessible_frontiers) {
        if (distanceBetween(goal, inaccessible) < 1.0) {
            return false;
        }
    }
    
    return true;
}

bool Explorer::isPointInMap(const geometry_msgs::Point& point) {
    double min_x = map.info.origin.position.x;
    double max_x = min_x + map.info.width * map.info.resolution;
    double min_y = map.info.origin.position.y;
    double max_y = min_y + map.info.height * map.info.resolution;
    
    return point.x >= min_x && point.x <= max_x && point.y >= min_y && point.y <= max_y;
}

double Explorer::distanceBetween(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

void Explorer::publishGoal(const geometry_msgs::Point& goal) {
    geometry_msgs::PoseStamped goal_msg;
    goal_msg.header.frame_id = "map";
    goal_msg.header.stamp = ros::Time::now();
    goal_msg.pose.position = goal;
    goal_msg.pose.orientation.w = 1.0; // Facing forward

    goal_pub.publish(goal_msg);
    ROS_INFO("Published new goal at (%.2f, %.2f)", goal.x, goal.y);
}

std::vector<FrontierCell> Explorer::detectFrontiers(const nav_msgs::OccupancyGrid& grid){
    std::vector<FrontierCell> frontiers;

    for (int y = 1; y < grid.info.height - 1; y++) {
        for (int x = 1; x < grid.info.width - 1; x++) {
            if (isFrontierCell(grid, x, y)) {
                double world_x, world_y;
                gridToWorld(grid, x, y, world_x, world_y);
                frontiers.emplace_back(x, y, world_x, world_y);
            }
        }
    }
    return frontiers;
}

std::vector<FrontierCluster> Explorer::clusterFrontiers(const std::vector<FrontierCell>& frontiers) {
    std::vector<FrontierCluster> clusters;
    std::vector<bool> visited(frontiers.size(), false);

    for (size_t i = 0; i < frontiers.size(); ++i) {
        if (visited[i]) continue;

        FrontierCluster cluster;
        std::queue<size_t> q;
        q.push(i);
        visited[i] = true;

        while (!q.empty()) {
            size_t idx = q.front(); q.pop();
            cluster.cells.push_back(frontiers[idx]);

            for (size_t j = 0; j < frontiers.size(); ++j) {
                if (!visited[j]) {
                    double dist_sq = pow(frontiers[idx].world_x - frontiers[j].world_x, 2) + pow(frontiers[idx].world_y - frontiers[j].world_y, 2);
                    if (dist_sq < frontier_search_radius * frontier_search_radius) {
                        visited[j] = true;
                        q.push(j);
                    }
                }
            }
        }

        if ((int)cluster.cells.size() >= min_frontier_size) {
            double sum_x = 0, sum_y = 0;
            for (const auto& cell : cluster.cells) {
                sum_x += cell.world_x;
                sum_y += cell.world_y;
            }
            cluster.centroid.x = sum_x / cluster.cells.size();
            cluster.centroid.y = sum_y / cluster.cells.size();
            cluster.centroid.z = 0;
            cluster.size = cluster.cells.size();
            cluster.distance_to_robot = distanceToRobot(cluster.centroid);
            clusters.push_back(cluster);
        }
    }

    return clusters;
}

bool Explorer::isFrontierAccessible(const geometry_msgs::Point& frontier) {
    for (const auto& inaccessible : inaccessible_frontiers) {
        double dist = sqrt(pow(frontier.x - inaccessible.x, 2) + pow(frontier.y - inaccessible.y, 2));
        // 2 meters buffer
        if (dist < 2) {
            return false;
        }
    }
    return true;
}

bool Explorer::isFrontierCell(const nav_msgs::OccupancyGrid& grid, int x, int y) {
    if (!isFree(grid, x, y)) return false;

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

bool Explorer::isFree(const nav_msgs::OccupancyGrid& grid, int x, int y) {
    if (!isValidCell(grid, x, y)) return false;
    int idx = y * grid.info.width + x;
    return grid.data[idx] >= 0 && grid.data[idx] < 50;
}

bool Explorer::isUnknown(const nav_msgs::OccupancyGrid& grid, int x, int y) {
    if (!isValidCell(grid, x, y)) return false;
    int idx = y * grid.info.width + x;
    return grid.data[idx] == -1;
}

bool Explorer::isOccupied(const nav_msgs::OccupancyGrid& grid, int x, int y) {
    if (!isValidCell(grid, x, y)) return false;
    int idx = y * grid.info.width + x;
    return grid.data[idx] >= 50;
}

bool Explorer::isValidCell(const nav_msgs::OccupancyGrid& grid, int x, int y) {
    return x >= 0 && x < (int)grid.info.width && y >= 0 && y < (int)grid.info.height;
}

void Explorer::gridToWorld(const nav_msgs::OccupancyGrid& grid, int grid_x, int grid_y, double& world_x, double& world_y) {
    world_x = grid.info.origin.position.x + (grid_x + 0.5) * grid.info.resolution;
    world_y = grid.info.origin.position.y + (grid_y + 0.5) * grid.info.resolution;
}

FrontierCluster Explorer::selectBestFrontier(const std::vector<FrontierCluster>& clusters) {
    FrontierCluster best_cluster;
    double best_cost = std::numeric_limits<double>::max();
    const double w_dist = 0.5; 
    const double w_energy = 1.0;
    const double w_info = 3.5;

    for (const auto& cluster : clusters) {
        if (cluster.distance_to_robot > max_exploration_distance) {
            continue;
        }

        // Travel Distance Cost: normalized path length
        double Cdist = cluster.distance_to_robot / max_exploration_distance;

        
        // Expected Information Gain: using natural log of cluster size as a proxy
        double I_gain = std::log(static_cast<double>(cluster.size));
        
        // Total cost: lower value is preferred
        double cost = w_dist * Cdist + w_energy * cluster.distance_to_robot - w_info * I_gain;

        // Map boundary penalty: discourage clusters near map boundaries
        double map_min_x = map.info.origin.position.x;
        double map_max_x = map_min_x + map.info.width * map.info.resolution;
        double map_min_y = map.info.origin.position.y;
        double map_max_y = map_min_y + map.info.height * map.info.resolution;
        double boundary_margin = 1.0; // meters
        if (cluster.centroid.x < map_min_x + boundary_margin ||
            cluster.centroid.x > map_max_x - boundary_margin ||
            cluster.centroid.y < map_min_y + boundary_margin ||
            cluster.centroid.y > map_max_y - boundary_margin) {
            cost += 100.0; // high penalty
        }

        if (cost < best_cost) {
            best_cost = cost;
            best_cluster = cluster;
        }
    }
    return best_cluster;
}
