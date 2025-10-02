#include "trajectory_tracker.h"
#include <boost/bind.hpp>
#include <cmath>
#include <functional>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <tf/transform_datatypes.h>
#include <vector>

TrajectoryTracker::TrajectoryTracker(ros::NodeHandle &nh_) : nh(nh_) {
  robot_names = {"agv1"};
  for (auto name : robot_names) {
    vehicles[name] = Vehicle(name);
    // subscribe to odom
    vehicles[name].odom_sub = nh.subscribe<nav_msgs::Odometry>(
        "/" + name + "/odom", 10,
        boost::bind(&TrajectoryTracker::odom_callback, this,
                    boost::placeholders::_1, name));
    // subscribe to path
    vehicles[name].path_sub = nh.subscribe<nav_msgs::Path>(
        "/" + name + "/local_path", 10,
        boost::bind(&TrajectoryTracker::path_callback, this,
                    boost::placeholders::_1, name));
    // publisher cmd_vel
    vehicles[name].cmd_vel_pub =
        nh.advertise<geometry_msgs::Twist>("/" + name + "/cmd_vel", 10);
    vehicles[name].adjust_sub = nh.subscribe<std_msgs::String>(
        "/" + name + "/adjust_pose", 1,
        boost::bind(&TrajectoryTracker::adjust_callback, this,
                    boost::placeholders::_1, name));
   
  }
}

void TrajectoryTracker::adjust_callback(const std_msgs::String::ConstPtr &msg,
                                        const std::string &name) {
  Vehicle &r = vehicles[name];
  if (r.status != msg->data) {
    r.status = msg->data;
  }
}

void TrajectoryTracker::odom_callback(const nav_msgs::Odometry::ConstPtr &msg,
                                      const std::string &name) {
  Vehicle &r = vehicles[name];
  r.current_pose = msg->pose.pose;

  if (r.status != "IN") {
    if (r.status == "LEFT") {
      r.send_cmd_vel(0, 1.0);
    } else if (r.status == "RIGHT") {
      r.send_cmd_vel(0, -1.0);
    }
    return;
  }

  // 路径为空时，暂停运动
  if (r.current_path.poses.empty()) {
    r.send_cmd_vel(0, 0);
    return;
  }
  // 限制idx
  int idx = r.current_index;
  if (idx >= static_cast<int>(r.current_path.poses.size()))
    idx = r.current_path.poses.size() - 1;
  // 计算距离
  double dx =
      r.current_path.poses[idx].pose.position.x - r.current_pose.position.x;
  double dy =
      r.current_path.poses[idx].pose.position.y - r.current_pose.position.y;
  double distance = sqrt(dx * dx + dy * dy);
  // 计算特别案例
  if (distance < r.tolerance &&
      idx < static_cast<int>(r.current_path.poses.size()) - 1) {
    r.current_index++;
    // 重置积分误差当切换到新waypoint时
    r.integral_linear_error = 0.0;
    r.integral_angular_error = 0.0;
    idx = r.current_index;
    dx = r.current_path.poses[idx].pose.position.x - r.current_pose.position.x;
    dy = r.current_path.poses[idx].pose.position.y - r.current_pose.position.y;
    distance = sqrt(dx * dx + dy * dy);
  } else if (distance < r.tolerance &&
             idx == static_cast<int>(r.current_path.poses.size()) - 1) {
    //r.send_cmd_vel(0, 0);
    r.integral_linear_error = 0.0;
    r.integral_angular_error = 0.0;
    return;
  }

  // 计算目标点的朝向
  double target_angle = std::atan2(dy, dx);

  // 从四元数转换得到当前航向角
  tf::Quaternion q;
  tf::quaternionMsgToTF(r.current_pose.orientation, q);
  double roll, pitch, current_yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, current_yaw);

  // 计算误差角
  double error_angle = target_angle - current_yaw;
  while (error_angle > M_PI)
    error_angle -= 2 * M_PI;
  while (error_angle < -M_PI)
    error_angle += 2 * M_PI;

  // 更新积分误差
  r.integral_linear_error += distance * 0.1; // 假设控制周期为0.1s
  r.integral_angular_error += error_angle * 0.1;
  
  // 限制积分累积防止积分饱和
  double max_integral = 2.0;
  if (r.integral_linear_error > max_integral) r.integral_linear_error = max_integral;
  if (r.integral_linear_error < -max_integral) r.integral_linear_error = -max_integral;
  if (r.integral_angular_error > max_integral) r.integral_angular_error = max_integral;
  if (r.integral_angular_error < -max_integral) r.integral_angular_error = -max_integral;

  // 计算PI控制器输出
  double linear_vel, angular_vel;
  if (fabs(error_angle) >= M_PI / 6) {
    linear_vel = r.kp_linear * distance + r.ki_linear * r.integral_linear_error;
    angular_vel = r.kp_angular * error_angle + r.ki_angular * r.integral_angular_error;
  } else {
    linear_vel = r.kp_linear * distance + r.ki_linear * r.integral_linear_error;
    angular_vel = r.kp_angular_small * error_angle + r.ki_angular * r.integral_angular_error;
  }

  // 当角度误差过大时停止线性运动并重置积分项
  if (fabs(error_angle) >= M_PI / 2) {
    linear_vel = 0;
    r.integral_linear_error = 0; // 重置积分项
  }


  // 限制最大速度
  if (linear_vel > r.max_linear_velocity)
    linear_vel = r.max_linear_velocity;
  if (angular_vel > r.max_angular_velocity)
    angular_vel = r.max_angular_velocity;
  if (angular_vel < -r.max_angular_velocity)
    angular_vel = -r.max_angular_velocity;

  // 发送速度
  r.send_cmd_vel(linear_vel, angular_vel);
}

void TrajectoryTracker::path_callback(const nav_msgs::Path::ConstPtr &msg,
                                      const std::string &name) {
  Vehicle &r = vehicles[name];
  // 清除旧path和重置积分误差
  r.current_path.poses.clear();
  r.integral_linear_error = 0.0;
  r.integral_angular_error = 0.0;

  // 空path处理
  if (msg->poses.empty()) {
    r.send_cmd_vel(0, 0);
    return;
  }

  // 插值
  for (size_t i = 0; i < msg->poses.size() - 1; ++i) {
    r.current_path.poses.push_back(msg->poses[i]);

    geometry_msgs::PoseStamped middle_pose;
    middle_pose.pose.position.x =
        (msg->poses[i].pose.position.x + msg->poses[i + 1].pose.position.x) /
        2.0;
    middle_pose.pose.position.y =
        (msg->poses[i].pose.position.y + msg->poses[i + 1].pose.position.y) /
        2.0;
    middle_pose.pose.position.z =
        (msg->poses[i].pose.position.z + msg->poses[i + 1].pose.position.z) /
        2.0;
    middle_pose.pose.orientation = msg->poses[i + 1].pose.orientation;
    r.current_path.poses.push_back(middle_pose);
  }
  r.current_index = 0;
}