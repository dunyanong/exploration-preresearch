#ifndef Vehicle_H
#define Vehicle_H

#include <cmath>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

struct Vehicle {
  // basic info
  std::string name;

  // kinematic info
  geometry_msgs::Pose current_pose;
  geometry_msgs::Twist current_velocity;
  nav_msgs::Path current_path;
  int current_index;

  // ros-related
  ros::Subscriber odom_sub;
  ros::Subscriber path_sub;
  ros::Publisher cmd_vel_pub;
  ros::Subscriber adjust_sub;


  // STATUS
  std::string status = "IN";

  // control parameters
  double kp_linear;        // 线性速度的P
  double kp_angular;       // 角速度的P
  double kp_angular_small; // 小角度时的P
  double ki_linear;        // 线性速度的I
  double ki_angular;       // 角速度的I
  double tolerance;
  double max_linear_velocity, max_angular_velocity;
  
  // integral error accumulation
  double integral_linear_error;
  double integral_angular_error;

  Vehicle() = default;
  Vehicle(const std::string &name_)
      : name(name_), kp_linear(5.0), kp_angular(8.0), kp_angular_small(3.0),
        ki_linear(0.2), ki_angular(0.1), tolerance(0.3), max_linear_velocity(5.0), 
        max_angular_velocity(6.0), integral_linear_error(0.0), integral_angular_error(0.0) {}

  void send_cmd_vel(double v, double w) {
    geometry_msgs::Twist target_velocity;
    target_velocity.linear.x = v;
    target_velocity.angular.z = w;
    cmd_vel_pub.publish(target_velocity);
  }
};

#endif