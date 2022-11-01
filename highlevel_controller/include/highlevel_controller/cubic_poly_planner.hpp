#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <ros/ros.h>

namespace highlevel_controller {
Eigen::Vector3d get_position(ros::Duration t, ros::Duration target_time,
	Eigen::Vector3d start_pos, Eigen::Vector3d target_pos);
}