#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <ros/ros.h>

namespace highlevel_controller {
Eigen::VectorXd get_position(ros::Duration t, ros::Duration target_time,
	Eigen::VectorXd start_pos, Eigen::VectorXd target_pos);
}