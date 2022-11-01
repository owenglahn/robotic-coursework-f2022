#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include "highlevel_controller/cubic_poly_planner.hpp"

namespace highlevel_controller {
Eigen::Vector3d get_position(ros::Duration t, ros::Duration target_time,
	Eigen::Vector3d start_pos, Eigen::Vector3d target_pos) {
	double s_t = 3/pow(target_time.toSec(), 2) * pow(t.toSec(), 2) 
		- 2/pow(target_time.toSec(), 3) * pow(t.toSec(), 3);
	ROS_INFO("time scale=%f", s_t);
	return start_pos + s_t * (target_pos - start_pos);
}
}
