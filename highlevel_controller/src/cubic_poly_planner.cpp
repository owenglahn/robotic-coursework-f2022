#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include "highlevel_controller/cubic_poly_planner.hpp"

namespace highlevel_controller {
Eigen::VectorXd get_position(ros::Duration t, ros::Duration target_time,
	Eigen::VectorXd start_pos, Eigen::VectorXd target_pos) {
	double s_t = 3/pow(target_time.toSec(), 2) * pow(t.toSec(), 2) 
		- 2/pow(target_time.toSec(), 3) * pow(t.toSec(), 3);
	return start_pos + s_t * (target_pos - start_pos);
}
}
