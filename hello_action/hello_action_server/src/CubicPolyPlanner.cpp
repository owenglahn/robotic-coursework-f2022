#include "hello_action_server/CubicPolyPlanner.hpp"

CubicPolyPlanner::CubicPolyPlanner() {
	this -> target_pos = Eigen::Vector3d::Zero();
	this -> start_pos = Eigen::Vector3d::Zero();
}

Eigen::Vector3d CubicPolyPlanner::get_position() {
	double s_t = 3/pow(target_time.toSec(), 2) * pow(t.toSec(), 2) 
		- 2/pow(target_time.toSec(), 3) * pow(t.toSec(), 3);
	ROS_INFO("time scale=%f", s_t);
	return start_pos + s_t * (target_pos - start_pos);
}

void CubicPolyPlanner::update() {
	t = ros::Time::now() - start_time;
}

void CubicPolyPlanner::set_target(ros::Duration target_time, Eigen::Vector3d target_pos,
	Eigen::Vector3d start_pos) {
	this -> target_time = target_time;
	this -> target_pos = target_pos;
	this -> start_pos = start_pos;
	this -> start_time = ros::Time::now();
}