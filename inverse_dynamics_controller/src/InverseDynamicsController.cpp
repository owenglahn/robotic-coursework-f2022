#include "inverse_dynamics_controller/InverseDynamicsController.hpp"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include "inverse_dynamics_controller/move_to.h"
#include <ros/ros.h>
#include <iostream>
#include <math.h>

namespace inverse_dynamics_controller {

InverseDynamicsController::InverseDynamicsController(ros::NodeHandle& nodeHandle) 
	: nodeHandle_(nodeHandle) {
	std::cout << "Initializing IDC" << std::endl;
	if ( !readParameters() ) {
		ROS_ERROR("Could not read parameters.");
		ros::requestShutdown();
	}
	std::cout << "read params" << std::endl;
	potentialField = 
		new PotentialField(urdfFileName, k_attr, k_task, k_joint, d_attr, d_task, d_joint);
	// prepare all publishers, subscribers, servers, clients, etc
	subscriber_ 	= nodeHandle_.subscribe(subscriberTopic_, 1, &InverseDynamicsController::topicCallback, this);
	serviceServer_ 	= nodeHandle_.advertiseService(serviceName_, &InverseDynamicsController::serviceCallback, this);
	publisher_ = nodeHandle_.advertise<std_msgs::Float64MultiArray>(publisherTopic_, 2);
	std::cout << "created sub, pub and serv" << std::endl;
	joint_torque_limits = Eigen::VectorXd::Zero(7);
	joint_torque_limits << 39, 39, 39, 39, 9, 9, 9; 
	t_total = Eigen::VectorXd::Zero(7);

	ROS_INFO("Successfully launched node.");
}

InverseDynamicsController::~InverseDynamicsController() {
	delete potentialField;
}

bool InverseDynamicsController::readParameters() {
	std::cout << "Reading params" << std::endl;
	if ( !nodeHandle_.getParam("/inverse_dynamics_controller/subscriber_topic", subscriberTopic_ )) {
		return false ;
	}
	if (!nodeHandle_.getParam("/inverse_dynamics_controller/urdf_file", urdfFileName) ||
		!nodeHandle_.getParam("/inverse_dynamics_controller/advertiser_service", serviceName_) || 
		!nodeHandle_.getParam("/inverse_dynamics_controller/publisher_topic", publisherTopic_) ||
		!nodeHandle_.getParam("/inverse_dynamics_controller/k_attr", k_attr) || 
		!nodeHandle_.getParam("/inverse_dynamics_controller/k_task", k_task) || 
		!nodeHandle_.getParam("/inverse_dynamics_controller/k_joint", k_joint) || 
		!nodeHandle_.getParam("/inverse_dynamics_controller/d_task", d_task) || 
		!nodeHandle_.getParam("/inverse_dynamics_controller/d_joint", d_joint) || 
		!nodeHandle_.getParam("/inverse_dynamics_controller/d_attr", d_attr)) {

		return false;
	}
	return true;
}

void InverseDynamicsController::update() {
	// update whatever algorithms
	potentialField -> update();
	potentialField -> update_joint_refs();
	t_total = potentialField -> get_task_torque_all_terms() + 
		potentialField -> get_P() * potentialField -> get_joint_torque_all_terms();
	limit_joint_torque(t_total);
	if (isnan(t_total[0])) {
		std::cout << "task torque\n" << potentialField -> get_task_torque_all_terms() << 
			"P\n" << potentialField -> get_P() << "joint torque\n" <<  potentialField -> get_joint_torque_all_terms()
			 << std::endl;
		ros::shutdown();
	}
	std::cout << "torque\n" << t_total << std::endl;
	std_msgs::Float64MultiArray toSend;	
	toSend.data.insert(toSend.data.end(), t_total.data(), t_total.data() + 7);
	publisher_.publish(toSend);
}

void InverseDynamicsController::topicCallback(const sensor_msgs::JointState& joint_state) {
	// do something here
	potentialField -> update_joints(joint_state);	
}

void InverseDynamicsController::limit_joint_torque(Eigen::VectorXd& torque) {
	// Eigen::VectorXd ratios = torque;
	// for (int i = 0; i < ratios.rows(); i++) {
	// 	ratios[i] = std::abs(ratios[i]) / joint_torque_limits[i]; 
	// }
	// double max_ratio = *std::max_element(ratios.data(), ratios.data() + 7);
	// if (max_ratio > 1.0) {
	// 	for (int i = 0; i < torque.rows(); i++) {
	// 		torque[i] = torque[i] / max_ratio;
	// 	}
	// }
	for (int i = 0; i < 7; i ++) {
		torque[i] = torque[i] / std::abs(torque[i]) * 
			std::min(std::abs(torque[i]), joint_torque_limits[i]);
	}
}

bool InverseDynamicsController::serviceCallback(
	inverse_dynamics_controller::move_to::Request& request, 
	inverse_dynamics_controller::move_to::Response& response) {

	potentialField -> target_pos[0] = request.x;
	potentialField -> target_pos[1] = request.y;
	potentialField -> target_pos[2] = request.z;
	return true;
}

} /* namespace */
