#include "inverse_dynamics_controller/InverseDynamicsController.hpp"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include "inverse_dynamics_controller/move_to.h"
#include <ros/ros.h>

namespace inverse_dynamics_controller {

InverseDynamicsController::InverseDynamicsController(ros::NodeHandle& nodeHandle) 
	: nodeHandle_(nodeHandle) {
	std::cout << "Initializing IDC" << std::endl;
	if ( !readParameters() ) {
		std::cout << "failed to read params" << std::endl;
		ROS_ERROR("Could not read parameters.");
		ros::requestShutdown();
	}
	std::cout << "read params" << std::endl;
	potentialFieldTask.urdfFileName = urdfFileName;
	potentialFieldJoint.urdfFileName = urdfFileName;
	// prepare all publishers, subscribers, servers, clients, etc
	subscriber_ 	= nodeHandle_.subscribe(subscriberTopic_, 1, &InverseDynamicsController::topicCallback, this);
	serviceServer_ 	= nodeHandle_.advertiseService(serviceName_, &InverseDynamicsController::serviceCallback, this);
	publisher_ = nodeHandle_.advertise<std_msgs::Float64MultiArray>(publisherTopic_, 2);
	std::cout << "created sub, pub and serv" << std::endl;

	ROS_INFO("Successfully launched node.");
}

InverseDynamicsController::~InverseDynamicsController() {
}

bool InverseDynamicsController::readParameters() {
	std::cout << "Reading params" << std::endl;
	if ( !nodeHandle_.getParam("subscriber_topic", subscriberTopic_ )) {
		return false ;
	}
	if (!nodeHandle_.getParam("urdf_file", urdfFileName) ||
		!nodeHandle_.getParam("advertiser_service", serviceName_)) {
		return false;
	}
	return true;
}

void InverseDynamicsController::update() {
	std::cout << "Calling update on controller" << std::endl;
	// update whatever algorithms
	potentialFieldTask.update();
	potentialFieldJoint.update_joint_refs();

}

void InverseDynamicsController::topicCallback(const sensor_msgs::JointState& joint_state) {
	// do something here
	potentialFieldTask.update_joints(joint_state);	
	potentialFieldJoint.update_joints(joint_state);
}

bool InverseDynamicsController::serviceCallback(
	inverse_dynamics_controller::move_to::Request& request, 
	inverse_dynamics_controller::move_to::Response& response) {

	Eigen::VectorXd t_total = potentialFieldTask.get_task_torque_all_terms() + 
		potentialFieldJoint.get_P() * potentialFieldJoint.get_joint_torque_all_terms();
	std_msgs::Float64MultiArray toSend;	
	for (int i = 0; i < 7; i++) {
		toSend.data[i] = t_total[i];
	}
	publisher_.publish(toSend);
	return true;
}

} /* namespace */
