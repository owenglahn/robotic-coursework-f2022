#include "ros_package_template/RosPackageTemplate.hpp"

namespace ros_package_template {

RosPackageTemplate::RosPackageTemplate(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle) {
	if ( !readParameters() ) {
		ROS_ERROR("Could not read parameters.");
		ros::requestShutdown();
	}
	algorithm_.urdfFileName = urdfFileName;
	// prepare all publishers, subscribers, servers, clients, etc
	subscriber_ 	= nodeHandle_.subscribe(subscriberTopic_, 1, &RosPackageTemplate::topicCallback, this);
	serviceServer_ 	= nodeHandle_.advertiseService(serviceName, &RosPackageTemplate::serviceCallback, this);
	publisher_ = nodeHandle_.advertise<std_msgs::Float64MultiArray>(publisherName_, 2);

	ROS_INFO("Successfully launched node.");
}

RosPackageTemplate::~RosPackageTemplate() {
}

bool RosPackageTemplate::readParameters() {
	if ( !nodeHandle_.getParam("subscriber_topic", subscriberTopic_ )) {
		return false ;
	}
	return nodeHandle_.getParam("urdf_file", urdfFileName) &&
		nodeHandle.getParam("advertising_service", serviceName);
}

void RosPackageTemplate::update() {
	// update whatever algorithms
	algorithm_.update() ;
}

void RosPackageTemplate::topicCallback(const sensor_msgs::JointState& joint_state) {
	// do something here

}

bool RosPackageTemplate::serviceCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
	response.success = true;
	response.message = "ServiceCallback: done" ;
	return true;
}

} /* namespace */
