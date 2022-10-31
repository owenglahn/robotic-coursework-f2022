#include "ros_package_template/RosPackageTemplate.hpp"

namespace ros_package_template {

RosPackageTemplate::RosPackageTemplate(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle) {
	if ( !readParameters() ) {
		ROS_ERROR("Could not read parameters.");
		ros::requestShutdown();
	}

	// prepare all publishers, subscribers, servers, clients, etc
	subscriber_ 	= nodeHandle_.subscribe(subscriberTopic_, 1, &RosPackageTemplate::topicCallback, this);
	serviceServer_ 	= nodeHandle_.advertiseService("get_average", &RosPackageTemplate::serviceCallback, this);

	ROS_INFO("Successfully launched node.");
}

RosPackageTemplate::~RosPackageTemplate() {
}

bool RosPackageTemplate::readParameters() {
	if ( !nodeHandle_.getParam("subscriber_topic", subscriberTopic_ )) {
		return false ;
	}
	return true;
}

void RosPackageTemplate::update() {

	// update whatever algorithms
	algorithm_.update() ;
}

void RosPackageTemplate::topicCallback(const sensor_msgs::Temperature& message) {
	// do something here

}

bool RosPackageTemplate::serviceCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
	response.success = true;
	response.message = "ServiceCallback: done" ;
	return true;
}

} /* namespace */
