#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>
#include <std_srvs/Trigger.h>
#include <string>

#include "ros_package_template/Algorithm.hpp"


namespace ros_package_template {

/*!
 * The ROS wrapper for Algorithm
 */
class RosPackageTemplate {

	public:
	/*!
	* Constructor.
	* @param nodeHandle the ROS node handle.
	*/
	RosPackageTemplate(ros::NodeHandle& nodeHandle);

  	/*!
  	 * Destructor.
  	 */
	virtual ~RosPackageTemplate();

	/*
	 * A function handling necessary actions in every loop
	 */
	void update() ;

	private:
	/*!
	* Reads and verifies the ROS parameters.
	* @return true if successful.
	*/
	bool readParameters();

	/*!
	* ROS topic callback method.
	* @param message the received message.
	*/
	void topicCallback(const sensor_msgs::JointState& joint_state);

	/*!
	* ROS service server callback.
	* @param request the request of the service.
	* @param response the provided response.
	* @return true if successful, false otherwise.
	*/
	bool serviceCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);

	//! ROS node handle.
	ros::NodeHandle& nodeHandle_;

	//! ROS topic subscriber.
	ros::Subscriber subscriber_ ;

	//! ROS topic name to subscribe to.
	std::string subscriberTopic_ ;

	ros::Publisher publisher_;
	std::string publisherTopic_;
	std::string urdfFileName;

	//! ROS service server.
	ros::ServiceServer serviceServer_;
	std::string serviceName_;

	//! Algorithm computation object.
	Algorithm algorithm_;
};

} /* namespace */
