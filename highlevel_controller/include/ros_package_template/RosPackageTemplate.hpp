#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>
#include <std_srvs/Trigger.h>
#include <string>

#include "highlevel_controller/Algorithm.hpp"


namespace highlevel_controller {

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
	void topicCallback(const sensor_msgs::Temperature& message);

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

	//! ROS service server.
	ros::ServiceServer serviceServer_;

	//! Algorithm computation object.
	Algorithm algorithm_;
};

} /* namespace */
