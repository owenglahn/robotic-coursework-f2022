#pragma once

#include <sensor_msgs/JointState.h>
#include <string>

#include "inverse_dynamics_controller/PotentialField.hpp"
#include "inverse_dynamics_controller/move_to.h"
#include <ros/ros.h>

namespace inverse_dynamics_controller {

/*!
 * The ROS wrapper for PotentialField
 */
class InverseDynamicsController {

	public:
	/*!
	* Constructor.
	* @param nodeHandle the ROS node handle.
	*/
	InverseDynamicsController(ros::NodeHandle& nodeHandle);

  	/*!
  	 * Destructor.
  	 */
	virtual ~InverseDynamicsController();

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
	bool serviceCallback(inverse_dynamics_controller::move_to::Request& request, 
		inverse_dynamics_controller::move_to::Response& response);

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

	//! PotentialField computation object.
	PotentialField* potentialFieldTask;
	PotentialField* potentialFieldJoint;
	double k_scale;
	double d_scale;
};

} /* namespace */
