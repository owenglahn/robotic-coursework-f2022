#pragma once

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <hello_action_type/MoveToAction.h>

#include <gazebo_msgs/ModelStates.h>
#include "hello_action_server/CubicPolyPlanner.hpp"

class ActionServer {

	public:
	/*!
	* Constructor.
	* @param node_handle_ the ROS node handle.
	*/
	ActionServer(ros::NodeHandle& node_handle);

  	/*!
  	 * Destructor.
  	 */
	virtual ~ActionServer();

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

	void move_to_ballback(const hello_action_type::MoveToGoalConstPtr& goal) ;
	void feedbackCallback(const gazebo_msgs::ModelStates& fbk_pose_msg) ;

	//! ROS node handle.
	ros::NodeHandle& node_handle_;

	//! ROS topic subscriber.
	ros::Subscriber feedback_subscriber_ ;

	//! ROS topic name to subscribe to.
	std::string subscriberTopic_ ;

	//! ROS service server.
	ros::ServiceServer serviceServer_;

	hello_action_type::MoveToGoal action_goal_ ;

	actionlib::SimpleActionServer<hello_action_type::MoveToAction> move_to_action_server_ ;

	gazebo_msgs::ModelStates feedback_state_ ;

	hello_action_type::MoveToFeedback move_to_feedback_ ;

	ros::Rate loop_rate_ 	;
	//! Algorithm computation object.

	CubicPolyPlanner cubicPolyPlanner_;
};


